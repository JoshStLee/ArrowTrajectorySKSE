#include "pch.h"
#include "trajectory.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace
{
    struct PhysicsVector
    {
        float x;
        float y;
        float z;

        PhysicsVector operator+(const PhysicsVector& rhs) const { return { x + rhs.x, y + rhs.y, z + rhs.z }; }
        PhysicsVector operator-(const PhysicsVector& rhs) const { return { x - rhs.x, y - rhs.y, z - rhs.z }; }
        PhysicsVector operator*(float scalar) const { return { x * scalar, y * scalar, z * scalar }; }
        PhysicsVector& operator+=(const PhysicsVector& rhs)
        {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }
    };

    using CameraUpdate = void (*)(RE::PlayerCamera*);

    CameraUpdate g_originalCameraUpdate = nullptr;
    RE::GFxMovieView* g_hudMovie = nullptr;
    RE::GFxValue g_trajectoryClip;
    double g_crosshairBaseX = 0.0;
    double g_crosshairBaseY = 0.0;
    bool g_crosshairBaseCaptured = false;
    bool g_reportedDrawingFailure = false;

    constexpr float kStep = 1.0f / 20.0f;
    constexpr float kMaxRange = 12000.0f;
    constexpr std::size_t kMaxSegments = 128;
    constexpr float kLinearDamping = 0.099609f;
    constexpr float kGravityScale = 59.0f;
    constexpr float kMinBowPower = 0.2f;

    struct BowDrawTimerEntry
    {
        float drawTime;
        float unknown[3];
    };

    PhysicsVector ToVector(const RE::NiPoint3& point)
    {
        return { point.x, point.y, point.z };
    }

    RE::NiPoint3 ToNi(const PhysicsVector& point)
    {
        return { point.x, point.y, point.z };
    }

    class FirstHitCollector final : public RE::hkpClosestRayHitCollector
    {
    public:
        explicit FirstHitCollector(const RE::NiAVObject* ignoredObject) :
            ignoredObject(ignoredObject)
        {}

        void AddRayHit(const RE::hkpCdBody& body, const RE::hkpShapeRayCastCollectorOutput& hitInfo) override
        {
            if (hitInfo.hitFraction >= rayHit.hitFraction) {
                return;
            }

            const auto bodyRoot = [&body] {
                auto result = &body;
                while (result->parent) {
                    result = result->parent;
                }
                return result;
            }();

            const auto collidable = static_cast<const RE::hkpCollidable*>(bodyRoot);
            const auto filter = collidable->broadPhaseHandle.collisionFilterInfo.filter;
            if (filter >= 64 || ((1ULL << filter) & 0x40122716ULL) == 0) {
                return;
            }

            using GetAVObject = RE::NiAVObject* (*)(const RE::hkpCdBody*);
            static REL::Relocation<GetAVObject> getAVObject{ REL::VariantID(76160, 77988, 0) };
            const auto avObject = getAVObject(bodyRoot);
            if (!avObject || avObject == ignoredObject) {
                return;
            }

            earlyOutHitFraction = hitInfo.hitFraction;
            rayHit.hitFraction = hitInfo.hitFraction;
            rayHit.normal = hitInfo.normal;
            rayHit.rootCollidable = collidable;
        }

        const RE::NiAVObject* ignoredObject;
    };

    float Dot(const PhysicsVector& lhs, const PhysicsVector& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    float Length(const PhysicsVector& value)
    {
        return std::sqrt(Dot(value, value));
    }

    PhysicsVector Normalize(const PhysicsVector& value)
    {
        const auto length = Length(value);
        return length > 0.0001f ? value * (1.0f / length) : PhysicsVector{};
    }

    std::optional<PhysicsVector> CastRay(
        RE::PlayerCharacter* player,
        const PhysicsVector& start,
        const PhysicsVector& end)
    {
        const auto cell = player->GetParentCell();
        const auto world = cell ? cell->GetbhkWorld() : nullptr;
        if (!world) {
            return std::nullopt;
        }

        const auto delta = end - start;
        const auto scale = RE::bhkWorld::GetWorldScaleInverse();
        RE::bhkPickData pickData{};
        pickData.rayInput.from = RE::hkVector4(start.x * scale, start.y * scale, start.z * scale, 1.0f);
        pickData.rayInput.to = RE::hkVector4(0.0f, 0.0f, 0.0f, 0.0f);
        pickData.ray = RE::hkVector4(delta.x * scale, delta.y * scale, delta.z * scale, 1.0f);

        const auto player3D = player->Get3D();
        FirstHitCollector collector(player3D);
        collector.Reset();
        pickData.closestRayHitCollector = &collector;
        world->PickObject(pickData);

        if (!collector.HasHit()) {
            return std::nullopt;
        }
        return start + delta * collector.rayHit.hitFraction;
    }

    bool IsCombatSpell(const RE::MagicItem* spell)
    {
        return spell &&
            spell->GetDelivery() != RE::MagicSystem::Delivery::kSelf &&
            spell->GetCastingType() != RE::MagicSystem::CastingType::kConstantEffect;
    }

    bool IsBowOrCrossbow(const RE::Actor* actor)
    {
        if (!actor->IsWeaponDrawn()) {
            return false;
        }
        const auto weapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(false));
        if (!weapon) {
            return false;
        }
        const auto type = weapon->weaponData.animationType;
        return type == RE::WEAPON_TYPE::kBow || type == RE::WEAPON_TYPE::kCrossbow;
    }

    bool IsCombatMagicDrawn(const RE::Actor* actor)
    {
        if (!actor->IsWeaponDrawn()) {
            return false;
        }

        const auto& selectedSpells = actor->GetActorRuntimeData().selectedSpells;
        const auto leftSpell = selectedSpells[RE::Actor::SlotTypes::kLeftHand];
        const auto rightSpell = selectedSpells[RE::Actor::SlotTypes::kRightHand];
        if (!IsCombatSpell(leftSpell) && !IsCombatSpell(rightSpell)) {
            return false;
        }

        const auto leftWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(true));
        const auto rightWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(false));
        return (!leftWeapon || leftWeapon->weaponData.animationType == RE::WEAPON_TYPE::kStaff) ||
            (!rightWeapon || rightWeapon->weaponData.animationType == RE::WEAPON_TYPE::kStaff);
    }

    float GetBowDrawStrength(const RE::PlayerCharacter* player)
    {
        const auto& timerArray = player->GetPlayerRuntimeData().unkBA0;
        if (timerArray.size() == 0) {
            return 1.0f;
        }

        const auto* entries = reinterpret_cast<const BowDrawTimerEntry*>(timerArray.data());
        const auto drawTime = entries[timerArray.size() - 1].drawTime;
        if (!std::isfinite(drawTime)) {
            return 1.0f;
        }
        const auto minDrawTime = *REL::Relocation<float*>{ REL::RelocationID(505070, 374232) };
        const auto minPower = *REL::Relocation<float*>{ REL::RelocationID(505072, 374235) };
        const auto remapped = ((drawTime - minDrawTime) / (1.0f - minDrawTime)) * (1.0f - minPower) + minPower;
        return std::clamp(remapped, kMinBowPower, 1.0f);
    }

    RE::NiAVObject* FindArrowNode(RE::Actor* actor)
    {
        const auto root = actor->Get3D();
        if (!root) {
            return nullptr;
        }

        const RE::BSFixedString weaponName("WEAPON");
        const RE::BSFixedString arrowName("Arrow:0");
        const RE::BSFixedString rightMagazineName("NPC R MagicNode [RMag]");

        if (auto* weapon = root->GetObjectByName(weaponName)) {
            if (auto* arrow = weapon->GetObjectByName(arrowName)) {
                return arrow;
            }
        }
        if (auto* magazine = root->GetObjectByName(rightMagazineName)) {
            if (auto* arrow = magazine->GetObjectByName(arrowName)) {
                return arrow;
            }
        }
        return nullptr;
    }

    bool SimulateArrow(
        RE::PlayerCharacter* player,
        RE::NiCamera* camera,
        std::vector<RE::NiPoint3>& path,
        PhysicsVector& impact)
    {
        const auto ammo = player->GetCurrentAmmo();
        const auto projectile = ammo ? ammo->data.projectile : nullptr;
        if (!projectile || projectile->data.speed <= 0.0f) {
            return false;
        }

        auto direction = ToVector(camera->world.rotate.GetVectorY());
        if (Dot(direction, direction) <= 0.0001f) {
            return false;
        }

        float tiltRadians = 0.0f;
        if (const auto* setting = RE::INISettingCollection::GetSingleton()->GetSetting("f3PArrowTiltUpAngle:Combat")) {
            tiltRadians = setting->GetFloat() * 0.017453292519943295f;
        }
        RE::NiMatrix3 tilt;
        tilt.MakeXRotation(tiltRadians);
        direction = ToVector(camera->world.rotate * (tilt * RE::NiPoint3(0.0f, 1.0f, 0.0f)));
        direction = Normalize(direction);

        auto start = camera->world.translate;
        if (const auto* arrowNode = FindArrowNode(player)) {
            start = arrowNode->world.translate;
        }

        const auto drawStrength = GetBowDrawStrength(player);
        float weaponSpeed = 1.0f;
        if (const auto weapon = skyrim_cast<const RE::TESObjectWEAP*>(player->GetEquippedObject(false))) {
            weaponSpeed = weapon->weaponData.speed;
        }
        const auto speedUpperLimit = *REL::Relocation<float*>{ REL::RelocationID(505064, 374223) };
        const auto speedLowerLimit = *REL::Relocation<float*>{ REL::RelocationID(505066, 374226) };
        const auto speedMultiplier = *REL::Relocation<float*>{ REL::RelocationID(515530, 401668) };
        if (speedUpperLimit <= weaponSpeed) {
            weaponSpeed = speedUpperLimit;
        }
        if (weaponSpeed <= speedLowerLimit) {
            weaponSpeed = speedLowerLimit;
        }
        const auto weaponSpeedMultiplier = (speedLowerLimit - weaponSpeed) * speedMultiplier + 1.0f;
        auto velocity = direction * (projectile->data.speed * drawStrength * weaponSpeedMultiplier);

        auto worldGravity = PhysicsVector{ 0.0f, 0.0f, -9.8f };
        if (const auto cell = player->GetParentCell(); cell && cell->GetbhkWorld() && cell->GetbhkWorld()->GetWorld1()) {
            const auto& gravity = cell->GetbhkWorld()->GetWorld1()->gravity.quad;
            worldGravity = { gravity.m128_f32[0], gravity.m128_f32[1], gravity.m128_f32[2] };
        }
        const auto gravity = worldGravity * (projectile->data.gravity * kGravityScale);
        auto position = ToVector(start);
        auto previous = position;
        path.clear();
        path.reserve(kMaxSegments + 1);
        path.push_back(start);

        for (std::size_t i = 0; i < kMaxSegments; ++i) {
            velocity += (gravity - velocity * kLinearDamping) * kStep;
            position += velocity * kStep;

            if (const auto hit = CastRay(player, previous, position)) {
                impact = *hit;
                path.push_back(ToNi(*hit));
                return true;
            }

            path.push_back(ToNi(position));
            previous = position;
            if (Length(position - ToVector(start)) >= kMaxRange) {
                break;
            }
        }
        return false;
    }

    RE::IMenu* GetHUDMenu()
    {
        const auto ui = RE::UI::GetSingleton();
        const auto strings = RE::InterfaceStrings::GetSingleton();
        return ui && strings ? ui->GetMenu(strings->hudMenu).get() : nullptr;
    }

    bool EnsureHUDState(RE::IMenu* menu)
    {
        if (!menu || !menu->uiMovie) {
            return false;
        }
        if (g_hudMovie == menu->uiMovie.get() && g_crosshairBaseCaptured) {
            return true;
        }

        g_trajectoryClip = RE::GFxValue();
        g_hudMovie = menu->uiMovie.get();
        g_crosshairBaseCaptured = false;

        RE::GFxValue x;
        RE::GFxValue y;
        if (!g_hudMovie->GetVariable(&x, "_root.HUDMovieBaseInstance.CrosshairInstance._x") ||
            !g_hudMovie->GetVariable(&y, "_root.HUDMovieBaseInstance.CrosshairInstance._y") ||
            !x.IsNumber() || !y.IsNumber())
        {
            return false;
        }

        g_crosshairBaseX = x.GetNumber();
        g_crosshairBaseY = y.GetNumber();
        g_crosshairBaseCaptured = true;

        RE::GFxValue hudBase;
        if (!g_hudMovie->GetVariable(&hudBase, "_root.HUDMovieBaseInstance") ||
            !hudBase.IsDisplayObject() ||
            !hudBase.CreateEmptyMovieClip(&g_trajectoryClip, "ArrowTrajectorySKSE", 1000))
        {
            logger::warn("Unable to create the HUD trajectory drawing layer.");
            return false;
        }
        return true;
    }

    bool SetCrosshairScreenPosition(RE::IMenu* menu, float x, float y)
    {
        if (!menu || !menu->uiMovie || !g_crosshairBaseCaptured) {
            return false;
        }

        const auto rect = menu->uiMovie->GetVisibleFrameRect();
        RE::GFxValue crosshair;
        if (!menu->uiMovie->GetVariable(&crosshair, "_root.HUDMovieBaseInstance.Crosshair") ||
            !crosshair.IsDisplayObject())
        {
            return false;
        }

        RE::GFxValue::DisplayInfo position;
        position.SetPosition(
            static_cast<double>(x) - (rect.left + rect.right) * 0.5 + g_crosshairBaseX,
            static_cast<double>(y) - (rect.top + rect.bottom) * 0.5 + g_crosshairBaseY);
        return crosshair.SetDisplayInfo(position);
    }

    bool SetCrosshairPosition(RE::IMenu* menu, const RE::NiPoint3& worldPosition)
    {
        const auto camera = RE::Main::WorldRootCamera();
        if (!camera) {
            return false;
        }

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        return camera->WorldPtToScreenPt3(worldPosition, x, y, z, 1.0e-5f) &&
            SetCrosshairScreenPosition(menu, x, y);
    }

    void DrawTrajectory(RE::IMenu* menu, RE::NiCamera* camera, const std::vector<RE::NiPoint3>& path)
    {
        if (!menu || !menu->uiMovie || !g_trajectoryClip.IsDisplayObject()) {
            return;
        }

        g_trajectoryClip.Invoke("clear");
        if (path.size() < 2) {
            RE::GFxValue::DisplayInfo hidden;
            hidden.SetVisible(false);
            g_trajectoryClip.SetDisplayInfo(hidden);
            return;
        }

        RE::GFxValue::DisplayInfo visible;
        visible.SetVisible(true);
        g_trajectoryClip.SetDisplayInfo(visible);

        const std::array<RE::GFxValue, 3> lineStyle{
            RE::GFxValue(2.0),
            RE::GFxValue(0xFFD36A),
            RE::GFxValue(85.0)
        };
        if (!g_trajectoryClip.Invoke("lineStyle", lineStyle)) {
            if (!g_reportedDrawingFailure) {
                logger::warn("The HUD movie does not support drawing the trajectory overlay.");
                g_reportedDrawingFailure = true;
            }
            return;
        }

        const auto rect = menu->uiMovie->GetVisibleFrameRect();
        const auto centerX = (rect.left + rect.right) * 0.5;
        const auto centerY = (rect.top + rect.bottom) * 0.5;
        bool drawing = false;
        for (const auto& point : path) {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            if (!camera->WorldPtToScreenPt3(point, x, y, z, 1.0e-5f)) {
                drawing = false;
                continue;
            }

            const std::array<RE::GFxValue, 2> args{
                RE::GFxValue(static_cast<double>(x) - centerX),
                RE::GFxValue(static_cast<double>(y) - centerY)
            };
            if (!g_trajectoryClip.Invoke(drawing ? "lineTo" : "moveTo", args)) {
                if (!g_reportedDrawingFailure) {
                    logger::warn("Unable to draw the trajectory path in the HUD movie.");
                    g_reportedDrawingFailure = true;
                }
                return;
            }
            drawing = true;
        }
    }

    void ResetHUD(RE::IMenu* menu)
    {
        if (!menu || !menu->uiMovie) {
            return;
        }
        const auto rect = menu->uiMovie->GetVisibleFrameRect();
        SetCrosshairScreenPosition(
            menu,
            (rect.left + rect.right) * 0.5f,
            (rect.top + rect.bottom) * 0.5f);

        if (g_trajectoryClip.IsDisplayObject()) {
            g_trajectoryClip.Invoke("clear");
            RE::GFxValue::DisplayInfo hidden;
            hidden.SetVisible(false);
            g_trajectoryClip.SetDisplayInfo(hidden);
        }
    }

    void CameraUpdateHook(RE::PlayerCamera* camera)
    {
        g_originalCameraUpdate(camera);
        Trajectory::Update(camera);
    }
}

bool Trajectory::Install()
{
    REL::Relocation<std::uintptr_t> vtable{ RE::VTABLE_PlayerCamera[0] };
    g_originalCameraUpdate = reinterpret_cast<CameraUpdate>(vtable.write_vfunc(2, CameraUpdateHook));
    if (!g_originalCameraUpdate) {
        logger::critical("Failed to hook PlayerCamera::Update; ArrowTrajectory will not load.");
        return false;
    }
    return true;
}

void Trajectory::Update(RE::PlayerCamera* camera)
{
    if (!camera || !RE::PlayerCharacter::GetSingleton()) {
        return;
    }

    const auto player = RE::PlayerCharacter::GetSingleton();
    auto* menu = GetHUDMenu();
    if (!EnsureHUDState(menu)) {
        return;
    }

    const bool usingBow = IsBowOrCrossbow(player);
    const bool usingMagic = !usingBow && IsCombatMagicDrawn(player);
    if (!usingBow && !usingMagic) {
        ResetHUD(menu);
        return;
    }

    const auto worldCamera = RE::Main::WorldRootCamera();
    if (!worldCamera) {
        ResetHUD(menu);
        return;
    }

    std::vector<RE::NiPoint3> path;
    PhysicsVector hitPosition{};
    bool hit = false;
    if (usingBow) {
        if (!player->GetCurrentAmmo()) {
            ResetHUD(menu);
            return;
        }
        hit = SimulateArrow(player, worldCamera, path, hitPosition);
        DrawTrajectory(menu, worldCamera, path);
    } else {
        const auto direction = ToVector(worldCamera->world.rotate.GetVectorY());
        const auto origin = ToVector(worldCamera->world.translate);
        if (Dot(direction, direction) > 0.0001f) {
            if (const auto result = CastRay(player, origin, origin + Normalize(direction) * kMaxRange)) {
                hitPosition = *result;
                hit = true;
            }
        }
        DrawTrajectory(menu, worldCamera, {});
    }

    if (hit) {
        if (!SetCrosshairPosition(menu, ToNi(hitPosition))) {
            ResetHUD(menu);
        }
    } else {
        ResetHUD(menu);
    }
}
