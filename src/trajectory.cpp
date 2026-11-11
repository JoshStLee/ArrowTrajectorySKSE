#include "pch.h"
#include "trajectory.h"
#include "trajectory_renderer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <optional>
#include <unordered_map>
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

    struct CameraRotation
    {
        float pitch;
        float yaw;
    };

    using CameraStateUpdate = void (*)(RE::TESCameraState*, RE::BSTSmartPointer<RE::TESCameraState>&);

    std::unordered_map<std::uintptr_t, CameraStateUpdate> g_originalCameraStateUpdates;
    bool g_attackInputHeld = false;
    bool g_attackInputListenerRegistered = false;
    RE::GFxMovieView* g_hudMovie = nullptr;
    double g_crosshairBaseX = 0.0;
    double g_crosshairBaseY = 0.0;
    bool g_crosshairBaseCaptured = false;
    bool g_reportedFirstPath = false;
    bool g_reportedFirstPrediction = false;
    bool g_reportedPredictionFailure = false;
    bool g_reportedPredictionMiss = false;
    bool g_reportedAimParameters = false;
    bool g_reportedFirstUpdate = false;
    bool g_reportedCombatState = false;
    bool g_reportedNoAmmo = false;
    std::uint64_t g_lastBowStateDiagnostic = 0;
    bool g_hasBowStateDiagnostic = false;
    bool g_lastAttackHeldDiagnostic = false;
    std::chrono::steady_clock::time_point g_lastStateHeartbeat{};

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

    bool HasBowOrCrossbow(const RE::Actor* actor);
    bool IsBowOrCrossbow(const RE::Actor* actor);

    class AttackInputSink final : public RE::BSTEventSink<RE::InputEvent*>
    {
    public:
        RE::BSEventNotifyControl ProcessEvent(
            RE::InputEvent* const* event,
            RE::BSTEventSource<RE::InputEvent*>*) override
        {
            if (!event || !*event) {
                return RE::BSEventNotifyControl::kContinue;
            }

            const auto userEvents = RE::UserEvents::GetSingleton();
            if (!userEvents) {
                return RE::BSEventNotifyControl::kContinue;
            }

            for (auto* current = *event; current; current = current->next) {
                if (current->GetEventType() != RE::INPUT_EVENT_TYPE::kButton) {
                    continue;
                }
                const auto button = current->AsButtonEvent();
                if (!button ||
                    std::strcmp(button->QUserEvent().c_str(), userEvents->rightAttack.c_str()) != 0)
                {
                    continue;
                }

                const bool held = button->IsPressed();
                if (held) {
                    const auto player = RE::PlayerCharacter::GetSingleton();
                    const auto ammo = player ? player->GetCurrentAmmo() : nullptr;
                    if (!player || !HasBowOrCrossbow(player) || !ammo || !ammo->data.projectile) {
                        continue;
                    }
                }
                if (held != g_attackInputHeld) {
                    g_attackInputHeld = held;
                    logger::info("Mapped bow attack input is {}.", held ? "held" : "released");
                }
            }
            return RE::BSEventNotifyControl::kContinue;
        }
    };

    AttackInputSink g_attackInputSink;

    PhysicsVector ToVector(const RE::NiPoint3& point)
    {
        return { point.x, point.y, point.z };
    }

    CameraRotation GetSmoothCamCameraRotation(const RE::NiMatrix3& matrix)
    {
        constexpr float halfPi = 1.57079632679489661923f;
        const auto a = std::clamp(-matrix.entry[2][0], -1.0f, 1.0f);
        const auto pitch = std::clamp(std::asin(a), -halfPi, halfPi) - halfPi;
        const auto yaw = matrix.entry[0][0] <= 0.001f || matrix.entry[2][2] <= 0.001f
            ? std::atan2(matrix.entry[0][2], matrix.entry[1][2]) - halfPi
            : std::atan2(matrix.entry[0][0], matrix.entry[1][0]);
        return { pitch, yaw };
    }

    CameraRotation GetSmoothCamCameraRotation(const RE::NiQuaternion& quaternion)
    {
        constexpr float halfPi = 1.57079632679489661923f;
        return {
            -std::atan2(
                2.0f * (quaternion.y * quaternion.z + quaternion.w * quaternion.x),
                quaternion.w * quaternion.w - quaternion.x * quaternion.x -
                    quaternion.y * quaternion.y + quaternion.z * quaternion.z) - halfPi,
            -std::atan2(
                2.0f * (quaternion.x * quaternion.y + quaternion.w * quaternion.z),
                quaternion.w * quaternion.w + quaternion.x * quaternion.x -
                    quaternion.y * quaternion.y - quaternion.z * quaternion.z)
        };
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

    bool HasBowOrCrossbow(const RE::Actor* actor)
    {
        if (!actor) {
            return false;
        }
        const auto rightWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(false));
        const auto leftWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(true));
        const auto isRanged = [](const RE::TESObjectWEAP* weapon) {
            return weapon &&
                (weapon->weaponData.animationType == RE::WEAPON_TYPE::kBow ||
                 weapon->weaponData.animationType == RE::WEAPON_TYPE::kCrossbow);
        };
        return isRanged(rightWeapon) || isRanged(leftWeapon);
    }

    bool IsBowOrCrossbow(const RE::Actor* actor)
    {
        if (!HasBowOrCrossbow(actor)) {
            return false;
        }
        const auto rightWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(false));
        const auto leftWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(true));
        const auto hasWeaponType = [](const RE::TESObjectWEAP* weapon, RE::WEAPON_TYPE type) {
            return weapon && weapon->weaponData.animationType == type;
        };
        const bool isCrossbow =
            hasWeaponType(rightWeapon, RE::WEAPON_TYPE::kCrossbow) ||
            hasWeaponType(leftWeapon, RE::WEAPON_TYPE::kCrossbow);
        const bool hasBow =
            isCrossbow ||
            hasWeaponType(rightWeapon, RE::WEAPON_TYPE::kBow) ||
            hasWeaponType(leftWeapon, RE::WEAPON_TYPE::kBow);
        if (!hasBow) {
            return false;
        }

        std::uint32_t movementBits = 0;
        std::memcpy(&movementBits, std::addressof(actor->actorState1), sizeof(movementBits));
        const bool bowDrawn = (movementBits & (1u << 31)) != 0;
        const bool crossbowReady = (movementBits & (1u << 29)) != 0;
        const bool aiming = g_attackInputHeld || (bowDrawn && (!isCrossbow || crossbowReady));
        const auto weaponState = static_cast<std::uint8_t>(actor->GetWeaponState());
        const auto attackState = static_cast<std::uint8_t>(actor->GetAttackState());
        const auto rightType = rightWeapon ? rightWeapon->weaponData.animationType.underlying() : 0xFF;
        const auto leftType = leftWeapon ? leftWeapon->weaponData.animationType.underlying() : 0xFF;
        const std::uint64_t diagnosticState =
            static_cast<std::uint64_t>(movementBits) |
            (static_cast<std::uint64_t>(weaponState) << 32) |
            (static_cast<std::uint64_t>(attackState) << 40) |
            (static_cast<std::uint64_t>(rightType) << 48) |
            (static_cast<std::uint64_t>(leftType) << 56);
        const auto now = std::chrono::steady_clock::now();
        const bool stateChanged =
            !g_hasBowStateDiagnostic ||
            diagnosticState != g_lastBowStateDiagnostic ||
            g_attackInputHeld != g_lastAttackHeldDiagnostic;
        const bool heartbeatElapsed =
            g_lastStateHeartbeat.time_since_epoch().count() == 0 ||
            now - g_lastStateHeartbeat >= std::chrono::seconds(5);
        if (stateChanged || heartbeatElapsed) {
            logger::info(
                "Ranged weapon state: rightType={}, leftType={}, actionBits=0x{:08X}, attack={}, weapon={}, drawn={}, bowBit={}, crossbowBit={}, mappedAttackHeld={}, aiming={}, reason={}.",
                rightWeapon ? static_cast<int>(rightType) : -1,
                leftWeapon ? static_cast<int>(leftType) : -1,
                movementBits,
                static_cast<int>(attackState),
                static_cast<int>(weaponState),
                actor->IsWeaponDrawn(),
                bowDrawn,
                crossbowReady,
                g_attackInputHeld,
                aiming,
                stateChanged ? "changed" : "5s heartbeat");
            g_lastBowStateDiagnostic = diagnosticState;
            g_lastAttackHeldDiagnostic = g_attackInputHeld;
            g_hasBowStateDiagnostic = true;
            g_lastStateHeartbeat = now;
        }

        return aiming;
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
        const RE::BSFixedString leftMagazineName("NPC L MagicNode [LMag]");

        const auto rightWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(false));
        const auto leftWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(true));
        const auto isCrossbow = [](const RE::TESObjectWEAP* weapon) {
            return weapon && weapon->weaponData.animationType == RE::WEAPON_TYPE::kCrossbow;
        };
        if (isCrossbow(rightWeapon) || isCrossbow(leftWeapon)) {
            if (auto* magazine = root->GetObjectByName(rightMagazineName)) {
                if (auto* arrow = magazine->GetObjectByName(arrowName)) {
                    return arrow;
                }
                return magazine;
            }
        }

        if (auto* weapon = root->GetObjectByName(weaponName)) {
            if (weapon->GetObjectByName(arrowName)) {
                return weapon;
            }
        }
        if (auto* magazine = root->GetObjectByName(rightMagazineName)) {
            if (auto* arrow = magazine->GetObjectByName(arrowName)) {
                return arrow;
            }
        }
        if (auto* magazine = root->GetObjectByName(leftMagazineName)) {
            if (auto* arrow = magazine->GetObjectByName(arrowName)) {
                return arrow;
            }
        }
        return nullptr;
    }

    bool SimulateArrow(
        RE::PlayerCharacter* player,
        RE::PlayerCamera* playerCamera,
        RE::NiCamera* camera,
        std::vector<RE::NiPoint3>& path,
        PhysicsVector& impact,
        const char*& failureReason)
    {
        failureReason = nullptr;
        const auto ammo = player->GetCurrentAmmo();
        const auto projectile = ammo ? ammo->data.projectile : nullptr;
        if (!projectile || projectile->data.speed <= 0.0f) {
            failureReason = "current ammo has no projectile or a non-positive projectile speed";
            return false;
        }

        const auto thirdPersonState = playerCamera
            ? skyrim_cast<const RE::ThirdPersonState*>(playerCamera->currentState.get())
            : nullptr;
        const auto cameraRotation = GetSmoothCamCameraRotation(camera->world.rotate);
        auto aimRotation = cameraRotation;
        aimRotation.pitch = -cameraRotation.pitch - 1.57079632679489661923f;
        if (thirdPersonState && thirdPersonState->freeRotationEnabled) {
            aimRotation.yaw = player->GetAngleZ();
        }

        float tiltRadians = 0.0f;
        if (const auto* setting = RE::INISettingCollection::GetSingleton()->GetSetting("f3PArrowTiltUpAngle:Combat")) {
            tiltRadians = setting->GetFloat() * 0.017453292519943295f;
        }
        auto projectilePitch = aimRotation.pitch - tiltRadians;
        projectilePitch *= -1.0f;
        const auto pitchCos = std::cos(projectilePitch);
        const auto direction = PhysicsVector{
            pitchCos * std::sin(aimRotation.yaw),
            pitchCos * std::cos(aimRotation.yaw),
            std::sin(projectilePitch)
        };
        if (!g_reportedAimParameters) {
            logger::info(
                "SmoothCam-matched bow aim: renderPitch={:.4f}, renderYaw={:.4f}, aimPitch={:.4f}, aimYaw={:.4f}, projectilePitch={:.4f}, direction=({:.4f}, {:.4f}, {:.4f}).",
                cameraRotation.pitch,
                cameraRotation.yaw,
                aimRotation.pitch,
                aimRotation.yaw,
                projectilePitch,
                direction.x,
                direction.y,
                direction.z);
            g_reportedAimParameters = true;
        }

        const auto arrowNode = FindArrowNode(player);
        if (!arrowNode) {
            failureReason = "could not find the bow arrow or hand/magazine node in the player's 3D";
            return false;
        }
        const auto player3D = player->Get3D();
        if (!player3D) {
            failureReason = "player 3D is unavailable";
            return false;
        }
        const auto playerPosition = player3D->world.translate;
        const auto arrowOffset = arrowNode->world.translate - playerPosition;
        const auto yawDelta = player->GetAngleZ() - cameraRotation.yaw;
        const auto cosine = std::cos(yawDelta);
        const auto sine = std::sin(yawDelta);
        const RE::NiPoint3 start{
            playerPosition.x + cosine * arrowOffset.x - sine * arrowOffset.y,
            playerPosition.y + sine * arrowOffset.x + cosine * arrowOffset.y,
            playerPosition.z + arrowOffset.z
        };

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
        failureReason = "predicted path did not hit a collision surface";
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

    void DrawTrajectory(RE::NiCamera* camera, const std::vector<RE::NiPoint3>& path)
    {
        TrajectoryRenderer::Submit(path, camera);
        if (!g_reportedFirstPath && path.size() > 1) {
            logger::info("Trajectory world path passed to the Direct3D renderer: {} points.", path.size());
            g_reportedFirstPath = true;
        }
    }

    void ResetHUD(RE::IMenu* menu)
    {
        TrajectoryRenderer::Clear();
        if (!menu || !menu->uiMovie) {
            return;
        }
        const auto rect = menu->uiMovie->GetVisibleFrameRect();
        SetCrosshairScreenPosition(
            menu,
            (rect.left + rect.right) * 0.5f,
            (rect.top + rect.bottom) * 0.5f);

    }

    void CenterCrosshair(RE::IMenu* menu)
    {
        if (!menu || !menu->uiMovie) {
            return;
        }
        const auto rect = menu->uiMovie->GetVisibleFrameRect();
        SetCrosshairScreenPosition(
            menu,
            (rect.left + rect.right) * 0.5f,
            (rect.top + rect.bottom) * 0.5f);
    }

    void CameraStateUpdateHook(
        RE::TESCameraState* state,
        RE::BSTSmartPointer<RE::TESCameraState>& nextState)
    {
        const auto vtableAddress = *reinterpret_cast<std::uintptr_t*>(state);
        const auto original = g_originalCameraStateUpdates.find(vtableAddress);
        if (original == g_originalCameraStateUpdates.end()) {
            logger::error("Camera-state update hook received an unregistered vtable: {:X}.", vtableAddress);
            return;
        }

        original->second(state, nextState);
        TrajectoryRenderer::Install();
        if (!g_reportedFirstUpdate) {
            logger::info("Camera-state update hook is running.");
            g_reportedFirstUpdate = true;
        }
        Trajectory::Update(RE::PlayerCamera::GetSingleton());
    }
}

bool Trajectory::Install()
{
    TrajectoryRenderer::Install();

    if (!g_attackInputListenerRegistered) {
        const auto inputManager = RE::BSInputDeviceManager::GetSingleton();
        if (!inputManager) {
            logger::critical("Failed to register attack input listener: input device manager is unavailable.");
            return false;
        }
        inputManager->AddEventSink(&g_attackInputSink);
        g_attackInputListenerRegistered = true;
        logger::info("Registered the mapped right-attack input observer.");
    }

    const auto camera = RE::PlayerCamera::GetSingleton();
    if (!camera) {
        logger::critical("Failed to hook camera-state updates: PlayerCamera singleton is unavailable.");
        return false;
    }

    std::size_t hookedVTables = 0;
    for (const auto& state : camera->GetRuntimeData().cameraStates) {
        auto* cameraState = state.get();
        if (!cameraState) {
            continue;
        }

        const auto vtableAddress = *reinterpret_cast<std::uintptr_t*>(cameraState);
        if (g_originalCameraStateUpdates.contains(vtableAddress)) {
            continue;
        }

        REL::Relocation<std::uintptr_t> vtable{ vtableAddress };
        const auto original = vtable.write_vfunc(3, CameraStateUpdateHook);
        if (!original) {
            logger::critical("Failed to hook camera-state update vtable {:X}.", vtableAddress);
            return false;
        }
        g_originalCameraStateUpdates.emplace(vtableAddress, reinterpret_cast<CameraStateUpdate>(original));
        ++hookedVTables;
    }

    if (hookedVTables == 0) {
        logger::critical("Failed to hook camera-state updates: no initialized camera states were available.");
        return false;
    }

    logger::info("Camera-state update hooks installed on {} vtables.", hookedVTables);
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
        TrajectoryRenderer::Clear();
        static bool reportedHUDUnavailable = false;
        if (!reportedHUDUnavailable) {
            logger::warn("Trajectory update skipped: HUD movie or drawing layer is unavailable.");
            reportedHUDUnavailable = true;
        }
        return;
    }

    const bool usingBow = IsBowOrCrossbow(player);
    const bool usingMagic = !usingBow && IsCombatMagicDrawn(player);
    if ((usingBow || usingMagic) && !g_reportedCombatState) {
        logger::info("Raycast crosshair active: bow={}, combat magic={}.", usingBow, usingMagic);
        g_reportedCombatState = true;
    }
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
            if (!g_reportedNoAmmo) {
                logger::warn("Trajectory prediction skipped: no current bow ammo/projectile.");
                g_reportedNoAmmo = true;
            }
            ResetHUD(menu);
            return;
        }
        const char* failureReason = nullptr;
        hit = SimulateArrow(player, camera, worldCamera, path, hitPosition, failureReason);
        if (path.size() > 1) {
            DrawTrajectory(worldCamera, path);
        } else {
            TrajectoryRenderer::Clear();
        }
        if (!g_reportedFirstPrediction) {
            logger::info(
                "First bow prediction completed: hit={}, worldPathPoints={}.",
                hit,
                path.size());
            g_reportedFirstPrediction = true;
        }
        if (!hit && failureReason && path.empty() && !g_reportedPredictionFailure) {
            logger::warn("Bow trajectory was not simulated: {}.", failureReason);
            g_reportedPredictionFailure = true;
        } else if (!hit && failureReason && !path.empty() && !g_reportedPredictionMiss) {
            logger::info("Bow trajectory was simulated but no collision was found; drawing the predicted path.");
            g_reportedPredictionMiss = true;
        }
    } else {
        const auto direction = ToVector(worldCamera->world.rotate.GetVectorY());
        const auto origin = ToVector(worldCamera->world.translate);
        if (Dot(direction, direction) > 0.0001f) {
            if (const auto result = CastRay(player, origin, origin + Normalize(direction) * kMaxRange)) {
                hitPosition = *result;
                hit = true;
            }
        }
        DrawTrajectory(worldCamera, {});
    }

    if (hit) {
        if (!SetCrosshairPosition(menu, ToNi(hitPosition))) {
            CenterCrosshair(menu);
        }
    } else {
        CenterCrosshair(menu);
    }
}
