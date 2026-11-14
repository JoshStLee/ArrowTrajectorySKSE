#include "pch.h"
#include "trajectory_prediction.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace TrajectoryPrediction
{
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

    using ProjectileFloatFunction = float (*)(RE::Projectile*);

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

    AimRotation GetAimRotationInternal(const RE::NiQuaternion& rotation)
    {
        return {
            -std::atan2(
                2.0f * (rotation.y * rotation.z + rotation.w * rotation.x),
                rotation.w * rotation.w - rotation.x * rotation.x -
                    rotation.y * rotation.y + rotation.z * rotation.z),
            -std::atan2(
                2.0f * (rotation.x * rotation.y + rotation.w * rotation.z),
                rotation.w * rotation.w + rotation.x * rotation.x -
                    rotation.y * rotation.y - rotation.z * rotation.z)
        };
    }

    void CorrectPlayerArrowVelocityInternal(RE::ArrowProjectile* arrow)
    {
        const auto player = RE::PlayerCharacter::GetSingleton();
        if (!arrow || !player) {
            return;
        }
        auto& projectileData = arrow->GetProjectileRuntimeData();
        if (!projectileData.shooter || projectileData.shooter.get().get() != player ||
            !projectileData.weaponSource || !projectileData.ammoSource) {
            return;
        }

        const auto playerCamera = RE::PlayerCamera::GetSingleton();
        const auto thirdPersonState = playerCamera
            ? skyrim_cast<const RE::ThirdPersonState*>(playerCamera->currentState.get())
            : nullptr;
        if (!thirdPersonState) {
            return;
        }

        auto aim = GetAimRotationInternal(thirdPersonState->rotation);
        if (thirdPersonState->freeRotationEnabled) {
            aim.yaw = player->GetAngleZ();
        }

        if ((projectileData.flags.underlying() & (1u << 31)) != 0) {
            aim.pitch = arrow->GetAngleX();
            aim.yaw = arrow->GetAngleZ();
        } else {
            float tilt = 2.5f;
            if (const auto* setting = RE::INISettingCollection::GetSingleton()->GetSetting("f3PArrowTiltUpAngle:Combat")) {
                tilt = setting->GetFloat();
            }
            aim.pitch -= tilt * 0.017453292519943295f;
        }
        aim.pitch *= -1.0f;

        static REL::Relocation<ProjectileFloatFunction> getWeaponSpeedMultiplier{
            REL::VariantID(42537, 43700, 0)
        };
        static REL::Relocation<ProjectileFloatFunction> getDrawPower{
            REL::VariantID(42536, 43699, 0)
        };
        const auto speed = projectileData.ammoSource->data.projectile
            ? projectileData.ammoSource->data.projectile->data.speed
            : 0.0f;
        if (speed <= 0.0f) {
            return;
        }

        const auto velocityScale =
            getWeaponSpeedMultiplier(arrow) * getDrawPower(arrow) * speed * projectileData.speedMult;
        const auto pitchCos = std::cos(aim.pitch);
        projectileData.linearVelocity = {
            pitchCos * std::sin(aim.yaw) * velocityScale,
            pitchCos * std::cos(aim.yaw) * velocityScale,
            std::sin(aim.pitch) * velocityScale
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
            if (hitInfo.hitFraction >= earlyOutHitFraction) {
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
            // The CFilter member stores the packed layer, biped-part, and
            // system-group bits. SmoothCam's mask selects collision-layer
            // indices, so extract the layer before indexing the mask.
            const auto filter = std::to_underlying(
                collidable->broadPhaseHandle.collisionFilterInfo.GetCollisionLayer());
            // Match SmoothCam's RayCollector: only collision layers selected
            // by its Havok filter mask participate in aim and trajectory hits.
            constexpr std::uint64_t kSmoothCamCollisionMask = 0x40122716;
            if (filter >= 64 || (kSmoothCamCollisionMask & (1ULL << filter)) == 0) {
                return;
            }
            using GetAVObject = RE::NiAVObject* (*)(const RE::hkpCdBody*);
            static REL::Relocation<GetAVObject> getAVObject{ REL::VariantID(76160, 77988, 0) };
            const auto avObject = getAVObject(bodyRoot);
            // SmoothCam ignores the player's scene root and treats unresolved
            // Havok bodies as non-hits, matching its crosshair ray behavior.
            if (!avObject || avObject == ignoredObject) {
                return;
            }

            earlyOutHitFraction = hitInfo.hitFraction;
            hits.emplace_back(hitInfo.hitFraction, avObject);
        }

        std::vector<std::pair<float, RE::NiAVObject*>> hits;
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

    std::optional<PhysicsVector> CastRayVector(
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
        // SmoothCam's hkpCastRay uses this Havok world-to-physics scale.
        constexpr float scale = 0.0142875f;
        RE::bhkPickData pickData{};
        pickData.rayInput.from = RE::hkVector4(start.x * scale, start.y * scale, start.z * scale, 1.0f);
        pickData.rayInput.to = RE::hkVector4(0.0f, 0.0f, 0.0f, 0.0f);
        pickData.ray = RE::hkVector4(delta.x * scale, delta.y * scale, delta.z * scale, 1.0f);

        const auto player3D = player->Get3D();
        FirstHitCollector collector(player3D);
        collector.Reset();
        pickData.closestRayHitCollector = &collector;
        world->PickObject(pickData);

        if (collector.hits.empty()) {
            return std::nullopt;
        }
        const auto closest = std::min_element(
            collector.hits.begin(), collector.hits.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
        return start + delta * closest->first;
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

        // Match Crosshair::Manager::FindArrowNode in SmoothCam. In particular,
        // a drawn bow's arrow origin is the WEAPON hand node, while crossbows
        // and fallback rigs use their magazine nodes.
        static const RE::BSFixedString weaponName("WEAPON");
        static const RE::BSFixedString arrowName("Arrow:0");
        static const RE::BSFixedString rightMagazineName("NPC R MagicNode [RMag]");
        static const RE::BSFixedString leftMagazineName("NPC L MagicNode [LMag]");

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

        auto* handNode = skyrim_cast<RE::NiNode*>(root->GetObjectByName(weaponName));
        if (handNode && handNode->children.size() > 0 && handNode->GetObjectByName(arrowName)) {
            return handNode;
        }

        handNode = skyrim_cast<RE::NiNode*>(root->GetObjectByName(rightMagazineName));
        if (handNode && handNode->children.size() > 0) {
            if (auto* arrow = handNode->GetObjectByName(arrowName)) {
                return arrow;
            }
        }

        handNode = skyrim_cast<RE::NiNode*>(root->GetObjectByName(leftMagazineName));
        if (handNode && handNode->children.size() > 0) {
            if (auto* arrow = handNode->GetObjectByName(arrowName)) {
                return arrow;
            }
        }
        return nullptr;
    }

    bool SimulateArrowInternal(
        RE::PlayerCharacter* player,
        RE::PlayerCamera* playerCamera,
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
        if (!thirdPersonState) {
            failureReason = "third-person camera state is unavailable";
            return false;
        }
        auto aim = GetAimRotationInternal(thirdPersonState->rotation);
        if (thirdPersonState->freeRotationEnabled) {
            aim.yaw = player->GetAngleZ();
        }
        float tiltRadians = 0.0f;
        if (const auto* setting = RE::INISettingCollection::GetSingleton()->GetSetting("f3PArrowTiltUpAngle:Combat")) {
            tiltRadians = setting->GetFloat() * 0.017453292519943295f;
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
        // Match SmoothCam's Crosshair::Manager::TranslateFirePostion: rotate
        // the arrow's actor-local offset by the difference between actor yaw
        // and camera yaw before using it as the launch origin.
        const auto playerPosition = player3D->world.translate;
        const auto arrowOffset = arrowNode->world.translate - playerPosition;
        const auto yawDelta = player->GetAngleZ() - aim.yaw;
        const auto yawCos = std::cos(yawDelta);
        const auto yawSin = std::sin(yawDelta);
        const RE::NiPoint3 start{
            playerPosition.x + yawCos * arrowOffset.x - yawSin * arrowOffset.y,
            playerPosition.y + yawSin * arrowOffset.x + yawCos * arrowOffset.y,
            playerPosition.z + arrowOffset.z
        };
        // Match SmoothCam's ArrowFixes::mUpdateArrowFlightPath: subtract the
        // game's bow tilt from its aim Euler pitch, then negate pitch when
        // converting the launch rotation to a world-space velocity.
        const auto projectilePitch = -(aim.pitch - tiltRadians);
        const auto pitchCos = std::cos(projectilePitch);
        const PhysicsVector direction{
            pitchCos * std::sin(aim.yaw),
            pitchCos * std::cos(aim.yaw),
            std::sin(projectilePitch)
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

        // SmoothCam stores 128 points including the launch point, so it makes
        // at most 127 simulation steps.
        for (std::size_t i = 1; i < kMaxSegments; ++i) {
            velocity += (gravity - velocity * kLinearDamping) * kStep;
            position += velocity * kStep;

            if (const auto hit = CastRayVector(player, previous, position)) {
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

}

AimRotation GetAimRotation(const RE::NiQuaternion& rotation)
{
    return GetAimRotationInternal(rotation);
}

std::optional<RE::NiPoint3> CastRay(
    RE::PlayerCharacter* player,
    const RE::NiPoint3& start,
    const RE::NiPoint3& end)
{
    const auto toVector = [](const RE::NiPoint3& point) {
        return PhysicsVector{ point.x, point.y, point.z };
    };
    const auto hit = CastRayVector(player, toVector(start), toVector(end));
    if (!hit) {
        return std::nullopt;
    }
    return ToNi(*hit);
}

bool SimulateArrow(
    RE::PlayerCharacter* player,
    RE::PlayerCamera* playerCamera,
    std::vector<RE::NiPoint3>& path,
    RE::NiPoint3& impact,
    const char*& failureReason)
{
    PhysicsVector internalImpact{};
    const auto hit = SimulateArrowInternal(player, playerCamera, path, internalImpact, failureReason);
    impact = ToNi(internalImpact);
    return hit;
}

void CorrectPlayerArrowVelocity(RE::ArrowProjectile* arrow)
{
    CorrectPlayerArrowVelocityInternal(arrow);
}
}
