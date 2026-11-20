#include "pch.h"
#include "trajectory.h"
#include "trajectory_hud.h"
#include "trajectory_prediction.h"
#include "trajectory_renderer.h"

#include <cmath>
#include <mutex>
#include <optional>
#include <vector>

namespace
{
    constexpr float kAimRayFallbackDistance = 3000.0f;
    constexpr float kAimRaycastDistance = 12000.0f;

    bool g_reportedPredictionFailure = false;
    bool g_reportedPredictionMiss = false;
    bool g_reportedPredictionHit = false;
    bool g_reportedMagicRayResult = false;
    bool g_reportedNoAmmo = false;
    bool g_guidanceWasActive = false;

    std::mutex g_mountedSpellPreviewMutex;
    std::optional<RE::NiPoint3> g_mountedSpellTarget;

    struct AimNodeCache
    {
        const RE::NiAVObject* root{};
        RE::NiAVObject* headMagic{};
        RE::NiAVObject* rightHand{};
        RE::NiAVObject* rightHandFallback{};
    };

    AimNodeCache g_aimNodeCache;

    void SetMountedSpellTarget(std::optional<RE::NiPoint3> target)
    {
        std::scoped_lock lock(g_mountedSpellPreviewMutex);
        g_mountedSpellTarget = std::move(target);
    }

    std::optional<RE::NiPoint3> GetMountedSpellTarget()
    {
        std::scoped_lock lock(g_mountedSpellPreviewMutex);
        return g_mountedSpellTarget;
    }

    float Dot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    RE::NiPoint3 Normalize(const RE::NiPoint3& value)
    {
        const auto lengthSquared = Dot(value, value);
        if (lengthSquared <= 0.0001f) {
            return {};
        }
        const auto inverseLength = 1.0f / std::sqrt(lengthSquared);
        return { value.x * inverseLength, value.y * inverseLength, value.z * inverseLength };
    }

    bool IsCombatSpell(const RE::MagicItem* spell)
    {
        if (!spell) {
            return false;
        }
        const auto type = spell->GetSpellType();
        return (type == RE::MagicSystem::SpellType::kSpell ||
                type == RE::MagicSystem::SpellType::kLeveledSpell ||
                type == RE::MagicSystem::SpellType::kScroll) &&
            spell->GetDelivery() != RE::MagicSystem::Delivery::kSelf &&
            spell->GetCastingType() != RE::MagicSystem::CastingType::kConstantEffect;
    }

    bool IsBowOrCrossbowReadied(const RE::Actor* actor)
    {
        const auto rightWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(false));
        const auto leftWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(true));
        const auto isRangedWeapon = [](const RE::TESObjectWEAP* weapon) {
            return weapon &&
                (weapon->weaponData.animationType == RE::WEAPON_TYPE::kBow ||
                 weapon->weaponData.animationType == RE::WEAPON_TYPE::kCrossbow);
        };
        if (isRangedWeapon(rightWeapon) || isRangedWeapon(leftWeapon)) {
            return Trajectory::IsBowWeaponDrawn();
        }

        static const RE::BSFixedString rightHandTypeVariable("iRightHandType");
        static const RE::BSFixedString leftHandTypeVariable("iLeftHandType");
        std::int32_t rightHandType = -1;
        std::int32_t leftHandType = -1;
        const auto hasRightType = actor->GetGraphVariableInt(rightHandTypeVariable, rightHandType);
        const auto hasLeftType = actor->GetGraphVariableInt(leftHandTypeVariable, leftHandType);
        const bool hasBowHandType =
            (hasRightType && rightHandType == static_cast<std::int32_t>(RE::WEAPON_TYPE::kBow)) ||
            (hasLeftType && leftHandType == static_cast<std::int32_t>(RE::WEAPON_TYPE::kBow));
        if (hasBowHandType) {
            return Trajectory::IsBowWeaponDrawn();
        }

        // Type 9 is also reported for a selected spell. Only use it as a
        // crossbow fallback when no combat spell occupies either hand.
        const auto& selectedSpells = actor->GetActorRuntimeData().selectedSpells;
        const bool combatSpellSelected =
            IsCombatSpell(selectedSpells[RE::Actor::SlotTypes::kLeftHand]) ||
            IsCombatSpell(selectedSpells[RE::Actor::SlotTypes::kRightHand]);
        const bool crossbowHandType =
            (hasRightType && rightHandType == static_cast<std::int32_t>(RE::WEAPON_TYPE::kCrossbow)) ||
            (hasLeftType && leftHandType == static_cast<std::int32_t>(RE::WEAPON_TYPE::kCrossbow));
        return crossbowHandType && !combatSpellSelected && Trajectory::IsBowWeaponDrawn();
    }

    bool IsBowStringInUse(const RE::Actor* actor)
    {
        if (Trajectory::IsBowAttackHeld()) {
            return true;
        }
        switch (actor->GetAttackState()) {
        case RE::ATTACK_STATE_ENUM::kBowDraw:
        case RE::ATTACK_STATE_ENUM::kBowAttached:
        case RE::ATTACK_STATE_ENUM::kBowDrawn:
        case RE::ATTACK_STATE_ENUM::kBowReleasing:
        case RE::ATTACK_STATE_ENUM::kBowReleased:
        case RE::ATTACK_STATE_ENUM::kBowNextAttack:
        case RE::ATTACK_STATE_ENUM::kBowFollowThrough:
            return true;
        default:
            return false;
        }
    }

    struct AimRay
    {
        RE::NiPoint3 origin;
        RE::NiPoint3 direction;
        float yaw;
        bool mountedHandOrigin;
    };

    AimRay GetSharedAimRay(
        RE::PlayerCharacter* player,
        RE::PlayerCamera* playerCamera,
        RE::NiCamera* worldCamera,
        bool useMountedSpellOrigin = false)
    {
        auto direction = Normalize(worldCamera->world.rotate.GetVectorY());
        auto yaw = std::atan2(direction.x, direction.y);
        if (const auto thirdPersonState = playerCamera
                ? skyrim_cast<const RE::ThirdPersonState*>(playerCamera->currentState.get())
                : nullptr) {
            auto aim = TrajectoryPrediction::GetAimRotation(thirdPersonState->rotation);
            if (thirdPersonState->freeRotationEnabled && !player->IsOnMount()) {
                aim.yaw = player->GetAngleZ();
            }
            const auto pitchCos = std::cos(aim.pitch);
            direction = Normalize({
                pitchCos * std::sin(aim.yaw),
                pitchCos * std::cos(aim.yaw),
                -std::sin(aim.pitch)
            });
            yaw = aim.yaw;
        }

        auto origin = worldCamera->world.translate;
        bool mountedHandOrigin = false;
        const auto player3D = player->Get3D();
        static const RE::BSFixedString headMagicNodeName("NPC Head MagicNode [Hmag]");
        if (player3D) {
            if (g_aimNodeCache.root != player3D) {
                g_aimNodeCache = { player3D };
            }
            const RE::NiAVObject* magicNode = nullptr;
            if (player->IsOnMount() && useMountedSpellOrigin) {
                static const RE::BSFixedString rightHandNodeName("NPC R Hand [RHnd]");
                static const RE::BSFixedString rightHandNodeFallbackName("NPC R Hand");
                if (!g_aimNodeCache.rightHand) {
                    g_aimNodeCache.rightHand = player3D->GetObjectByName(rightHandNodeName);
                }
                magicNode = g_aimNodeCache.rightHand;
                if (!magicNode) {
                    if (!g_aimNodeCache.rightHandFallback) {
                        g_aimNodeCache.rightHandFallback = player3D->GetObjectByName(rightHandNodeFallbackName);
                    }
                    magicNode = g_aimNodeCache.rightHandFallback;
                }
                mountedHandOrigin = magicNode != nullptr;
            }
            if (!magicNode) {
                if (!g_aimNodeCache.headMagic) {
                    g_aimNodeCache.headMagic = player3D->GetObjectByName(headMagicNodeName);
                }
                magicNode = g_aimNodeCache.headMagic;
            }
            if (magicNode) {
                if (player->IsOnMount()) {
                    // Horse camera offsets are already represented by the
                    // mounted node transform; do not rotate it by actor yaw.
                    origin = magicNode->world.translate;
                } else {
                    const auto playerPosition = player3D->world.translate;
                    const auto magicOffset = magicNode->world.translate - playerPosition;
                    const auto yawDelta = player->GetAngleZ() - yaw;
                    const auto yawCos = std::cos(yawDelta);
                    const auto yawSin = std::sin(yawDelta);
                    origin = {
                        playerPosition.x + yawCos * magicOffset.x - yawSin * magicOffset.y,
                        playerPosition.y + yawSin * magicOffset.x + yawCos * magicOffset.y,
                        playerPosition.z + magicOffset.z
                    };
                }
            }
        }

        return { origin, direction, yaw, mountedHandOrigin };
    }

    struct MagicAimTarget
    {
        RE::NiPoint3 position;
        bool hit;
    };

    MagicAimTarget GetMagicAimTarget(
        RE::PlayerCharacter* player,
        const AimRay& aimRay)
    {
        const auto& direction = aimRay.direction;
        auto target = aimRay.origin + direction * kAimRayFallbackDistance;
        if (const auto result = TrajectoryPrediction::CastRay(
                player,
                aimRay.origin,
                aimRay.origin + direction * kAimRaycastDistance)) {
            return { *result, true };
        }
        return { target, false };
    }

    bool IsCombatSpellReadied(const RE::Actor* actor, std::size_t slot, const RE::MagicItem* spell)
    {
        if (!IsCombatSpell(spell)) {
            return false;
        }

        // These animation-graph flags describe whether a hand has its magic
        // readied. The selected spell can remain assigned after sheathing.
        static const RE::BSFixedString leftReadyVariable("bMLh_Ready");
        static const RE::BSFixedString rightReadyVariable("bMRh_Ready");
        bool readied = false;
        const auto& readyVariable = slot == RE::Actor::SlotTypes::kLeftHand
            ? leftReadyVariable
            : rightReadyVariable;
        const bool graphReadied = actor->GetGraphVariableBool(readyVariable, readied) && readied;

        return graphReadied;
    }

    bool HasCombatMagicReadied(RE::Actor* actor)
    {
        const auto& selectedSpells = actor->GetActorRuntimeData().selectedSpells;
        const auto leftSpell = selectedSpells[RE::Actor::SlotTypes::kLeftHand];
        const auto rightSpell = selectedSpells[RE::Actor::SlotTypes::kRightHand];
        return IsCombatSpellReadied(actor, RE::Actor::SlotTypes::kLeftHand, leftSpell) ||
            IsCombatSpellReadied(actor, RE::Actor::SlotTypes::kRightHand, rightSpell);
    }

    bool IsSpellCastActive(RE::Actor* actor, std::size_t slot)
    {
        const auto source = slot == RE::Actor::SlotTypes::kLeftHand
            ? RE::MagicSystem::CastingSource::kLeftHand
            : RE::MagicSystem::CastingSource::kRightHand;
        const auto caster = actor->GetMagicCaster(source);
        if (!caster) {
            return false;
        }

        const auto state = caster->state.get();
        return state == RE::MagicCaster::State::kCharging ||
            state == RE::MagicCaster::State::kCasting;
    }

    bool IsSpellCastActive(RE::Actor* actor)
    {
        return IsSpellCastActive(actor, RE::Actor::SlotTypes::kLeftHand) ||
            IsSpellCastActive(actor, RE::Actor::SlotTypes::kRightHand);
    }

    void DrawCameraGuide(
        RE::PlayerCharacter* player,
        RE::IMenu* menu,
        RE::NiCamera* camera,
        bool includeLine)
    {
        const auto aimRay = GetSharedAimRay(player, RE::PlayerCamera::GetSingleton(), camera);
        const auto& direction = aimRay.direction;
        if (Dot(direction, direction) <= 0.0001f) {
            TrajectoryRenderer::Clear();
            return;
        }

        const auto origin = aimRay.origin;
        auto cursor = origin + direction * kAimRayFallbackDistance;
        if (const auto result = TrajectoryPrediction::CastRay(player, origin, origin + direction * kAimRaycastDistance)) {
            cursor = *result;
        }

        static thread_local std::vector<RE::NiPoint3> path;
        path.clear();
        if (includeLine) {
            path.push_back(origin);
            path.push_back(cursor);
        }
        TrajectoryRenderer::Submit(path, menu, camera, &cursor);
    }

}

bool Trajectory::CorrectMountedSpellLaunchData(RE::Projectile::LaunchData& launchData)
{
    const auto player = RE::PlayerCharacter::GetSingleton();
    if (!player || !player->IsOnMount() || launchData.shooter != player ||
        launchData.castingSource != RE::MagicSystem::CastingSource::kRightHand ||
        !launchData.spell || !launchData.projectileBase || !launchData.projectileBase->IsMissile()) {
        return false;
    }

    const auto target = GetMountedSpellTarget();
    if (!target) {
        return false;
    }

    // Keep Skyrim's launch-time hand position: mounted casting animations can
    // move the hand after the last guide update. Aim that actual spawn point at
    // the guide's cached endpoint instead of reusing the potentially stale one.
    const auto launchOrigin = launchData.origin;
    if (!std::isfinite(launchOrigin.x) || !std::isfinite(launchOrigin.y) ||
        !std::isfinite(launchOrigin.z)) {
        return false;
    }
    const auto offset = *target - launchOrigin;
    const auto horizontalDistance = std::hypot(offset.x, offset.y);
    const auto distance = std::hypot(horizontalDistance, offset.z);
    if (!std::isfinite(distance) || distance <= 0.001f) {
        return false;
    }

    // Preserve the engine's current hand spawn point, but force its launch
    // angles toward the exact endpoint used by the rendered guide.
    launchData.angleZ = std::atan2(offset.x, offset.y);
    launchData.angleX = -std::atan2(offset.z, horizontalDistance);
    launchData.useOrigin = true;
    return true;
}

void Trajectory::Update(RE::PlayerCamera* camera)
{
    if (!camera || !RE::PlayerCharacter::GetSingleton()) {
        if (g_guidanceWasActive) {
            SetMountedSpellTarget(std::nullopt);
        }
        TrajectoryRenderer::Clear();
        if (g_guidanceWasActive) {
            const auto menu = TrajectoryHUD::GetMenu();
            if (TrajectoryHUD::EnsureState(menu)) {
                TrajectoryHUD::Reset(menu);
                g_guidanceWasActive = false;
            }
        }
        return;
    }

    // HUDMenu::AdvanceMovie still runs with the game paused. Never predict from a
    // camera that Skyrim did not update this frame; put the crosshair back instead.
    const auto ui = RE::UI::GetSingleton();
    if (!ui || ui->GameIsPaused()) {
        if (g_guidanceWasActive) {
            SetMountedSpellTarget(std::nullopt);
        }
        TrajectoryRenderer::Clear();
        if (g_guidanceWasActive) {
            if (auto* pausedMenu = TrajectoryHUD::GetMenu(); pausedMenu && TrajectoryHUD::EnsureState(pausedMenu)) {
                TrajectoryHUD::Reset(pausedMenu);
                g_guidanceWasActive = false;
            }
        }
        return;
    }

    const auto player = RE::PlayerCharacter::GetSingleton();
    auto* menu = TrajectoryHUD::GetMenu();
    if (!TrajectoryHUD::EnsureState(menu)) {
        if (g_guidanceWasActive) {
            SetMountedSpellTarget(std::nullopt);
        }
        TrajectoryRenderer::Clear();
        static bool reportedHUDUnavailable = false;
        if (!reportedHUDUnavailable) {
            logger::warn("Trajectory update skipped: HUD movie or drawing layer is unavailable.");
            reportedHUDUnavailable = true;
        }
        return;
    }

    const bool usingMagic = HasCombatMagicReadied(player);
    const bool usingBow = !usingMagic && IsBowOrCrossbowReadied(player);

    if (!usingMagic && !usingBow) {
        if (g_guidanceWasActive) {
            SetMountedSpellTarget(std::nullopt);
        }
        TrajectoryRenderer::Clear();
        if (g_guidanceWasActive) {
            TrajectoryHUD::Reset(menu);
        }
        g_guidanceWasActive = false;
        return;
    }

    if (!usingMagic || !player->IsOnMount()) {
        SetMountedSpellTarget(std::nullopt);
    }

    const auto worldCamera = RE::Main::WorldRootCamera();
    if (!worldCamera) {
        if (g_guidanceWasActive) {
            SetMountedSpellTarget(std::nullopt);
        }
        TrajectoryRenderer::Clear();
        if (g_guidanceWasActive) {
            TrajectoryHUD::Reset(menu);
        }
        g_guidanceWasActive = false;
        return;
    }

    g_guidanceWasActive = true;
    // The HUD dot remains useful with Skyrim's first-person aiming view. In
    // third person, use the world-space trajectory without the misleading dot.
    TrajectoryHUD::SetCrosshairVisible(menu, camera->IsInFirstPerson());

    static thread_local std::vector<RE::NiPoint3> path;
    path.clear();
    RE::NiPoint3 hitPosition{};
    bool hit = false;
    std::optional<RE::NiPoint3> projectedHUDCursor;
    if (usingBow) {
        // At bow-ready idle there is no meaningful arrow velocity yet. Show
        // the camera ray and world dot until the bow attack state starts; only
        // then use the ballistic predictor.
        if (!IsBowStringInUse(player)) {
            DrawCameraGuide(player, menu, worldCamera, false);
            return;
        }

        const auto ammo = player->GetCurrentAmmo();
        if (!ammo || !ammo->data.projectile) {
            if (!g_reportedNoAmmo) {
                logger::warn("Trajectory prediction skipped: no current bow ammo/projectile.");
                g_reportedNoAmmo = true;
            }
            DrawCameraGuide(player, menu, worldCamera, true);
            return;
        }
        const char* failureReason = nullptr;
        hit = TrajectoryPrediction::SimulateArrow(player, camera, path, hitPosition, failureReason);
        if (hit && !g_reportedPredictionHit) {
            logger::info(
                "Trajectory prediction endpoint found: impact=({:.1f}, {:.1f}, {:.1f}), pathPoints={}",
                hitPosition.x,
                hitPosition.y,
                hitPosition.z,
                path.size());
            g_reportedPredictionHit = true;
        }
        // SmoothCam suppresses its arrow arc when the predictor misses, but
        // that policy hides every useful diagnostic when this standalone
        // port's Havok ray does not report a collision. Keep the simulated
        // path visible on a miss; SimulateArrow still truncates it at the
        // first collision when one is reported.
        if (path.size() > 1) {
            if (!hit) {
                hitPosition = path.back();
            }
            const auto cursor = hitPosition;
            RE::NiPoint3 projectedCursor{};
            if (TrajectoryRenderer::Submit(path, menu, worldCamera, &cursor, &projectedCursor)) {
                projectedHUDCursor = projectedCursor;
            }
        } else {
            DrawCameraGuide(player, menu, worldCamera, path.empty());
        }
        if (!hit && failureReason && path.empty() && !g_reportedPredictionFailure) {
            logger::warn("Bow trajectory was not simulated: {}.", failureReason);
            g_reportedPredictionFailure = true;
        } else if (!hit && failureReason && !path.empty() && !g_reportedPredictionMiss) {
            logger::warn("Bow trajectory was simulated for {} points, but Havok reported no collision; showing the untruncated prediction.", path.size());
            g_reportedPredictionMiss = true;
        }
    } else if (usingMagic) {
        // Use the same third-person camera-state aim rotation as the bow
        // predictor and SmoothCam's magic ray. Unlike the bow, spells travel
        // along this direction without applying gravity or simulating an arc.
        const auto mountedSpell = player->IsOnMount();
        const auto aimRay = GetSharedAimRay(player, camera, worldCamera, mountedSpell);
        if (Dot(aimRay.direction, aimRay.direction) <= 0.0001f) {
            SetMountedSpellTarget(std::nullopt);
            DrawCameraGuide(player, menu, worldCamera, true);
            TrajectoryHUD::Center(menu);
            return;
        }

        // Match SmoothCam's magic launch point and TranslateFirePostion yaw
        // adjustment so this straight path starts where spell aim originates.
        const auto origin = aimRay.origin;
        // When the collision ray has no hit, keep a visible aim dot along the
        // same camera ray. A real impact always replaces this fallback point.
        const auto aimTarget = GetMagicAimTarget(player, aimRay);
        hitPosition = aimTarget.position;
        hit = aimTarget.hit;
        if (mountedSpell && aimRay.mountedHandOrigin) {
            SetMountedSpellTarget(hitPosition);
        } else {
            SetMountedSpellTarget(std::nullopt);
        }
        if (!g_reportedMagicRayResult) {
            if (hit) {
                logger::info(
                    "Magic aim ray hit: position=({:.1f}, {:.1f}, {:.1f}); submitting world cursor.",
                    hitPosition.x,
                    hitPosition.y,
                    hitPosition.z);
            } else {
                logger::warn("Magic aim ray found no collision; submitted the fallback cursor along the camera ray.");
            }
            g_reportedMagicRayResult = true;
        }
        const auto cursor = hitPosition;
        if (IsSpellCastActive(player)) {
            path = { origin, cursor };
        }
        RE::NiPoint3 projectedCursor{};
        if (TrajectoryRenderer::Submit(path, menu, worldCamera, &cursor, &projectedCursor)) {
            projectedHUDCursor = projectedCursor;
        }
    } else {
        TrajectoryRenderer::Clear();
    }

    if (hit) {
        const auto positioned = projectedHUDCursor
            ? TrajectoryHUD::SetCrosshairPosition(menu, projectedHUDCursor->x, projectedHUDCursor->y)
            : TrajectoryHUD::SetCrosshairPosition(menu, hitPosition);
        if (!positioned) {
            TrajectoryHUD::Center(menu);
        }
    } else {
        TrajectoryHUD::Center(menu);
    }
}
