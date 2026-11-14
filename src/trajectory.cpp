#include "pch.h"
#include "trajectory.h"
#include "trajectory_hud.h"
#include "trajectory_prediction.h"
#include "trajectory_renderer.h"

#include <cmath>
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
    };

    AimRay GetSharedAimRay(
        RE::PlayerCharacter* player,
        RE::PlayerCamera* playerCamera,
        RE::NiCamera* worldCamera)
    {
        auto direction = Normalize(worldCamera->world.rotate.GetVectorY());
        auto yaw = std::atan2(direction.x, direction.y);
        if (const auto thirdPersonState = playerCamera
                ? skyrim_cast<const RE::ThirdPersonState*>(playerCamera->currentState.get())
                : nullptr) {
            auto aim = TrajectoryPrediction::GetAimRotation(thirdPersonState->rotation);
            if (thirdPersonState->freeRotationEnabled) {
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
        const auto player3D = player->Get3D();
        static const RE::BSFixedString headMagicNodeName("NPC Head MagicNode [Hmag]");
        if (player3D) {
            if (const auto* magicNode = player3D->GetObjectByName(headMagicNodeName)) {
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
        return { origin, direction, yaw };
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

    void DrawCameraGuide(RE::PlayerCharacter* player, RE::NiCamera* camera, bool includeLine)
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

        std::vector<RE::NiPoint3> path;
        if (includeLine) {
            path = { origin, cursor };
        }
        TrajectoryRenderer::Submit(path, camera, &cursor);
    }

}

void Trajectory::Update(RE::PlayerCamera* camera)
{
    if (!camera || !RE::PlayerCharacter::GetSingleton()) {
        return;
    }

    // HUDMenu::AdvanceMovie still runs with the game paused. Never predict from a
    // camera that Skyrim did not update this frame; put the crosshair back instead.
    const auto ui = RE::UI::GetSingleton();
    if (!ui || ui->GameIsPaused()) {
        if (auto* pausedMenu = TrajectoryHUD::GetMenu(); pausedMenu && TrajectoryHUD::EnsureState(pausedMenu)) {
            TrajectoryRenderer::Clear();
            TrajectoryHUD::Reset(pausedMenu);
        } else {
            TrajectoryRenderer::Clear();
        }
        return;
    }

    const auto player = RE::PlayerCharacter::GetSingleton();
    auto* menu = TrajectoryHUD::GetMenu();
    if (!TrajectoryHUD::EnsureState(menu)) {
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

    const auto worldCamera = RE::Main::WorldRootCamera();
    if (!worldCamera) {
        TrajectoryRenderer::Clear();
        TrajectoryHUD::Reset(menu);
        return;
    }

    // The HUD dot remains useful with Skyrim's first-person aiming view. In
    // third person, use the world-space trajectory without the misleading dot.
    TrajectoryHUD::SetCrosshairVisible(menu, camera->IsInFirstPerson());

    std::vector<RE::NiPoint3> path;
    RE::NiPoint3 hitPosition{};
    bool hit = false;
    if (usingBow) {
        // At bow-ready idle there is no meaningful arrow velocity yet. Show
        // the camera ray and world dot until the bow attack state starts; only
        // then use the ballistic predictor.
        if (!IsBowStringInUse(player)) {
            DrawCameraGuide(player, worldCamera, true);
            return;
        }

        const auto ammo = player->GetCurrentAmmo();
        if (!ammo || !ammo->data.projectile) {
            if (!g_reportedNoAmmo) {
                logger::warn("Trajectory prediction skipped: no current bow ammo/projectile.");
                g_reportedNoAmmo = true;
            }
            DrawCameraGuide(player, worldCamera, true);
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
            TrajectoryRenderer::Submit(path, worldCamera, &cursor);
        } else {
            DrawCameraGuide(player, worldCamera, path.empty());
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
        const auto aimRay = GetSharedAimRay(player, camera, worldCamera);
        auto normalizedDirection = aimRay.direction;
        if (Dot(normalizedDirection, normalizedDirection) <= 0.0001f) {
            DrawCameraGuide(player, worldCamera, true);
            TrajectoryHUD::Center(menu);
            return;
        }
        normalizedDirection = Normalize(normalizedDirection);

        // Match SmoothCam's magic launch point and TranslateFirePostion yaw
        // adjustment so this straight path starts where spell aim originates.
        const auto origin = aimRay.origin;
        // When the collision ray has no hit, keep a visible aim dot along the
        // same camera ray. A real impact always replaces this fallback point.
        hitPosition = origin + normalizedDirection * kAimRayFallbackDistance;
        if (const auto result = TrajectoryPrediction::CastRay(
                player,
                origin,
                origin + normalizedDirection * kAimRaycastDistance)) {
            hitPosition = *result;
            hit = true;
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
        const std::vector<RE::NiPoint3> spellPath{ origin, cursor };
        TrajectoryRenderer::Submit(spellPath, worldCamera, &cursor);
    } else {
        TrajectoryRenderer::Clear();
    }

    if (hit) {
        if (!TrajectoryHUD::SetCrosshairPosition(menu, hitPosition)) {
            TrajectoryHUD::Center(menu);
        }
    } else {
        TrajectoryHUD::Center(menu);
    }
}
