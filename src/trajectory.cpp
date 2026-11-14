#include "pch.h"
#include "trajectory.h"
#include "trajectory_hud.h"
#include "trajectory_prediction.h"
#include "trajectory_renderer.h"

#include <cmath>
#include <vector>

namespace
{
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

    bool HasCombatMagicEquipped(RE::Actor* actor)
    {
        const auto& selectedSpells = actor->GetActorRuntimeData().selectedSpells;
        const auto leftSpell = selectedSpells[RE::Actor::SlotTypes::kLeftHand];
        const auto rightSpell = selectedSpells[RE::Actor::SlotTypes::kRightHand];
        const auto leftWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(true));
        const auto rightWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(false));
        const auto hasSpellInHand = [](const RE::MagicItem* spell, const RE::TESObjectWEAP* weapon) {
            return IsCombatSpell(spell) &&
                (!weapon || weapon->weaponData.animationType == RE::WEAPON_TYPE::kStaff);
        };
        return hasSpellInHand(leftSpell, leftWeapon) || hasSpellInHand(rightSpell, rightWeapon);
    }

    void DrawTrajectory(
        RE::NiCamera* camera,
        const std::vector<RE::NiPoint3>& path,
        const RE::NiPoint3* cursor = nullptr)
    {
        TrajectoryRenderer::Submit(path, camera, cursor);
    }

    void DrawCameraRay(RE::PlayerCharacter* player, RE::NiCamera* camera)
    {
        const auto direction = Normalize(camera->world.rotate.GetVectorY());
        if (Dot(direction, direction) <= 0.0001f) {
            TrajectoryRenderer::Clear();
            return;
        }

        const auto origin = camera->world.translate;
        auto endpoint = origin + direction * 3000.0f;
        if (const auto result = TrajectoryPrediction::CastRay(player, origin, origin + direction * 12000.0f)) {
            endpoint = *result;
        }

        const auto cursor = endpoint;
        const std::vector<RE::NiPoint3> path{ origin, cursor };
        DrawTrajectory(camera, path, &cursor);
    }

    void DrawCameraCursor(RE::PlayerCharacter* player, RE::NiCamera* camera)
    {
        const auto direction = Normalize(camera->world.rotate.GetVectorY());
        if (Dot(direction, direction) <= 0.0001f) {
            TrajectoryRenderer::Clear();
            return;
        }

        const auto origin = camera->world.translate;
        auto cursor = origin + direction * 3000.0f;
        if (const auto result = TrajectoryPrediction::CastRay(player, origin, origin + direction * 12000.0f)) {
            cursor = *result;
        }

        // Submit only a world-projected cursor while the bow is readied but
        // the string has not been drawn far enough to predict an arrow path.
        DrawTrajectory(camera, {}, &cursor);
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

    const bool usingBow = HasBowOrCrossbow(player);
    const bool usingMagic = !usingBow && HasCombatMagicEquipped(player);

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
        const auto ammo = player->GetCurrentAmmo();
        if (!ammo || !ammo->data.projectile) {
            if (!g_reportedNoAmmo) {
                logger::warn("Trajectory prediction skipped: no current bow ammo/projectile.");
                g_reportedNoAmmo = true;
            }
            DrawCameraCursor(player, worldCamera);
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
            DrawTrajectory(worldCamera, path, &cursor);
        } else {
            DrawCameraCursor(player, worldCamera);
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
        auto direction = worldCamera->world.rotate.GetVectorY();
        auto normalizedDirection = Normalize(direction);
        auto cameraYaw = std::atan2(normalizedDirection.x, normalizedDirection.y);
        if (const auto thirdPersonState = camera
                ? skyrim_cast<const RE::ThirdPersonState*>(camera->currentState.get())
                : nullptr) {
            auto aim = TrajectoryPrediction::GetAimRotation(thirdPersonState->rotation);
            if (thirdPersonState->freeRotationEnabled) {
                aim.yaw = player->GetAngleZ();
            }
            const auto pitchCos = std::cos(aim.pitch);
            normalizedDirection = RE::NiPoint3{
                pitchCos * std::sin(aim.yaw),
                pitchCos * std::cos(aim.yaw),
                -std::sin(aim.pitch)
            };
            cameraYaw = aim.yaw;
        }
        if (Dot(normalizedDirection, normalizedDirection) <= 0.0001f) {
            DrawCameraRay(player, worldCamera);
            TrajectoryHUD::Center(menu);
            return;
        }
        normalizedDirection = Normalize(normalizedDirection);

        // Match SmoothCam's magic launch point and TranslateFirePostion yaw
        // adjustment so this straight path starts where spell aim originates.
        auto origin = worldCamera->world.translate;
        const auto player3D = player->Get3D();
        static const RE::BSFixedString headMagicNodeName("NPC Head MagicNode [Hmag]");
        if (player3D) {
            if (const auto* magicNode = player3D->GetObjectByName(headMagicNodeName)) {
                const auto playerPosition = player3D->world.translate;
                const auto magicOffset = magicNode->world.translate - playerPosition;
                const auto yawDelta = player->GetAngleZ() - cameraYaw;
                const auto yawCos = std::cos(yawDelta);
                const auto yawSin = std::sin(yawDelta);
                origin = {
                    playerPosition.x + yawCos * magicOffset.x - yawSin * magicOffset.y,
                    playerPosition.y + yawSin * magicOffset.x + yawCos * magicOffset.y,
                    playerPosition.z + magicOffset.z
                };
            }
        }
        // When the collision ray has no hit, keep a visible aim dot along the
        // same camera ray. A real impact always replaces this fallback point.
        constexpr float kMagicCursorFallbackRange = 3000.0f;
        hitPosition = origin + normalizedDirection * kMagicCursorFallbackRange;
        if (const auto result = TrajectoryPrediction::CastRay(
                player,
                origin,
                origin + normalizedDirection * 12000.0f)) {
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
        DrawTrajectory(worldCamera, spellPath, &cursor);
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
