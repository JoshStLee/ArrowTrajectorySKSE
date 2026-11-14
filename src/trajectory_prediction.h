#pragma once

#include <optional>
#include <vector>

#include "RE/N/NiPoint3.h"
#include "RE/N/NiQuaternion.h"

namespace RE
{
    class ArrowProjectile;
    class PlayerCamera;
    class PlayerCharacter;
}

namespace TrajectoryPrediction
{
    struct AimRotation
    {
        float pitch;
        float yaw;
    };

    AimRotation GetAimRotation(const RE::NiQuaternion& rotation);

    std::optional<RE::NiPoint3> CastRay(
        RE::PlayerCharacter* player,
        const RE::NiPoint3& start,
        const RE::NiPoint3& end);

    bool SimulateArrow(
        RE::PlayerCharacter* player,
        RE::PlayerCamera* playerCamera,
        std::vector<RE::NiPoint3>& path,
        RE::NiPoint3& impact,
        const char*& failureReason);

    void CorrectPlayerArrowVelocity(RE::ArrowProjectile* arrow);
}
