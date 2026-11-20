#pragma once

#include <vector>

namespace RE
{
    class NiCamera;
    class NiPoint3;
    class Projectile;
}

namespace TrajectoryRenderer
{
    void SetActualTraceVisible(bool visible);
    bool IsActualTraceVisible();
    bool Install();
    void Submit(
        const std::vector<RE::NiPoint3>& path,
        RE::NiCamera* camera,
        const RE::NiPoint3* cursor = nullptr);
    void BeginActualProjectileTrace(
        const RE::Projectile* projectile,
        const RE::NiPoint3* initialPosition = nullptr);
    void RecordActualProjectilePosition(
        const RE::Projectile* projectile,
        const RE::NiPoint3& position);
    void Clear();
}
