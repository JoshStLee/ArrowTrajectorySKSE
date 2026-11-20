#pragma once

#include <vector>

namespace RE
{
    class IMenu;
    class NiCamera;
    class NiPoint3;
    class Projectile;
}

namespace TrajectoryRenderer
{
    void SetActualTraceVisible(bool visible);
    bool IsActualTraceVisible();
    bool Install();
    bool Submit(
        const std::vector<RE::NiPoint3>& path,
        RE::IMenu* menu,
        RE::NiCamera* camera,
        const RE::NiPoint3* cursor = nullptr,
        RE::NiPoint3* cursorHUDPosition = nullptr);
    void BeginActualProjectileTrace(
        const RE::Projectile* projectile,
        const RE::NiPoint3* initialPosition = nullptr);
    bool IsTracingProjectile(const RE::Projectile* projectile);
    bool RecordActualProjectilePosition(
        const RE::Projectile* projectile,
        const RE::NiPoint3& position);
    void EndActualProjectileTrace(const RE::Projectile* projectile);
    void Clear();
}
