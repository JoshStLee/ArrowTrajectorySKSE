#pragma once

#include "RE/P/Projectile.h"

namespace RE
{
    class PlayerCamera;
}

namespace Trajectory
{
    bool Install();
    bool IsBowAttackHeld();
    bool IsBowWeaponDrawn();
    bool CorrectMountedSpellLaunchData(RE::Projectile::LaunchData& launchData);
    void Update(RE::PlayerCamera* camera);
}
