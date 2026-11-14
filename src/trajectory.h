#pragma once

namespace RE
{
    class PlayerCamera;
}

namespace Trajectory
{
    bool Install();
    bool IsBowAttackHeld();
    bool IsBowWeaponDrawn();
    void Update(RE::PlayerCamera* camera);
}
