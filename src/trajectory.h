#pragma once

namespace RE
{
    class PlayerCamera;
}

namespace Trajectory
{
    bool Install();
    void Update(RE::PlayerCamera* camera);
}
