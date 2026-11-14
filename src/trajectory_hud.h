#pragma once

#include "RE/N/NiPoint3.h"

namespace RE
{
    class IMenu;
}

namespace TrajectoryHUD
{
    RE::IMenu* GetMenu();
    bool EnsureState(RE::IMenu* menu);
    bool SetCrosshairVisible(RE::IMenu* menu, bool visible);
    bool SetCrosshairPosition(RE::IMenu* menu, const RE::NiPoint3& worldPosition);
    void Reset(RE::IMenu* menu);
    void Center(RE::IMenu* menu);
}
