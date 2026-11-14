#include "pch.h"
#include "trajectory_hud.h"

namespace TrajectoryHUD
{
namespace
{
    RE::GFxMovieView* g_hudMovie = nullptr;
    double g_crosshairBaseX = 0.0;
    double g_crosshairBaseY = 0.0;
    bool g_crosshairBaseCaptured = false;
    bool g_crosshairVisibilityOverridden = false;
    bool g_crosshairVisibleBeforeOverride = true;

    bool SetScreenPosition(RE::IMenu* menu, float x, float y)
    {
        if (!menu || !menu->uiMovie || !g_crosshairBaseCaptured) {
            return false;
        }

        const auto rect = menu->uiMovie->GetVisibleFrameRect();
        RE::GFxValue crosshair;
        if (!menu->uiMovie->GetVariable(&crosshair, "_root.HUDMovieBaseInstance.Crosshair") ||
            !crosshair.IsDisplayObject())
        {
            return false;
        }

        RE::GFxValue::DisplayInfo position;
        position.SetPosition(
            static_cast<double>(x) - (rect.left + rect.right) * 0.5 + g_crosshairBaseX,
            static_cast<double>(y) - (rect.top + rect.bottom) * 0.5 + g_crosshairBaseY);
        return crosshair.SetDisplayInfo(position);
    }
}

RE::IMenu* GetMenu()
{
    const auto ui = RE::UI::GetSingleton();
    const auto strings = RE::InterfaceStrings::GetSingleton();
    return ui && strings ? ui->GetMenu(strings->hudMenu).get() : nullptr;
}

bool EnsureState(RE::IMenu* menu)
{
    if (!menu || !menu->uiMovie) {
        return false;
    }
    if (g_hudMovie == menu->uiMovie.get() && g_crosshairBaseCaptured) {
        return true;
    }

    g_hudMovie = menu->uiMovie.get();
    g_crosshairBaseCaptured = false;
    g_crosshairVisibilityOverridden = false;

    RE::GFxValue x;
    RE::GFxValue y;
    if (!g_hudMovie->GetVariable(&x, "_root.HUDMovieBaseInstance.CrosshairInstance._x") ||
        !g_hudMovie->GetVariable(&y, "_root.HUDMovieBaseInstance.CrosshairInstance._y") ||
        !x.IsNumber() || !y.IsNumber())
    {
        return false;
    }

    g_crosshairBaseX = x.GetNumber();
    g_crosshairBaseY = y.GetNumber();
    g_crosshairBaseCaptured = true;
    return true;
}

bool SetCrosshairVisible(RE::IMenu* menu, bool visible)
{
    if (!menu || !menu->uiMovie) {
        return false;
    }

    RE::GFxValue crosshair;
    if (!menu->uiMovie->GetVariable(&crosshair, "_root.HUDMovieBaseInstance.Crosshair") ||
        !crosshair.IsDisplayObject())
    {
        return false;
    }

    RE::GFxValue::DisplayInfo displayInfo;
    if (!crosshair.GetDisplayInfo(&displayInfo)) {
        return false;
    }

    if (!visible) {
        if (!g_crosshairVisibilityOverridden) {
            g_crosshairVisibleBeforeOverride = displayInfo.GetVisible();
            g_crosshairVisibilityOverridden = true;
        }
        displayInfo.SetVisible(false);
    } else if (g_crosshairVisibilityOverridden) {
        displayInfo.SetVisible(g_crosshairVisibleBeforeOverride);
    } else {
        return true;
    }

    if (!crosshair.SetDisplayInfo(displayInfo)) {
        if (!visible) {
            g_crosshairVisibilityOverridden = false;
        }
        return false;
    }
    if (visible) {
        g_crosshairVisibilityOverridden = false;
    }
    return true;
}

bool SetCrosshairPosition(RE::IMenu* menu, const RE::NiPoint3& worldPosition)
{
    const auto camera = RE::Main::WorldRootCamera();
    if (!camera) {
        return false;
    }

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    return camera->WorldPtToScreenPt3(worldPosition, x, y, z, 1.0e-5f) && SetScreenPosition(menu, x, y);
}

void Reset(RE::IMenu* menu)
{
    if (!menu || !menu->uiMovie) {
        return;
    }
    const auto rect = menu->uiMovie->GetVisibleFrameRect();
    SetScreenPosition(menu, (rect.left + rect.right) * 0.5f, (rect.top + rect.bottom) * 0.5f);
    SetCrosshairVisible(menu, true);
}

void Center(RE::IMenu* menu)
{
    if (!menu || !menu->uiMovie) {
        return;
    }
    const auto rect = menu->uiMovie->GetVisibleFrameRect();
    SetScreenPosition(menu, (rect.left + rect.right) * 0.5f, (rect.top + rect.bottom) * 0.5f);
}
}
