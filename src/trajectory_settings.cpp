#include "pch.h"
#include "trajectory_renderer.h"
#include "trajectory_settings.h"

#include <REX/W32/KERNEL32.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <string>

namespace
{
    using RenderFunction = void(__stdcall*)();
    using AddSectionItemFunction = void(*)(const char*, RenderFunction);
    using CheckboxFunction = bool(*)(const char*, bool*);

    bool g_menuRegistered = false;
    bool g_arrowPredictionProfilingEnabled = false;
    AddSectionItemFunction g_addSectionItem = nullptr;
    CheckboxFunction g_checkbox = nullptr;
    std::filesystem::path g_settingsPath;

    std::filesystem::path GetSettingsPath()
    {
        if (const auto logDirectory = SKSE::log::log_directory()) {
            return *logDirectory / "LineOFireSKSE.ini";
        }

        const auto module = REX::W32::GetModuleHandleW(L"LineOFireSKSE.dll");
        if (!module) {
            return {};
        }

        std::array<wchar_t, 32768> modulePath{};
        const auto length = REX::W32::GetModuleFileNameW(
            module,
            modulePath.data(),
            static_cast<std::uint32_t>(modulePath.size()));
        if (length == 0 || length >= modulePath.size()) {
            return {};
        }

        return std::filesystem::path(modulePath.data()).parent_path() / L"LineOFireSKSE.ini";
    }

    bool ReadTraceVisibility()
    {
        if (g_settingsPath.empty()) {
            return true;
        }
        std::ifstream settings(g_settingsPath);
        std::string line;
        while (std::getline(settings, line)) {
            if (line == "ShowActualTrace=0") {
                return false;
            }
            if (line == "ShowActualTrace=1") {
                return true;
            }
        }
        return true;
    }

    bool ReadArrowPredictionProfiling()
    {
        if (g_settingsPath.empty()) {
            return false;
        }
        std::ifstream settings(g_settingsPath);
        std::string line;
        while (std::getline(settings, line)) {
            if (line == "ProfileArrowPrediction=0") {
                return false;
            }
            if (line == "ProfileArrowPrediction=1") {
                return true;
            }
        }
        return false;
    }

    void SaveTraceVisibility(bool visible)
    {
        if (g_settingsPath.empty()) {
            return;
        }
        std::ofstream settings(g_settingsPath, std::ios::trunc);
        if (settings) {
            settings << "ShowActualTrace=" << (visible ? '1' : '0') << '\n';
            settings << "ProfileArrowPrediction=" << (g_arrowPredictionProfilingEnabled ? '1' : '0') << '\n';
        }
    }

    void __stdcall RenderVisibilityPage()
    {
        if (!g_checkbox) {
            return;
        }

        auto visible = TrajectoryRenderer::IsActualTraceVisible();
        if (g_checkbox("Show cyan projectile trace", &visible)) {
            TrajectoryRenderer::SetActualTraceVisible(visible);
            SaveTraceVisibility(visible);
        }
    }
}

void TrajectorySettings::Initialize()
{
    g_settingsPath = GetSettingsPath();
    TrajectoryRenderer::SetActualTraceVisible(ReadTraceVisibility());
    g_arrowPredictionProfilingEnabled = ReadArrowPredictionProfiling();
}

bool TrajectorySettings::IsArrowPredictionProfilingEnabled()
{
    return g_arrowPredictionProfilingEnabled;
}

void TrajectorySettings::RegisterMenu()
{
    if (g_menuRegistered) {
        return;
    }

    // Menu Framework is optional. Its documentation checks installation
    // before registering; a loaded module is a reliable runtime check under
    // mod managers that change the process working directory.
    auto module = REX::W32::GetModuleHandleW(L"SKSEMenuFramework");
    if (!module) {
        module = REX::W32::GetModuleHandleW(L"SKSEMenuFramework.dll");
    }
    if (!module) {
        return;
    }

    g_addSectionItem = reinterpret_cast<AddSectionItemFunction>(
        REX::W32::GetProcAddress(module, "AddSectionItem"));
    g_checkbox = reinterpret_cast<CheckboxFunction>(
        REX::W32::GetProcAddress(module, "igCheckbox"));
    if (!g_addSectionItem || !g_checkbox) {
        logger::warn("SKSE Menu Framework is loaded but required menu functions are unavailable.");
        g_addSectionItem = nullptr;
        g_checkbox = nullptr;
        return;
    }

    // AddSectionItem accepts a full slash-separated path; this is equivalent
    // to SetSection("Line-o-Fire") followed by AddSectionItem("Visibility").
    g_addSectionItem("Line-o-Fire/Visibility", RenderVisibilityPage);
    g_menuRegistered = true;
}
