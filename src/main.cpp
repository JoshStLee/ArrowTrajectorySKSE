#include "pch.h"
#include "trajectory.h"
#include "trajectory_settings.h"

namespace
{
    bool g_cameraHooksInstalled = false;
}

static void SKSEMessageHandler(SKSE::MessagingInterface::Message* message)
{
    switch (message->type) {
    case SKSE::MessagingInterface::kNewGame:
    case SKSE::MessagingInterface::kPostLoadGame:
        TrajectorySettings::RegisterMenu();
        if (!g_cameraHooksInstalled) {
            g_cameraHooksInstalled = Trajectory::Install();
            if (!g_cameraHooksInstalled) {
                logger::critical("Failed to install camera-state hooks after game initialization.");
            }
        }
        break;
    }
}

SKSEPluginLoad(const SKSE::LoadInterface* a_skse)
{
    REL::Module::reset();

    auto* messaging = reinterpret_cast<SKSE::MessagingInterface*>(
        a_skse->QueryInterface(SKSE::LoadInterface::kMessaging));
    if (!messaging) {
        logger::critical("Failed to load messaging interface! This error is fatal, plugin will not load.");
        return false;
    }

    logger::info("{} v{}"sv, Plugin::NAME, Plugin::VERSION.string());
    SKSE::Init(a_skse, { .trampoline = true, .trampolineSize = 1 << 10 });
    TrajectorySettings::Initialize();

    if (!messaging->RegisterListener("SKSE", SKSEMessageHandler)) {
        logger::critical("Failed to register the SKSE messaging listener; plugin will not load.");
        return false;
    }

    return true;
}
