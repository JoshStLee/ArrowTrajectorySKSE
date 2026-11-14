#include "pch.h"
#include "trajectory.h"
#include "trajectory_prediction.h"
#include "trajectory_renderer.h"

#include <unordered_map>

extern "C"
{
    int WINAPI MH_Initialize();
    int WINAPI MH_CreateHook(void* target, void* detour, void** original);
    int WINAPI MH_EnableHook(void* target);
}

namespace
{
    using CameraStateUpdate = void (*)(RE::TESCameraState*, RE::BSTSmartPointer<RE::TESCameraState>&);
    using ArrowFlightPathUpdate = void (*)(RE::Projectile*);
    using MenuAdvanceMovie = void (*)(RE::HUDMenu*, float, std::uint32_t);

    std::unordered_map<std::uintptr_t, CameraStateUpdate> g_originalCameraStateUpdates;
    ArrowFlightPathUpdate g_originalArrowFlightPathUpdate = nullptr;
    MenuAdvanceMovie g_originalHUDAdvanceMovie = nullptr;
    bool g_arrowFlightPathHookInstalled = false;
    bool g_hudAdvanceHookInstalled = false;
    bool g_reportedFirstUpdate = false;

    void ArrowFlightPathUpdateHook(RE::Projectile* projectile)
    {
        if (g_originalArrowFlightPathUpdate) {
            g_originalArrowFlightPathUpdate(projectile);
        }
        TrajectoryPrediction::CorrectPlayerArrowVelocity(skyrim_cast<RE::ArrowProjectile*>(projectile));
    }

    void HUDAdvanceMovieHook(RE::HUDMenu* menu, float interval, std::uint32_t currentTime)
    {
        if (g_originalHUDAdvanceMovie) {
            g_originalHUDAdvanceMovie(menu, interval, currentTime);
        }
        TrajectoryRenderer::Install();
        Trajectory::Update(RE::PlayerCamera::GetSingleton());
    }

    void CameraStateUpdateHook(
        RE::TESCameraState* state,
        RE::BSTSmartPointer<RE::TESCameraState>& nextState)
    {
        const auto vtableAddress = *reinterpret_cast<std::uintptr_t*>(state);
        const auto original = g_originalCameraStateUpdates.find(vtableAddress);
        if (original == g_originalCameraStateUpdates.end()) {
            logger::error("Camera-state update hook received an unregistered vtable: {:X}.", vtableAddress);
            return;
        }

        original->second(state, nextState);
        TrajectoryRenderer::Install();
        if (!g_reportedFirstUpdate) {
            logger::info("Camera-state update hook is running.");
            g_reportedFirstUpdate = true;
        }
        if (!g_hudAdvanceHookInstalled) {
            Trajectory::Update(RE::PlayerCamera::GetSingleton());
        }
    }
}

bool Trajectory::Install()
{
    TrajectoryRenderer::Install();

    if (!g_hudAdvanceHookInstalled) {
        REL::Relocation<std::uintptr_t> vtable{ RE::VTABLE_HUDMenu[0] };
        const auto original = vtable.write_vfunc(5, HUDAdvanceMovieHook);
        if (!original) {
            logger::error("Failed to hook HUDMenu::AdvanceMovie; trajectory updates fall back to camera-state updates.");
        } else {
            g_originalHUDAdvanceMovie = reinterpret_cast<MenuAdvanceMovie>(original);
            g_hudAdvanceHookInstalled = true;
            logger::info("Trajectory updates run after camera placement via HUDMenu::AdvanceMovie.");
        }
    }

    if (!g_arrowFlightPathHookInstalled) {
        constexpr int kMhOk = 0;
        constexpr int kMhAlreadyInitialized = 1;
        const auto initializeStatus = MH_Initialize();
        if (initializeStatus != kMhOk && initializeStatus != kMhAlreadyInitialized) {
            logger::error("Failed to initialize MinHook for SmoothCam-style arrow correction: status={}.", static_cast<int>(initializeStatus));
        } else {
            static REL::Relocation<std::uintptr_t> updateArrowFlightPath{ REL::RelocationID(42998, 44184) };
            const auto createStatus = MH_CreateHook(
                reinterpret_cast<void*>(updateArrowFlightPath.address()),
                reinterpret_cast<void*>(ArrowFlightPathUpdateHook),
                reinterpret_cast<void**>(&g_originalArrowFlightPathUpdate));
            if (createStatus != kMhOk) {
                logger::error("Failed to create SmoothCam-compatible UpdateArrowFlightPath hook: status={}", static_cast<int>(createStatus));
            } else {
                const auto enableStatus = MH_EnableHook(reinterpret_cast<void*>(updateArrowFlightPath.address()));
                if (enableStatus != kMhOk) {
                    logger::error("Failed to enable SmoothCam-compatible UpdateArrowFlightPath hook: status={}", static_cast<int>(enableStatus));
                } else {
                    g_arrowFlightPathHookInstalled = true;
                    logger::info("SmoothCam-style player arrow velocity hook installed on UpdateArrowFlightPath.");
                }
            }
        }
    }

    if (g_hudAdvanceHookInstalled) {
        return true;
    }

    const auto camera = RE::PlayerCamera::GetSingleton();
    if (!camera) {
        logger::critical("Failed to hook camera-state updates: PlayerCamera singleton is unavailable.");
        return false;
    }

    std::size_t hookedVTables = 0;
    for (const auto& state : camera->GetRuntimeData().cameraStates) {
        auto* cameraState = state.get();
        if (!cameraState) {
            continue;
        }

        const auto vtableAddress = *reinterpret_cast<std::uintptr_t*>(cameraState);
        if (g_originalCameraStateUpdates.contains(vtableAddress)) {
            continue;
        }

        REL::Relocation<std::uintptr_t> vtable{ vtableAddress };
        const auto original = vtable.write_vfunc(3, CameraStateUpdateHook);
        if (!original) {
            logger::critical("Failed to hook camera-state update vtable {:X}.", vtableAddress);
            return false;
        }
        g_originalCameraStateUpdates.emplace(vtableAddress, reinterpret_cast<CameraStateUpdate>(original));
        ++hookedVTables;
    }

    if (hookedVTables == 0) {
        logger::critical("Failed to hook camera-state updates: no initialized camera states were available.");
        return false;
    }

    logger::info("Camera-state update hooks installed on {} vtables.", hookedVTables);
    return true;
}
