#include "pch.h"
#include "trajectory.h"
#include "trajectory_prediction.h"
#include "trajectory_renderer.h"

#include <cstring>
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
    bool g_attackInputHeld = false;
    bool g_attackInputListenerRegistered = false;
    bool g_bowWeaponDrawn = false;
    bool g_actionEventListenerRegistered = false;

    class AttackInputSink final : public RE::BSTEventSink<RE::InputEvent*>
    {
    public:
        RE::BSEventNotifyControl ProcessEvent(
            RE::InputEvent* const* event,
            RE::BSTEventSource<RE::InputEvent*>*) override
        {
            if (!event || !*event) {
                return RE::BSEventNotifyControl::kContinue;
            }

            const auto userEvents = RE::UserEvents::GetSingleton();
            if (!userEvents) {
                return RE::BSEventNotifyControl::kContinue;
            }

            for (auto* current = *event; current; current = current->next) {
                if (current->GetEventType() != RE::INPUT_EVENT_TYPE::kButton) {
                    continue;
                }
                const auto button = current->AsButtonEvent();
                if (button && std::strcmp(button->QUserEvent().c_str(), userEvents->rightAttack.c_str()) == 0) {
                    const bool held = button->IsPressed();
                    if (held != g_attackInputHeld) {
                        g_attackInputHeld = held;
                    }
                }
            }
            return RE::BSEventNotifyControl::kContinue;
        }
    };

    AttackInputSink g_attackInputSink;

    class ActionEventSink final : public RE::BSTEventSink<SKSE::ActionEvent>
    {
    public:
        RE::BSEventNotifyControl ProcessEvent(
            const SKSE::ActionEvent* event,
            RE::BSTEventSource<SKSE::ActionEvent>*) override
        {
            const auto player = RE::PlayerCharacter::GetSingleton();
            if (!event || !player || event->actor != player) {
                return RE::BSEventNotifyControl::kContinue;
            }

            const auto actionType = event->type.get();
            if (actionType == SKSE::ActionEvent::Type::kBeginDraw ||
                actionType == SKSE::ActionEvent::Type::kEndDraw) {
                if (!g_bowWeaponDrawn) {
                    g_bowWeaponDrawn = true;
                    logger::debug("Player weapon draw event: guide enabled for an equipped bow.");
                }
            } else if (actionType == SKSE::ActionEvent::Type::kBeginSheathe ||
                       actionType == SKSE::ActionEvent::Type::kEndSheathe) {
                if (g_bowWeaponDrawn) {
                    g_bowWeaponDrawn = false;
                    logger::debug("Player weapon sheathe event: bow guide cleared.");
                }
            }
            return RE::BSEventNotifyControl::kContinue;
        }
    };

    ActionEventSink g_actionEventSink;

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

bool Trajectory::IsBowAttackHeld()
{
    return g_attackInputHeld;
}

bool Trajectory::IsBowWeaponDrawn()
{
    return g_bowWeaponDrawn;
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

    if (!g_attackInputListenerRegistered) {
        const auto inputManager = RE::BSInputDeviceManager::GetSingleton();
        if (inputManager) {
            inputManager->AddEventSink(&g_attackInputSink);
            g_attackInputListenerRegistered = true;
            logger::info("Registered the mapped right-attack observer for bow draw detection.");
        } else {
            logger::warn("Bow draw input observer is unavailable; using the actor's attack-state fallback.");
        }
    }

    if (!g_actionEventListenerRegistered) {
        if (auto* actionEvents = SKSE::GetActionEventSource()) {
            if (const auto player = RE::PlayerCharacter::GetSingleton()) {
                g_bowWeaponDrawn = player->IsWeaponDrawn();
            }
            actionEvents->AddEventSink(&g_actionEventSink);
            g_actionEventListenerRegistered = true;
            logger::info("Registered SKSE ActionEvent draw/sheathe tracking for the bow guide.");
        } else {
            logger::warn("SKSE ActionEvent source is unavailable; bow sheath tracking is inactive.");
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
