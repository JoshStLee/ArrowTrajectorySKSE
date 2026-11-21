#include "pch.h"
#include "trajectory_prediction.h"
#include "trajectory_renderer.h"
#include "trajectory_settings.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <optional>
#include <unordered_map>
#include <vector>

namespace TrajectoryPrediction
{
namespace
{
    struct PhysicsVector
    {
        float x;
        float y;
        float z;

        PhysicsVector operator+(const PhysicsVector& rhs) const { return { x + rhs.x, y + rhs.y, z + rhs.z }; }
        PhysicsVector operator-(const PhysicsVector& rhs) const { return { x - rhs.x, y - rhs.y, z - rhs.z }; }
        PhysicsVector operator*(float scalar) const { return { x * scalar, y * scalar, z * scalar }; }
        PhysicsVector& operator+=(const PhysicsVector& rhs)
        {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }
    };

    using ProjectileFloatFunction = float (*)(RE::Projectile*);

    constexpr float kStep = 1.0f / 20.0f;
    constexpr float kMaxRange = 12000.0f;
    constexpr std::size_t kMaxSegments = 128;
    constexpr float kLinearDamping = 0.099609f;
    constexpr float kGravityScale = 59.0f;
    constexpr float kMinBowPower = 0.2f;
    constexpr std::uint64_t kSmoothCamCollisionMask = 0x40122716;
    // Arrow paths should stop on corpse ragdolls, which use the DeadBip layer
    // omitted by SmoothCam's camera and aim-ray mask.
    constexpr std::uint64_t kArrowPredictionCollisionMask =
        kSmoothCamCollisionMask |
        (1ULL << std::to_underlying(RE::COL_LAYER::kDeadBip));

    struct PreviewLaunchData
    {
        RE::NiPoint3 origin;
        RE::NiPoint3 velocity;
        AimRotation aim;
    };

    struct PendingMountedArrowLaunch
    {
        RE::NiPoint3 velocity;
        RE::ProjectileHandle handle;
    };

    struct PendingMountedSpellLaunch
    {
        RE::NiPoint3 velocity;
        RE::ProjectileHandle handle;
    };

    using ProjectileHandleValue = RE::ProjectileHandle::native_handle_type;

    // Address keys keep the hot lookup cheap; handle generations distinguish a
    // new projectile when Skyrim reuses an object's memory address.
    std::unordered_map<std::uintptr_t, ProjectileHandleValue> g_correctedMountedArrows;
    std::deque<std::uintptr_t> g_recentMountedArrows;
    std::unordered_map<std::uintptr_t, PendingMountedArrowLaunch> g_pendingMountedArrowLaunches;
    std::deque<std::uintptr_t> g_recentPendingMountedArrows;
    std::unordered_map<std::uintptr_t, PendingMountedSpellLaunch> g_pendingMountedSpellLaunches;
    std::deque<std::uintptr_t> g_recentPendingMountedSpells;
    std::optional<PreviewLaunchData> g_lastMountedPreview;

    struct BowDrawTimerEntry
    {
        float drawTime;
        float unknown[3];
    };

    RE::Setting* GetBowTiltSetting();

    PhysicsVector ToVector(const RE::NiPoint3& point)
    {
        return { point.x, point.y, point.z };
    }

    AimRotation GetAimRotationInternal(const RE::NiQuaternion& rotation)
    {
        return {
            -std::atan2(
                2.0f * (rotation.y * rotation.z + rotation.w * rotation.x),
                rotation.w * rotation.w - rotation.x * rotation.x -
                    rotation.y * rotation.y + rotation.z * rotation.z),
            -std::atan2(
                2.0f * (rotation.x * rotation.y + rotation.w * rotation.z),
                rotation.w * rotation.w + rotation.x * rotation.x -
                    rotation.y * rotation.y - rotation.z * rotation.z)
        };
    }

    bool WasMountedArrowCorrected(RE::ArrowProjectile* arrow)
    {
        const auto address = reinterpret_cast<std::uintptr_t>(arrow);
        const auto found = g_correctedMountedArrows.find(address);
        if (found == g_correctedMountedArrows.end()) {
            return false;
        }

        const auto handle = RE::ProjectileHandle(arrow).native_handle();
        if (found->second == handle) {
            return true;
        }

        g_correctedMountedArrows.erase(found);
        std::erase(g_recentMountedArrows, address);
        return false;
    }

    void RememberMountedArrowCorrected(RE::ArrowProjectile* arrow)
    {
        const auto address = reinterpret_cast<std::uintptr_t>(arrow);
        const auto handle = RE::ProjectileHandle(arrow).native_handle();
        g_correctedMountedArrows.insert_or_assign(address, handle);
        std::erase(g_recentMountedArrows, address);
        g_recentMountedArrows.push_back(address);
        constexpr std::size_t kTrackedMountedArrows = 256;
        if (g_recentMountedArrows.size() > kTrackedMountedArrows) {
            g_correctedMountedArrows.erase(g_recentMountedArrows.front());
            g_recentMountedArrows.pop_front();
        }
    }

    template <class Pending>
    void StorePendingMountedLaunch(
        std::unordered_map<std::uintptr_t, Pending>& pendingLaunches,
        std::deque<std::uintptr_t>& recentLaunches,
        std::uintptr_t address,
        Pending pending)
    {
        for (auto it = pendingLaunches.begin(); it != pendingLaunches.end();) {
            if (!static_cast<bool>(it->second.handle.get())) {
                const auto expiredAddress = it->first;
                it = pendingLaunches.erase(it);
                std::erase(recentLaunches, expiredAddress);
            } else {
                ++it;
            }
        }

        if (!pendingLaunches.contains(address)) {
            recentLaunches.push_back(address);
        }
        pendingLaunches.insert_or_assign(address, pending);
        // A projectile can be destroyed before its first update hook consumes
        // the entry, so bound this queue as a fallback cleanup path.
        constexpr std::size_t kMaxPendingMountedLaunches = 64;
        while (recentLaunches.size() > kMaxPendingMountedLaunches) {
            pendingLaunches.erase(recentLaunches.front());
            recentLaunches.pop_front();
        }
    }

    template <class Pending>
    void ErasePendingMountedLaunch(
        std::unordered_map<std::uintptr_t, Pending>& pendingLaunches,
        std::deque<std::uintptr_t>& recentLaunches,
        std::uintptr_t address)
    {
        pendingLaunches.erase(address);
        std::erase(recentLaunches, address);
    }

    void CorrectPlayerArrowVelocityInternal(RE::ArrowProjectile* arrow)
    {
        const auto player = RE::PlayerCharacter::GetSingleton();
        if (!arrow || !player) {
            return;
        }
        auto& projectileData = arrow->GetProjectileRuntimeData();
        if (!projectileData.shooter || projectileData.shooter.get().get() != player ||
            !projectileData.weaponSource || !projectileData.ammoSource) {
            return;
        }

        // Mounted aim can move as the horse camera updates. Match the aim at
        // launch once, then let the arrow keep that velocity through flight.
        if (WasMountedArrowCorrected(arrow)) {
            return;
        }

        const auto playerCamera = RE::PlayerCamera::GetSingleton();
        const auto thirdPersonState = playerCamera
            ? skyrim_cast<const RE::ThirdPersonState*>(playerCamera->currentState.get())
            : nullptr;
        if (!thirdPersonState) {
            return;
        }

        auto aim = GetAimRotationInternal(thirdPersonState->rotation);
        if (thirdPersonState->freeRotationEnabled && !player->IsOnMount()) {
            aim.yaw = player->GetAngleZ();
        }
        // Mounted prediction is built from the third-person camera rotation.
        // Keep the released arrow on that same rotation too; using the arrow's
        // animation angles here makes the shot diverge from the mounted guide.
        const bool useProjectileAngles =
            (projectileData.flags.underlying() & (1u << 31)) != 0 && !player->IsOnMount();
        if (useProjectileAngles) {
            aim.pitch = arrow->GetAngleX();
            aim.yaw = arrow->GetAngleZ();
        } else {
            float tilt = 2.5f;
            if (const auto* setting = GetBowTiltSetting()) {
                tilt = setting->GetFloat();
            }
            aim.pitch -= tilt * 0.017453292519943295f;
        }
        aim.pitch *= -1.0f;

        static REL::Relocation<ProjectileFloatFunction> getWeaponSpeedMultiplier{
            REL::VariantID(42537, 43700, 0)
        };
        static REL::Relocation<ProjectileFloatFunction> getDrawPower{
            REL::VariantID(42536, 43699, 0)
        };
        const auto speed = projectileData.ammoSource->data.projectile
            ? projectileData.ammoSource->data.projectile->data.speed
            : 0.0f;
        if (speed <= 0.0f) {
            return;
        }

        const auto velocityScale =
            getWeaponSpeedMultiplier(arrow) * getDrawPower(arrow) * speed * projectileData.speedMult;
        if (player->IsOnMount() && g_lastMountedPreview) {
            // The mounted preview already simulates the exact launch vector
            // used to draw the visible path. Reuse that vector at release so
            // the arrow cannot take a separate camera or animation angle.
            // Queue it for the movement hook too: Skyrim can rebuild projectile
            // velocity after UpdateArrowFlightPath and before its first step.
            projectileData.linearVelocity = g_lastMountedPreview->velocity;
            const auto address = reinterpret_cast<std::uintptr_t>(arrow);
            const auto handle = RE::ProjectileHandle(arrow);
            StorePendingMountedLaunch(
                g_pendingMountedArrowLaunches,
                g_recentPendingMountedArrows,
                address,
                PendingMountedArrowLaunch{ g_lastMountedPreview->velocity, handle });
        } else {
            const auto pitchCos = std::cos(aim.pitch);
            projectileData.linearVelocity = {
                pitchCos * std::sin(aim.yaw) * velocityScale,
                pitchCos * std::cos(aim.yaw) * velocityScale,
                std::sin(aim.pitch) * velocityScale
            };
        }

        if (player->IsOnMount()) {
            RememberMountedArrowCorrected(arrow);
            if (TrajectoryRenderer::IsActualTraceVisible()) {
                TrajectoryRenderer::BeginActualProjectileTrace(arrow);
            }
        }

    }

    RE::NiPoint3 ToNi(const PhysicsVector& point)
    {
        return { point.x, point.y, point.z };
    }

    class FirstHitCollector final : public RE::hkpClosestRayHitCollector
    {
    public:
        explicit FirstHitCollector(
            const RE::NiAVObject* ignoredObject,
            const RE::NiAVObject* ignoredMountObject,
            std::uint64_t collisionMask = kSmoothCamCollisionMask) :
            ignoredObject(ignoredObject),
            ignoredMountObject(ignoredMountObject),
            collisionMask(collisionMask)
        {}

        void AddRayHit(const RE::hkpCdBody& body, const RE::hkpShapeRayCastCollectorOutput& hitInfo) override
        {
            if (hitInfo.hitFraction >= earlyOutHitFraction) {
                return;
            }

            const auto bodyRoot = [&body] {
                auto result = &body;
                while (result->parent) {
                    result = result->parent;
                }
                return result;
            }();

            const auto collidable = static_cast<const RE::hkpCollidable*>(bodyRoot);
            // The CFilter member stores the packed layer, biped-part, and
            // system-group bits. SmoothCam's mask selects collision-layer
            // indices, so extract the layer before indexing the mask.
            const auto filter = std::to_underlying(
                collidable->broadPhaseHandle.collisionFilterInfo.GetCollisionLayer());
            // Match SmoothCam's RayCollector: only collision layers selected
            // by its Havok filter mask participate in aim and trajectory hits.
            if (filter >= 64 || (collisionMask & (1ULL << filter)) == 0) {
                return;
            }
            using GetAVObject = RE::NiAVObject* (*)(const RE::hkpCdBody*);
            static REL::Relocation<GetAVObject> getAVObject{ REL::VariantID(76160, 77988, 0) };
            const auto avObject = getAVObject(bodyRoot);
            // SmoothCam ignores the player's scene root and treats unresolved
            // Havok bodies as non-hits, matching its crosshair ray behavior.
            if (!avObject || avObject == ignoredObject || avObject == ignoredMountObject) {
                return;
            }

            earlyOutHitFraction = hitInfo.hitFraction;
            closestHitFraction = hitInfo.hitFraction;
            hasHit = true;
        }

        float closestHitFraction = 1.0f;
        bool hasHit = false;
        const RE::NiAVObject* ignoredObject;
        const RE::NiAVObject* ignoredMountObject;
        std::uint64_t collisionMask;
    };

    float Dot(const PhysicsVector& lhs, const PhysicsVector& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    using ProfileClock = std::chrono::steady_clock;

    struct ArrowPredictionProfileWindow
    {
        ProfileClock::time_point startedAt{};
        std::chrono::nanoseconds predictionTotal{};
        std::chrono::nanoseconds pickObjectTotal{};
        std::chrono::nanoseconds maxPrediction{};
        std::chrono::nanoseconds maxPickObject{};
        std::uint64_t updates{};
        std::uint64_t raycasts{};
    };

    void RecordArrowPredictionProfileSample(
        std::chrono::nanoseconds predictionTime,
        std::chrono::nanoseconds pickObjectTime,
        std::chrono::nanoseconds maxPickObjectTime,
        std::uint64_t raycasts)
    {
        static ArrowPredictionProfileWindow window;
        const auto now = ProfileClock::now();
        if (window.updates == 0) {
            window.startedAt = now;
        }

        ++window.updates;
        window.raycasts += raycasts;
        window.predictionTotal += predictionTime;
        window.pickObjectTotal += pickObjectTime;
        window.maxPrediction = (std::max)(window.maxPrediction, predictionTime);
        window.maxPickObject = (std::max)(window.maxPickObject, maxPickObjectTime);

        const auto windowDuration = now - window.startedAt;
        if (windowDuration < std::chrono::seconds(5)) {
            return;
        }

        const auto updates = static_cast<double>(window.updates);
        const auto rays = static_cast<double>(window.raycasts);
        const auto averagePredictionUs =
            std::chrono::duration<double, std::micro>(window.predictionTotal).count() / updates;
        const auto averagePickUs = window.raycasts > 0
            ? std::chrono::duration<double, std::micro>(window.pickObjectTotal).count() / rays
            : 0.0;
        const auto averageRaycastsPerUpdate = rays / updates;
        const auto totalPickUsPerUpdate =
            std::chrono::duration<double, std::micro>(window.pickObjectTotal).count() / updates;
        logger::info(
            "Arrow prediction profile: {:.1f}s, {} updates, avg={:.1f}us max={:.1f}us, "
            "PickObject avg={:.1f}us max={:.1f}us, {:.1f} raycasts/update, PickObject total={:.1f}us/update",
            std::chrono::duration<double>(windowDuration).count(),
            window.updates,
            averagePredictionUs,
            std::chrono::duration<double, std::micro>(window.maxPrediction).count(),
            averagePickUs,
            std::chrono::duration<double, std::micro>(window.maxPickObject).count(),
            averageRaycastsPerUpdate,
            totalPickUsPerUpdate);
        window = {};
    }

    struct ArrowPredictionProfileSample
    {
        explicit ArrowPredictionProfileSample(bool enabled) :
            enabled(enabled),
            startedAt(enabled ? ProfileClock::now() : ProfileClock::time_point{})
        {}

        ~ArrowPredictionProfileSample()
        {
            if (enabled) {
                RecordArrowPredictionProfileSample(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(ProfileClock::now() - startedAt),
                    pickObjectTime,
                    maxPickObjectTime,
                    raycasts);
            }
        }

        void RecordPickObject(std::chrono::nanoseconds elapsed)
        {
            pickObjectTime += elapsed;
            maxPickObjectTime = (std::max)(maxPickObjectTime, elapsed);
            ++raycasts;
        }

        bool enabled;
        ProfileClock::time_point startedAt;
        std::chrono::nanoseconds pickObjectTime{};
        std::chrono::nanoseconds maxPickObjectTime{};
        std::uint64_t raycasts{};
    };

    RE::Setting* GetBowTiltSetting()
    {
        static auto* setting = [] {
            const auto collection = RE::INISettingCollection::GetSingleton();
            return collection ? collection->GetSetting("f3PArrowTiltUpAngle:Combat") : nullptr;
        }();
        return setting;
    }

    float Length(const PhysicsVector& value)
    {
        return std::sqrt(Dot(value, value));
    }

    PhysicsVector Normalize(const PhysicsVector& value)
    {
        const auto length = Length(value);
        return length > 0.0001f ? value * (1.0f / length) : PhysicsVector{};
    }

    struct RaycastContext
    {
        RE::bhkWorld* world{};
        const RE::NiAVObject* player3D{};
        RE::NiPointer<RE::Actor> mount;
        const RE::NiAVObject* mount3D{};
    };

    RaycastContext MakeRaycastContext(RE::PlayerCharacter* player, const RE::NiAVObject* player3D)
    {
        RaycastContext context;
        const auto cell = player->GetParentCell();
        context.world = cell ? cell->GetbhkWorld() : nullptr;
        context.player3D = player3D;
        if (player->IsOnMount() && player->GetMount(context.mount) && context.mount) {
            context.mount3D = context.mount->Get3D();
        }
        return context;
    }

    std::optional<PhysicsVector> CastRayVector(
        const RaycastContext& context,
        const PhysicsVector& start,
        const PhysicsVector& end,
        std::uint64_t collisionMask = kSmoothCamCollisionMask,
        ArrowPredictionProfileSample* profile = nullptr)
    {
        if (!context.world) {
            return std::nullopt;
        }

        const auto delta = end - start;
        // SmoothCam's hkpCastRay uses this Havok world-to-physics scale.
        constexpr float scale = 0.0142875f;
        RE::bhkPickData pickData{};
        pickData.rayInput.from = RE::hkVector4(start.x * scale, start.y * scale, start.z * scale, 1.0f);
        pickData.rayInput.to = RE::hkVector4(0.0f, 0.0f, 0.0f, 0.0f);
        pickData.ray = RE::hkVector4(delta.x * scale, delta.y * scale, delta.z * scale, 1.0f);

        FirstHitCollector collector(context.player3D, context.mount3D, collisionMask);
        collector.Reset();
        pickData.closestRayHitCollector = &collector;
        const auto pickStartedAt = profile ? ProfileClock::now() : ProfileClock::time_point{};
        context.world->PickObject(pickData);
        if (profile) {
            profile->RecordPickObject(std::chrono::duration_cast<std::chrono::nanoseconds>(
                ProfileClock::now() - pickStartedAt));
        }

        if (!collector.hasHit) {
            return std::nullopt;
        }
        return start + delta * collector.closestHitFraction;
    }

    std::optional<PhysicsVector> CastRayVector(
        RE::PlayerCharacter* player,
        const PhysicsVector& start,
        const PhysicsVector& end,
        std::uint64_t collisionMask = kSmoothCamCollisionMask)
    {
        const auto context = MakeRaycastContext(player, player->Get3D());
        return CastRayVector(context, start, end, collisionMask);
    }

    float GetBowDrawStrength(const RE::PlayerCharacter* player)
    {
        const auto& timerArray = player->GetPlayerRuntimeData().unkBA0;
        if (timerArray.size() == 0) {
            return 1.0f;
        }

        const auto* entries = reinterpret_cast<const BowDrawTimerEntry*>(timerArray.data());
        const auto drawTime = entries[timerArray.size() - 1].drawTime;
        if (!std::isfinite(drawTime)) {
            return 1.0f;
        }
        static const REL::Relocation<float*> minDrawTime{ REL::RelocationID(505070, 374232) };
        static const REL::Relocation<float*> minPower{ REL::RelocationID(505072, 374235) };
        const auto minDrawTimeValue = *minDrawTime;
        const auto minPowerValue = *minPower;
        const auto remapped = ((drawTime - minDrawTimeValue) / (1.0f - minDrawTimeValue)) *
            (1.0f - minPowerValue) + minPowerValue;
        return std::clamp(remapped, kMinBowPower, 1.0f);
    }

    RE::NiAVObject* FindArrowNode(RE::Actor* actor)
    {
        const auto root = actor->Get3D();
        if (!root) {
            return nullptr;
        }

        struct CachedNodes
        {
            const RE::NiAVObject* root{};
            RE::NiAVObject* weapon{};
            RE::NiAVObject* rightMagazine{};
            RE::NiAVObject* leftMagazine{};
        };
        static CachedNodes cachedNodes;
        if (cachedNodes.root != root) {
            cachedNodes = { root };
        }

        // Match Crosshair::Manager::FindArrowNode in SmoothCam. In particular,
        // a drawn bow's arrow origin is the WEAPON hand node, while crossbows
        // and fallback rigs use their magazine nodes.
        static const RE::BSFixedString weaponName("WEAPON");
        static const RE::BSFixedString arrowName("Arrow:0");
        static const RE::BSFixedString rightMagazineName("NPC R MagicNode [RMag]");
        static const RE::BSFixedString leftMagazineName("NPC L MagicNode [LMag]");
        const auto getCachedNode = [root](RE::NiAVObject*& cached, const RE::BSFixedString& name) {
            if (!cached) {
                cached = root->GetObjectByName(name);
            }
            return cached;
        };

        const auto rightWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(false));
        const auto leftWeapon = skyrim_cast<const RE::TESObjectWEAP*>(actor->GetEquippedObject(true));
        const auto isCrossbow = [](const RE::TESObjectWEAP* weapon) {
            return weapon && weapon->weaponData.animationType == RE::WEAPON_TYPE::kCrossbow;
        };
        if (isCrossbow(rightWeapon) || isCrossbow(leftWeapon)) {
            if (auto* magazine = getCachedNode(cachedNodes.rightMagazine, rightMagazineName)) {
                if (auto* arrow = magazine->GetObjectByName(arrowName)) {
                    return arrow;
                }
                return magazine;
            }
        }

        auto* handNode = skyrim_cast<RE::NiNode*>(getCachedNode(cachedNodes.weapon, weaponName));
        if (handNode && handNode->children.size() > 0 && handNode->GetObjectByName(arrowName)) {
            return handNode;
        }

        handNode = skyrim_cast<RE::NiNode*>(getCachedNode(cachedNodes.rightMagazine, rightMagazineName));
        if (handNode && handNode->children.size() > 0) {
            if (auto* arrow = handNode->GetObjectByName(arrowName)) {
                return arrow;
            }
        }

        handNode = skyrim_cast<RE::NiNode*>(getCachedNode(cachedNodes.leftMagazine, leftMagazineName));
        if (handNode && handNode->children.size() > 0) {
            if (auto* arrow = handNode->GetObjectByName(arrowName)) {
                return arrow;
            }
        }
        return nullptr;
    }

    bool SimulateArrowInternal(
        RE::PlayerCharacter* player,
        RE::PlayerCamera* playerCamera,
        std::vector<RE::NiPoint3>& path,
        PhysicsVector& impact,
        const char*& failureReason)
    {
        ArrowPredictionProfileSample profile(TrajectorySettings::IsArrowPredictionProfilingEnabled());
        // A preview is frame-local launch state. If this frame cannot produce
        // one, a later mounted release must fall back instead of reusing an
        // aim vector from an earlier camera position or riding direction.
        g_lastMountedPreview.reset();
        failureReason = nullptr;
        const auto ammo = player->GetCurrentAmmo();
        const auto projectile = ammo ? ammo->data.projectile : nullptr;
        if (!projectile || projectile->data.speed <= 0.0f) {
            failureReason = "current ammo has no projectile or a non-positive projectile speed";
            return false;
        }

        const auto thirdPersonState = playerCamera
            ? skyrim_cast<const RE::ThirdPersonState*>(playerCamera->currentState.get())
            : nullptr;
        if (!thirdPersonState) {
            failureReason = "third-person camera state is unavailable";
            return false;
        }
        auto aim = GetAimRotationInternal(thirdPersonState->rotation);
        if (thirdPersonState->freeRotationEnabled && !player->IsOnMount()) {
            aim.yaw = player->GetAngleZ();
        }
        float tiltRadians = 0.0f;
        if (const auto* setting = GetBowTiltSetting()) {
            tiltRadians = setting->GetFloat() * 0.017453292519943295f;
        }
        const auto arrowNode = FindArrowNode(player);
        if (!arrowNode) {
            failureReason = "could not find the bow arrow or hand/magazine node in the player's 3D";
            return false;
        }
        const auto player3D = player->Get3D();
        if (!player3D) {
            failureReason = "player 3D is unavailable";
            return false;
        }
        const auto raycastContext = MakeRaycastContext(player, player3D);
        // Match SmoothCam's Crosshair::Manager::TranslateFirePostion: rotate
        // the arrow's actor-local offset for ordinary third person aiming.
        // Horse camera keeps the node's world position unchanged.
        const auto playerPosition = player3D->world.translate;
        RE::NiPoint3 start = arrowNode->world.translate;
        if (player->IsOnMount()) {
            // Projectile::LaunchArrow runs Unk_A0 on this same weapon node
            // before it creates the projectile. Mirror that launch-origin
            // adjustment; the raw node position is not the arrow's spawn point.
            float launchPitch = 0.0f;
            float launchYaw = 0.0f;
            player->Unk_A0(arrowNode, launchPitch, launchYaw, start);
        } else {
            const auto arrowOffset = arrowNode->world.translate - playerPosition;
            const auto yawDelta = player->GetAngleZ() - aim.yaw;
            const auto yawCos = std::cos(yawDelta);
            const auto yawSin = std::sin(yawDelta);
            start = {
                playerPosition.x + yawCos * arrowOffset.x - yawSin * arrowOffset.y,
                playerPosition.y + yawSin * arrowOffset.x + yawCos * arrowOffset.y,
                playerPosition.z + arrowOffset.z
            };
        }
        // Match SmoothCam's ArrowFixes::mUpdateArrowFlightPath: subtract the
        // game's bow tilt from its aim Euler pitch, then negate pitch when
        // converting the launch rotation to a world-space velocity.
        const auto projectilePitch = -(aim.pitch - tiltRadians);
        const auto pitchCos = std::cos(projectilePitch);
        auto direction = PhysicsVector{
            pitchCos * std::sin(aim.yaw),
            pitchCos * std::cos(aim.yaw),
            std::sin(projectilePitch)
        };
        const auto drawStrength = GetBowDrawStrength(player);
        float weaponSpeed = 1.0f;
        if (const auto weapon = skyrim_cast<const RE::TESObjectWEAP*>(player->GetEquippedObject(false))) {
            weaponSpeed = weapon->weaponData.speed;
        }
        static const REL::Relocation<float*> speedUpperLimit{ REL::RelocationID(505064, 374223) };
        static const REL::Relocation<float*> speedLowerLimit{ REL::RelocationID(505066, 374226) };
        static const REL::Relocation<float*> speedMultiplier{ REL::RelocationID(515530, 401668) };
        const auto speedUpperLimitValue = *speedUpperLimit;
        const auto speedLowerLimitValue = *speedLowerLimit;
        const auto speedMultiplierValue = *speedMultiplier;
        if (speedUpperLimitValue <= weaponSpeed) {
            weaponSpeed = speedUpperLimitValue;
        }
        if (weaponSpeed <= speedLowerLimitValue) {
            weaponSpeed = speedLowerLimitValue;
        }
        const auto weaponSpeedMultiplier =
            (speedLowerLimitValue - weaponSpeed) * speedMultiplierValue + 1.0f;
        auto velocity = direction * (projectile->data.speed * drawStrength * weaponSpeedMultiplier);
        if (player->IsOnMount()) {
            g_lastMountedPreview = PreviewLaunchData{ start, ToNi(velocity), aim };
        }

        auto worldGravity = PhysicsVector{ 0.0f, 0.0f, -9.8f };
        const auto world = raycastContext.world ? raycastContext.world->GetWorld1() : nullptr;
        if (world) {
            const auto& gravity = world->gravity.quad;
            worldGravity = { gravity.m128_f32[0], gravity.m128_f32[1], gravity.m128_f32[2] };
        }
        const auto gravity = worldGravity * (projectile->data.gravity * kGravityScale);
        const auto startPosition = ToVector(start);
        auto position = startPosition;
        auto previous = position;
        path.clear();
        path.reserve(kMaxSegments + 1);
        path.push_back(start);

        // SmoothCam stores 128 points including the launch point, so it makes
        // at most 127 simulation steps.
        for (std::size_t i = 1; i < kMaxSegments; ++i) {
            velocity += (gravity - velocity * kLinearDamping) * kStep;
            position += velocity * kStep;

            if (const auto hit = CastRayVector(
                    raycastContext,
                    previous,
                    position,
                    kArrowPredictionCollisionMask,
                    profile.enabled ? &profile : nullptr)) {
                impact = *hit;
                path.push_back(ToNi(*hit));
                return true;
            }

            path.push_back(ToNi(position));
            previous = position;
            const auto offsetFromStart = position - startPosition;
            if (Dot(offsetFromStart, offsetFromStart) >= kMaxRange * kMaxRange) {
                break;
            }
        }

        failureReason = "predicted path did not hit a collision surface";
        return false;
    }

}

AimRotation GetAimRotation(const RE::NiQuaternion& rotation)
{
    return GetAimRotationInternal(rotation);
}

std::optional<RE::NiPoint3> CastRay(
    RE::PlayerCharacter* player,
    const RE::NiPoint3& start,
    const RE::NiPoint3& end)
{
    const auto toVector = [](const RE::NiPoint3& point) {
        return PhysicsVector{ point.x, point.y, point.z };
    };
    const auto hit = CastRayVector(player, toVector(start), toVector(end));
    if (!hit) {
        return std::nullopt;
    }
    return ToNi(*hit);
}

bool SimulateArrow(
    RE::PlayerCharacter* player,
    RE::PlayerCamera* playerCamera,
    std::vector<RE::NiPoint3>& path,
    RE::NiPoint3& impact,
    const char*& failureReason)
{
    PhysicsVector internalImpact{};
    const auto hit = SimulateArrowInternal(player, playerCamera, path, internalImpact, failureReason);
    impact = ToNi(internalImpact);
    return hit;
}

void CorrectPlayerArrowVelocity(RE::ArrowProjectile* arrow)
{
    CorrectPlayerArrowVelocityInternal(arrow);
}

void ApplyPendingMountedArrowVelocity(RE::ArrowProjectile* arrow)
{
    if (!arrow) {
        return;
    }

    const auto player = RE::PlayerCharacter::GetSingleton();
    const auto& projectileData = arrow->GetProjectileRuntimeData();
    if (!player || !projectileData.shooter || projectileData.shooter.get().get() != player) {
        return;
    }

    const auto address = reinterpret_cast<std::uintptr_t>(arrow);
    const auto pending = g_pendingMountedArrowLaunches.find(address);
    if (pending == g_pendingMountedArrowLaunches.end()) {
        return;
    }
    const auto handle = RE::ProjectileHandle(arrow);
    if (pending->second.handle != handle) {
        ErasePendingMountedLaunch(g_pendingMountedArrowLaunches, g_recentPendingMountedArrows, address);
        return;
    }

    arrow->GetProjectileRuntimeData().linearVelocity = pending->second.velocity;
    const auto& velocity = pending->second.velocity;
    const auto horizontalSpeed = std::hypot(velocity.x, velocity.y);
    const auto arrowPitch = -std::atan2(velocity.z, horizontalSpeed);
    const auto arrowYaw = std::atan2(velocity.x, velocity.y);
    auto angle = arrow->GetAngle();
    angle.x = arrowPitch;
    angle.z = arrowYaw;
    arrow->SetAngle(angle);
    ErasePendingMountedLaunch(g_pendingMountedArrowLaunches, g_recentPendingMountedArrows, address);
}

bool CorrectMountedSpellProjectileVelocity(
    RE::Projectile* projectile,
    const RE::Projectile::LaunchData& launchData)
{
    const auto player = RE::PlayerCharacter::GetSingleton();
    if (!projectile || !player || !player->IsOnMount() ||
        launchData.shooter != player ||
        launchData.castingSource != RE::MagicSystem::CastingSource::kRightHand ||
        !launchData.spell || !launchData.projectileBase || !launchData.projectileBase->IsMissile()) {
        return false;
    }

    const auto pitch = -launchData.angleX;
    const auto yaw = launchData.angleZ;
    const auto pitchCos = std::cos(pitch);
    const auto direction = RE::NiPoint3{
        pitchCos * std::sin(yaw),
        pitchCos * std::cos(yaw),
        std::sin(pitch)
    };
    auto& projectileData = projectile->GetProjectileRuntimeData();
    auto speed = std::hypot(
        projectileData.linearVelocity.x,
        projectileData.linearVelocity.y,
        projectileData.linearVelocity.z);
    if (!std::isfinite(speed) || speed <= 0.001f) {
        speed = launchData.projectileBase->data.speed;
    }
    if (!std::isfinite(speed) || speed <= 0.001f) {
        return false;
    }

    const auto velocity = direction * speed;
    projectileData.linearVelocity = velocity;
    const auto address = reinterpret_cast<std::uintptr_t>(projectile);
    const auto handle = RE::ProjectileHandle(projectile);
    StorePendingMountedLaunch(
        g_pendingMountedSpellLaunches,
        g_recentPendingMountedSpells,
        address,
        PendingMountedSpellLaunch{ velocity, handle });
    return true;
}

void ApplyPendingMountedSpellVelocity(RE::Projectile* projectile)
{
    if (!projectile) {
        return;
    }

    const auto player = RE::PlayerCharacter::GetSingleton();
    const auto& projectileData = projectile->GetProjectileRuntimeData();
    if (!player || !projectileData.shooter || projectileData.shooter.get().get() != player) {
        return;
    }

    const auto address = reinterpret_cast<std::uintptr_t>(projectile);
    const auto pending = g_pendingMountedSpellLaunches.find(address);
    if (pending == g_pendingMountedSpellLaunches.end()) {
        return;
    }
    const auto handle = RE::ProjectileHandle(projectile);
    if (pending->second.handle != handle) {
        ErasePendingMountedLaunch(g_pendingMountedSpellLaunches, g_recentPendingMountedSpells, address);
        return;
    }

    projectile->GetProjectileRuntimeData().linearVelocity = pending->second.velocity;
    ErasePendingMountedLaunch(g_pendingMountedSpellLaunches, g_recentPendingMountedSpells, address);
}

bool CorrectMountedArrowLaunchData(RE::Projectile::LaunchData& launchData)
{
    const auto player = RE::PlayerCharacter::GetSingleton();
    if (!player || !player->IsOnMount() || launchData.shooter != player ||
        !launchData.weaponSource || !launchData.ammoSource || !g_lastMountedPreview) {
        return false;
    }

    const auto& velocity = g_lastMountedPreview->velocity;
    const auto& origin = g_lastMountedPreview->origin;
    if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y) || !std::isfinite(velocity.z) ||
        !std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z) ||
        std::hypot(velocity.x, velocity.y, velocity.z) <= 0.001f) {
        return false;
    }

    // Correct the launch parameters before Skyrim creates the projectile.
    // This makes the engine's initial arrow orientation and spawn point use
    // the same aim vector as the mounted trajectory preview in every direction.
    launchData.origin = origin;
    launchData.angleZ = std::atan2(velocity.x, velocity.y);
    launchData.angleX = -std::atan2(velocity.z, std::hypot(velocity.x, velocity.y));
    launchData.useOrigin = true;
    return true;
}
}
