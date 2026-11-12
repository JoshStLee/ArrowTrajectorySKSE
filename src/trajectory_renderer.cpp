#include "pch.h"
#include "trajectory_renderer.h"

#include <REX/W32/D3DCOMPILER.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <vector>

namespace
{
    struct ScreenPoint
    {
        float x;
        float y;
        float depth;
    };

    struct Vertex
    {
        float x;
        float y;
        float depth;
        float alpha;
    };

    using PresentFunction = REX::W32::HRESULT (*)(
        REX::W32::IDXGISwapChain*,
        std::uint32_t,
        std::uint32_t);

    std::mutex g_pathMutex;
    std::vector<ScreenPoint> g_screenPath;
    std::vector<Vertex> g_vertices;
    RE::BSGraphics::Renderer* g_renderer = nullptr;
    REX::W32::ID3D11Device* g_device = nullptr;
    REX::W32::ID3D11DeviceContext* g_context = nullptr;
    REX::W32::ID3D11Buffer* g_vertexBuffer = nullptr;
    REX::W32::ID3D11InputLayout* g_inputLayout = nullptr;
    REX::W32::ID3D11VertexShader* g_vertexShader = nullptr;
    REX::W32::ID3D11PixelShader* g_pixelShader = nullptr;
    REX::W32::ID3D11RasterizerState* g_rasterizerState = nullptr;
    REX::W32::ID3D11BlendState* g_blendState = nullptr;
    REX::W32::ID3D11DepthStencilState* g_depthTestState = nullptr;
    REX::W32::ID3D11DepthStencilState* g_depthDisabledState = nullptr;
    PresentFunction g_originalPresent = nullptr;
    bool g_pipelineReady = false;
    bool g_pipelineFailureReported = false;
    bool g_projectionFailureReported = false;
    bool g_installFailureReported = false;
    bool g_presentReported = false;
    bool g_missingRenderTargetReported = false;

    template <class T>
    void Release(T*& object)
    {
        if (object) {
            object->Release();
            object = nullptr;
        }
    }

    bool CompileShader(
        const char* source,
        const char* entryPoint,
        const char* target,
        REX::W32::ID3DBlob** bytecode)
    {
        REX::W32::ID3DBlob* errors = nullptr;
        const auto result = REX::W32::D3DCompile(
            source,
            std::strlen(source),
            nullptr,
            nullptr,
            nullptr,
            entryPoint,
            target,
            0,
            0,
            bytecode,
            &errors);
        if (result < 0) {
            const auto* message = errors ? static_cast<const char*>(errors->GetBufferPointer()) : "no compiler diagnostics";
            logger::error("Trajectory D3D shader compilation failed: {}.", message);
        }
        Release(errors);
        return result >= 0;
    }

    bool CreatePipeline()
    {
        const auto releasePipeline = [] {
            Release(g_vertexBuffer);
            Release(g_inputLayout);
            Release(g_vertexShader);
            Release(g_pixelShader);
            Release(g_rasterizerState);
            Release(g_blendState);
            Release(g_depthTestState);
            Release(g_depthDisabledState);
        };

        constexpr auto shaderSource = R"(
            struct VertexInput { float3 position : POSITION; float alpha : COLOR0; };
            struct VertexOutput { float4 position : SV_Position; float alpha : COLOR0; };
            VertexOutput VSMain(VertexInput input)
            {
                VertexOutput output;
                output.position = float4(input.position, 1.0);
                output.alpha = input.alpha;
                return output;
            }
            float4 PSMain(VertexOutput input) : SV_Target
            {
                return float4(1.0, 1.0, 1.0, input.alpha);
            }
        )";
        REX::W32::ID3DBlob* vertexCode = nullptr;
        REX::W32::ID3DBlob* pixelCode = nullptr;
        if (!CompileShader(shaderSource, "VSMain", "vs_4_0", &vertexCode) ||
            !CompileShader(shaderSource, "PSMain", "ps_4_0", &pixelCode))
        {
            Release(vertexCode);
            Release(pixelCode);
            return false;
        }

        const auto vertexShaderCreated = g_device->CreateVertexShader(
            vertexCode->GetBufferPointer(),
            vertexCode->GetBufferSize(),
            nullptr,
            &g_vertexShader) >= 0;
        const auto pixelShaderCreated = g_device->CreatePixelShader(
            pixelCode->GetBufferPointer(),
            pixelCode->GetBufferSize(),
            nullptr,
            &g_pixelShader) >= 0;
        const REX::W32::D3D11_INPUT_ELEMENT_DESC inputElements[] = {
            {
                "POSITION",
                0,
                REX::W32::DXGI_FORMAT_R32G32B32_FLOAT,
                0,
                0,
                REX::W32::D3D11_INPUT_PER_VERTEX_DATA,
                0
            },
            {
                "COLOR",
                0,
                REX::W32::DXGI_FORMAT_R32_FLOAT,
                0,
                12,
                REX::W32::D3D11_INPUT_PER_VERTEX_DATA,
                0
            }
        };
        const auto inputLayoutCreated = g_device->CreateInputLayout(
            inputElements,
            2,
            vertexCode->GetBufferPointer(),
            vertexCode->GetBufferSize(),
            &g_inputLayout) >= 0;
        Release(vertexCode);
        Release(pixelCode);
        if (!vertexShaderCreated || !pixelShaderCreated || !inputLayoutCreated) {
            releasePipeline();
            return false;
        }

        REX::W32::D3D11_BUFFER_DESC bufferDescription{};
        bufferDescription.byteWidth = static_cast<std::uint32_t>(sizeof(Vertex) * 6 * 128);
        bufferDescription.usage = REX::W32::D3D11_USAGE_DYNAMIC;
        bufferDescription.bindFlags = REX::W32::D3D11_BIND_VERTEX_BUFFER;
        bufferDescription.cpuAccessFlags = REX::W32::D3D11_CPU_ACCESS_WRITE;
        if (g_device->CreateBuffer(&bufferDescription, nullptr, &g_vertexBuffer) < 0) {
            releasePipeline();
            return false;
        }

        REX::W32::D3D11_RASTERIZER_DESC rasterizerDescription{};
        rasterizerDescription.fillMode = REX::W32::D3D11_FILL_SOLID;
        rasterizerDescription.cullMode = REX::W32::D3D11_CULL_NONE;
        rasterizerDescription.depthClipEnable = TRUE;
        if (g_device->CreateRasterizerState(&rasterizerDescription, &g_rasterizerState) < 0) {
            releasePipeline();
            return false;
        }

        REX::W32::D3D11_BLEND_DESC blendDescription{};
        auto& targetBlend = blendDescription.renderTarget[0];
        targetBlend.blendEnable = TRUE;
        targetBlend.srcBlend = REX::W32::D3D11_BLEND_SRC_ALPHA;
        targetBlend.destBlend = REX::W32::D3D11_BLEND_INV_SRC_ALPHA;
        targetBlend.blendOp = REX::W32::D3D11_BLEND_OP_ADD;
        targetBlend.srcBlendAlpha = REX::W32::D3D11_BLEND_ONE;
        targetBlend.destBlendAlpha = REX::W32::D3D11_BLEND_INV_SRC_ALPHA;
        targetBlend.blendOpAlpha = REX::W32::D3D11_BLEND_OP_ADD;
        targetBlend.renderTargetWriteMask = REX::W32::D3D11_COLOR_WRITE_ENABLE_ALL;
        if (g_device->CreateBlendState(&blendDescription, &g_blendState) < 0) {
            releasePipeline();
            return false;
        }

        REX::W32::D3D11_DEPTH_STENCIL_DESC depthDescription{};
        depthDescription.depthEnable = TRUE;
        depthDescription.depthWriteMask = REX::W32::D3D11_DEPTH_WRITE_MASK_ZERO;
        depthDescription.depthFunc = REX::W32::D3D11_COMPARISON_LESS_EQUAL;
        if (g_device->CreateDepthStencilState(&depthDescription, &g_depthTestState) < 0) {
            releasePipeline();
            return false;
        }
        depthDescription.depthEnable = FALSE;
        depthDescription.depthFunc = REX::W32::D3D11_COMPARISON_ALWAYS;
        if (g_device->CreateDepthStencilState(&depthDescription, &g_depthDisabledState) < 0) {
            releasePipeline();
            return false;
        }

        g_pipelineReady = true;
        logger::info("Trajectory Direct3D line renderer initialized.");
        return true;
    }

    void DrawSubmittedPath()
    {
        std::vector<ScreenPoint> path;
        {
            std::scoped_lock lock(g_pathMutex);
            path = g_screenPath;
        }
        if (path.size() < 2) {
            return;
        }
        if (!g_pipelineReady && !CreatePipeline()) {
            if (!g_pipelineFailureReported) {
                logger::error("Trajectory Direct3D line renderer could not create its pipeline.");
                g_pipelineFailureReported = true;
            }
            return;
        }

        const auto* renderWindow = RE::BSGraphics::Renderer::GetCurrentRenderWindow();
        if (!renderWindow || renderWindow->windowWidth <= 0 || renderWindow->windowHeight <= 0) {
            return;
        }
        const auto width = static_cast<float>(renderWindow->windowWidth);
        const auto height = static_cast<float>(renderWindow->windowHeight);

        g_vertices.clear();
        g_vertices.reserve((path.size() - 1) * 6);
        const auto toClip = [width, height](float x, float y, float depth, float alpha) {
            return Vertex{ 2.0f * x / width - 1.0f, 1.0f - 2.0f * y / height, depth, alpha };
        };
        for (std::size_t i = 1; i < path.size(); ++i) {
            auto a = path[i - 1];
            auto b = path[i];
            if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
                !std::isfinite(b.x) || !std::isfinite(b.y))
            {
                continue;
            }

            const auto dx = b.x - a.x;
            const auto dy = b.y - a.y;
            const auto dz = b.depth - a.depth;
            const auto length = std::sqrt(dx * dx + dy * dy);
            if (!std::isfinite(length) || length < 0.01f) {
                continue;
            }

            float t0 = 0.0f;
            float t1 = 1.0f;
            const auto clip = [&t0, &t1](float p, float q) {
                if (std::abs(p) < 1.0e-6f) {
                    return q >= 0.0f;
                }
                const auto t = q / p;
                if (p < 0.0f) {
                    t0 = (std::max)(t0, t);
                } else {
                    t1 = (std::min)(t1, t);
                }
                return t0 <= t1;
            };
            if (!clip(-dx, a.x) || !clip(dx, width - a.x) ||
                !clip(-dy, a.y) || !clip(dy, height - a.y))
            {
                continue;
            }
            b = { a.x + dx * t1, a.y + dy * t1 };
            b.depth = path[i - 1].depth + dz * t1;
            a = { path[i - 1].x + dx * t0, path[i - 1].y + dy * t0, path[i - 1].depth + dz * t0 };

            // SmoothCam fades its arrow tail across the sampled path. Use a
            // matching distance-based taper for this solid white trajectory.
            const auto progress = static_cast<float>(i) / static_cast<float>(path.size() - 1);
            const auto halfWidth = 2.5f + (1.1f - 2.5f) * progress;
            const auto previousProgress = static_cast<float>(i - 1) / static_cast<float>(path.size() - 1);
            const auto alphaAt = [](float distance) {
                // Make the launch direction easy to read, then de-emphasize
                // the far ballistic arc, where small velocity errors accumulate.
                const auto fadeIn = std::clamp(distance / 0.08f, 0.0f, 1.0f);
                const auto fadeOut = 0.92f - 0.78f * distance;
                return std::clamp(fadeIn * fadeOut, 0.0f, 1.0f);
            };
            const auto alphaA = alphaAt(previousProgress + (progress - previousProgress) * t0);
            const auto alphaB = alphaAt(previousProgress + (progress - previousProgress) * t1);
            const auto normalX = -dy / length * halfWidth;
            const auto normalY = dx / length * halfWidth;
            const auto a0 = toClip(a.x + normalX, a.y + normalY, a.depth, alphaA);
            const auto a1 = toClip(a.x - normalX, a.y - normalY, a.depth, alphaA);
            const auto b0 = toClip(b.x + normalX, b.y + normalY, b.depth, alphaB);
            const auto b1 = toClip(b.x - normalX, b.y - normalY, b.depth, alphaB);
            g_vertices.insert(g_vertices.end(), { a0, a1, b0, b0, a1, b1 });
        }
        if (g_vertices.empty()) {
            return;
        }

        REX::W32::D3D11_MAPPED_SUBRESOURCE mapped{};
        if (g_context->Map(
                g_vertexBuffer,
                0,
                REX::W32::D3D11_MAP_WRITE_DISCARD,
                0,
                &mapped) < 0)
        {
            logger::error("Trajectory Direct3D vertex buffer mapping failed.");
            return;
        }
        std::memcpy(mapped.data, g_vertices.data(), g_vertices.size() * sizeof(Vertex));
        g_context->Unmap(g_vertexBuffer, 0);

        REX::W32::ID3D11RenderTargetView* renderTarget = nullptr;
        REX::W32::ID3D11DepthStencilView* depthTarget = nullptr;
        g_context->OMGetRenderTargets(1, &renderTarget, &depthTarget);
        if (!renderTarget) {
            if (!g_missingRenderTargetReported) {
                logger::warn("Trajectory Direct3D draw skipped: Present has no bound render target.");
                g_missingRenderTargetReported = true;
            }
            Release(depthTarget);
            return;
        }

        REX::W32::ID3D11InputLayout* oldInputLayout = nullptr;
        REX::W32::ID3D11Buffer* oldVertexBuffer = nullptr;
        REX::W32::ID3D11VertexShader* oldVertexShader = nullptr;
        REX::W32::ID3D11PixelShader* oldPixelShader = nullptr;
        REX::W32::ID3D11GeometryShader* oldGeometryShader = nullptr;
        REX::W32::ID3D11HullShader* oldHullShader = nullptr;
        REX::W32::ID3D11DomainShader* oldDomainShader = nullptr;
        REX::W32::ID3D11RasterizerState* oldRasterizer = nullptr;
        REX::W32::ID3D11BlendState* oldBlend = nullptr;
        REX::W32::ID3D11DepthStencilState* oldDepthState = nullptr;
        std::uint32_t oldStride = 0;
        std::uint32_t oldOffset = 0;
        std::uint32_t oldStencilRef = 0;
        std::uint32_t oldSampleMask = 0;
        float oldBlendFactors[4]{};
        REX::W32::D3D11_PRIMITIVE_TOPOLOGY oldTopology{};
        REX::W32::D3D11_VIEWPORT oldViewport{};
        std::uint32_t oldViewportCount = 1;
        g_context->IAGetInputLayout(&oldInputLayout);
        g_context->IAGetVertexBuffers(0, 1, &oldVertexBuffer, &oldStride, &oldOffset);
        g_context->IAGetPrimitiveTopology(&oldTopology);
        g_context->VSGetShader(&oldVertexShader, nullptr, nullptr);
        g_context->PSGetShader(&oldPixelShader, nullptr, nullptr);
        g_context->GSGetShader(&oldGeometryShader, nullptr, nullptr);
        g_context->HSGetShader(&oldHullShader, nullptr, nullptr);
        g_context->DSGetShader(&oldDomainShader, nullptr, nullptr);
        g_context->RSGetState(&oldRasterizer);
        g_context->OMGetBlendState(&oldBlend, oldBlendFactors, &oldSampleMask);
        g_context->OMGetDepthStencilState(&oldDepthState, &oldStencilRef);
        g_context->RSGetViewports(&oldViewportCount, &oldViewport);

        REX::W32::ID3D11Buffer* vertexBuffers[] = { g_vertexBuffer };
        const std::uint32_t stride = sizeof(Vertex);
        const std::uint32_t offset = 0;
        const float blendFactors[4]{};
        REX::W32::D3D11_VIEWPORT viewport{
            0.0f,
            0.0f,
            width,
            height,
            0.0f,
            1.0f
        };
        g_context->RSSetViewports(1, &viewport);
        g_context->RSSetState(g_rasterizerState);
        g_context->OMSetBlendState(g_blendState, blendFactors, 0xFFFFFFFF);
        g_context->OMSetDepthStencilState(
            depthTarget ? g_depthTestState : g_depthDisabledState,
            0);
        g_context->IASetInputLayout(g_inputLayout);
        g_context->IASetVertexBuffers(0, 1, vertexBuffers, &stride, &offset);
        g_context->IASetPrimitiveTopology(REX::W32::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        g_context->VSSetShader(g_vertexShader, nullptr, 0);
        g_context->PSSetShader(g_pixelShader, nullptr, 0);
        g_context->GSSetShader(nullptr, nullptr, 0);
        g_context->HSSetShader(nullptr, nullptr, 0);
        g_context->DSSetShader(nullptr, nullptr, 0);
        g_context->Draw(static_cast<std::uint32_t>(g_vertices.size()), 0);
        g_context->IASetInputLayout(oldInputLayout);
        REX::W32::ID3D11Buffer* oldVertexBuffers[] = { oldVertexBuffer };
        g_context->IASetVertexBuffers(0, 1, oldVertexBuffers, &oldStride, &oldOffset);
        g_context->IASetPrimitiveTopology(oldTopology);
        g_context->VSSetShader(oldVertexShader, nullptr, 0);
        g_context->PSSetShader(oldPixelShader, nullptr, 0);
        g_context->GSSetShader(oldGeometryShader, nullptr, 0);
        g_context->HSSetShader(oldHullShader, nullptr, 0);
        g_context->DSSetShader(oldDomainShader, nullptr, 0);
        g_context->RSSetState(oldRasterizer);
        if (oldViewportCount > 0) {
            g_context->RSSetViewports(1, &oldViewport);
        }
        g_context->OMSetBlendState(oldBlend, oldBlendFactors, oldSampleMask);
        g_context->OMSetDepthStencilState(oldDepthState, oldStencilRef);

        Release(oldInputLayout);
        Release(oldVertexBuffer);
        Release(oldVertexShader);
        Release(oldPixelShader);
        Release(oldGeometryShader);
        Release(oldHullShader);
        Release(oldDomainShader);
        Release(oldRasterizer);
        Release(oldBlend);
        Release(oldDepthState);
        Release(renderTarget);
        Release(depthTarget);
    }

    REX::W32::HRESULT PresentHook(
        REX::W32::IDXGISwapChain* swapChain,
        std::uint32_t syncInterval,
        std::uint32_t flags)
    {
        if (!g_presentReported) {
            logger::info("Trajectory Direct3D Present callback is running.");
            g_presentReported = true;
        }
        DrawSubmittedPath();
        return g_originalPresent(swapChain, syncInterval, flags);
    }
}

bool TrajectoryRenderer::Install()
{
    if (g_originalPresent) {
        return true;
    }

    g_renderer = RE::BSGraphics::Renderer::GetSingleton();
    auto* renderWindow = RE::BSGraphics::Renderer::GetCurrentRenderWindow();
    if (!g_renderer || !renderWindow || !renderWindow->swapChain) {
        if (!g_installFailureReported) {
            logger::warn("Trajectory D3D renderer is waiting for Skyrim's render window.");
            g_installFailureReported = true;
        }
        return false;
    }

    g_device = RE::BSGraphics::Renderer::GetDevice();
    g_context = g_renderer->GetRuntimeData().context;
    if (!g_device || !g_context) {
        if (!g_installFailureReported) {
            logger::warn("Trajectory D3D renderer is waiting for Skyrim's D3D device and context.");
            g_installFailureReported = true;
        }
        return false;
    }

    const auto vtableAddress = *reinterpret_cast<std::uintptr_t*>(renderWindow->swapChain);
    REL::Relocation<std::uintptr_t> vtable{ vtableAddress };
    const auto original = vtable.write_vfunc(8, PresentHook);
    if (!original) {
        if (!g_installFailureReported) {
            logger::error("Trajectory D3D renderer failed to hook IDXGISwapChain::Present.");
            g_installFailureReported = true;
        }
        return false;
    }
    g_originalPresent = reinterpret_cast<PresentFunction>(original);
    g_installFailureReported = false;
    logger::info("Trajectory Direct3D Present hook installed.");
    return true;
}

void TrajectoryRenderer::Submit(const std::vector<RE::NiPoint3>& path, RE::NiCamera* camera)
{
    std::vector<ScreenPoint> projected;
    if (!camera || path.size() < 2) {
        Clear();
        return;
    }

    const auto* renderWindow = RE::BSGraphics::Renderer::GetCurrentRenderWindow();
    if (!renderWindow || renderWindow->windowWidth <= 0 || renderWindow->windowHeight <= 0) {
        Clear();
        return;
    }
    const auto ui = RE::UI::GetSingleton();
    const auto strings = RE::InterfaceStrings::GetSingleton();
    const auto menu = ui && strings ? ui->GetMenu(strings->hudMenu) : nullptr;
    if (!menu || !menu->uiMovie) {
        Clear();
        return;
    }
    const auto hudRect = menu->uiMovie->GetVisibleFrameRect();
    const RE::NiRect<float> port(hudRect.left, hudRect.right, hudRect.top, hudRect.bottom);
    const auto hudWidth = hudRect.right - hudRect.left;
    const auto hudHeight = hudRect.bottom - hudRect.top;
    if (hudWidth <= 0.0f || hudHeight <= 0.0f) {
        Clear();
        return;
    }
    const auto scaleX = static_cast<float>(renderWindow->windowWidth) / hudWidth;
    const auto scaleY = static_cast<float>(renderWindow->windowHeight) / hudHeight;

    projected.reserve(path.size());
    for (const auto& point : path) {
        float screenX = 0.0f;
        float screenY = 0.0f;
        float screenDepth = 0.0f;
        if (!RE::NiCamera::WorldPtToScreenPt3(
                camera->GetRuntimeData().worldToCam,
                port,
                point,
                screenX,
                screenY,
                screenDepth,
                1.0e-5f) ||
            !std::isfinite(screenX) || !std::isfinite(screenY) || !std::isfinite(screenDepth))
        {
            const auto invalid = std::numeric_limits<float>::quiet_NaN();
            projected.push_back({ invalid, invalid });
            continue;
        }
        static_cast<void>(screenDepth);
        projected.push_back({
            (screenX - hudRect.left) * scaleX,
            (screenY - hudRect.top) * scaleY,
            screenDepth
        });
    }

    const auto visiblePoints = static_cast<std::size_t>(std::count_if(
        projected.begin(), projected.end(), [](const auto& point) {
            return std::isfinite(point.x) && std::isfinite(point.y);
        }));
    if (visiblePoints < 2) {
        if (!g_projectionFailureReported) {
            logger::warn(
                "Trajectory native projection produced only {} projectable points from {} world points.",
                visiblePoints,
                path.size());
            g_projectionFailureReported = true;
        }
        Clear();
        return;
    }
    {
        std::scoped_lock lock(g_pathMutex);
        g_screenPath = std::move(projected);
    }
}

void TrajectoryRenderer::Clear()
{
    std::scoped_lock lock(g_pathMutex);
    g_screenPath.clear();
}
