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
    bool g_projectionReported = false;
    bool g_installFailureReported = false;
    bool g_presentReported = false;
    bool g_drawReported = false;

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
            struct VertexInput { float3 position : POSITION; };
            float4 VSMain(VertexInput input) : SV_Position
            {
                return float4(input.position, 1.0);
            }
            float4 PSMain() : SV_Target
            {
                return float4(1.0, 1.0, 1.0, 1.0);
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
        const REX::W32::D3D11_INPUT_ELEMENT_DESC inputElement{
            "POSITION",
            0,
            REX::W32::DXGI_FORMAT_R32G32B32_FLOAT,
            0,
            0,
            REX::W32::D3D11_INPUT_PER_VERTEX_DATA,
            0
        };
        const auto inputLayoutCreated = g_device->CreateInputLayout(
            &inputElement,
            1,
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
        const auto toClip = [width, height](float x, float y, float depth) {
            return Vertex{ 2.0f * x / width - 1.0f, 1.0f - 2.0f * y / height, depth };
        };
        constexpr float halfWidth = 0.5f;
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

            const auto normalX = -dy / length * halfWidth;
            const auto normalY = dx / length * halfWidth;
            const auto a0 = toClip(a.x + normalX, a.y + normalY, a.depth);
            const auto a1 = toClip(a.x - normalX, a.y - normalY, a.depth);
            const auto b0 = toClip(b.x + normalX, b.y + normalY, b.depth);
            const auto b1 = toClip(b.x - normalX, b.y - normalY, b.depth);
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
            if (!g_drawReported) {
                logger::warn("Trajectory Direct3D draw skipped: Present has no bound render target.");
                g_drawReported = true;
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
        if (!g_drawReported) {
            logger::info(
                "Trajectory Direct3D draw submitted: {} vertices from {} screen points.",
                g_vertices.size(),
                path.size());
            g_drawReported = true;
        }

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
        if (!g_projectionReported) {
            logger::warn(
                "Trajectory native projection produced only {} projectable points from {} world points.",
                visiblePoints,
                path.size());
            g_projectionReported = true;
        }
        Clear();
        return;
    }
    if (!g_projectionReported) {
        auto minX = (std::numeric_limits<float>::max)();
        auto maxX = (std::numeric_limits<float>::lowest)();
        auto minY = (std::numeric_limits<float>::max)();
        auto maxY = (std::numeric_limits<float>::lowest)();
        auto minDepth = (std::numeric_limits<float>::max)();
        auto maxDepth = (std::numeric_limits<float>::lowest)();
        ScreenPoint first{};
        ScreenPoint last{};
        bool haveFirst = false;
        for (const auto& point : projected) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
                continue;
            }
            if (point.x < minX) minX = point.x;
            if (point.x > maxX) maxX = point.x;
            if (point.y < minY) minY = point.y;
            if (point.y > maxY) maxY = point.y;
            if (point.depth < minDepth) minDepth = point.depth;
            if (point.depth > maxDepth) maxDepth = point.depth;
            if (!haveFirst) {
                first = point;
                haveFirst = true;
            }
            last = point;
        }
        logger::info(
            "Trajectory native path queued: {} of {} projected; HUD rect [{:.1f}, {:.1f}, {:.1f}, {:.1f}], screen bounds X[{:.1f}, {:.1f}] Y[{:.1f}, {:.1f}], depth [{:.5f}, {:.5f}], endpoints ({:.1f}, {:.1f}) -> ({:.1f}, {:.1f}), render {}x{}.",
            visiblePoints,
            path.size(),
            hudRect.left,
            hudRect.top,
            hudRect.right,
            hudRect.bottom,
            minX,
            maxX,
            minY,
            maxY,
            minDepth,
            maxDepth,
            first.x,
            first.y,
            last.x,
            last.y,
            renderWindow->windowWidth,
            renderWindow->windowHeight);
        g_projectionReported = true;
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
