#include "d3d11_host_display.h"
#include "common/assert.h"
#include "common/d3d11/shader_cache.h"
#include "common/d3d11/shader_compiler.h"
#include "common/log.h"
#include "common/string_util.h"
#include "common_host_interface.h"
#include "core/host_interface.h"
#include "core/settings.h"
#include "core/shader_cache_version.h"
#include "display_ps.hlsl.h"
#include "display_ps_alpha.hlsl.h"
#include "display_vs.hlsl.h"
#include "imgui.h"
#include "imgui_impl_dx11.h"
#include "postprocessing_shadergen.h"
#include <array>
#include <dxgi1_5.h>
Log_SetChannel(D3D11HostDisplay);

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")

namespace FrontendCommon {

class D3D11HostDisplayTexture : public HostDisplayTexture
{
public:
  D3D11HostDisplayTexture(D3D11::Texture texture, HostDisplayPixelFormat format, bool dynamic)
    : m_texture(std::move(texture)), m_format(format), m_dynamic(dynamic)
  {
  }
  ~D3D11HostDisplayTexture() override = default;

  void* GetHandle() const override { return m_texture.GetD3DSRV(); }
  u32 GetWidth() const override { return m_texture.GetWidth(); }
  u32 GetHeight() const override { return m_texture.GetHeight(); }
  u32 GetLayers() const override { return 1; }
  u32 GetLevels() const override { return m_texture.GetLevels(); }
  u32 GetSamples() const override { return m_texture.GetSamples(); }
  HostDisplayPixelFormat GetFormat() const override { return m_format; }

  ALWAYS_INLINE ID3D11Texture2D* GetD3DTexture() const { return m_texture.GetD3DTexture(); }
  ALWAYS_INLINE ID3D11ShaderResourceView* GetD3DSRV() const { return m_texture.GetD3DSRV(); }
  ALWAYS_INLINE ID3D11ShaderResourceView* const* GetD3DSRVArray() const { return m_texture.GetD3DSRVArray(); }
  ALWAYS_INLINE bool IsDynamic() const { return m_dynamic; }

private:
  D3D11::Texture m_texture;
  HostDisplayPixelFormat m_format;
  bool m_dynamic;
};

D3D11HostDisplay::D3D11HostDisplay() = default;

D3D11HostDisplay::~D3D11HostDisplay()
{
  AssertMsg(!m_context, "Context should have been destroyed by now");
  AssertMsg(!m_swap_chain, "Swap chain should have been destroyed by now");
}

HostDisplay::RenderAPI D3D11HostDisplay::GetRenderAPI() const
{
  return HostDisplay::RenderAPI::D3D11;
}

void* D3D11HostDisplay::GetRenderDevice() const
{
  return m_device.Get();
}

void* D3D11HostDisplay::GetRenderContext() const
{
  return m_context.Get();
}

bool D3D11HostDisplay::HasRenderDevice() const
{
  return static_cast<bool>(m_device);
}

bool D3D11HostDisplay::HasRenderSurface() const
{
  return static_cast<bool>(m_swap_chain);
}

static constexpr std::array<DXGI_FORMAT, static_cast<u32>(HostDisplayPixelFormat::Count)>
  s_display_pixel_format_mapping = {{DXGI_FORMAT_UNKNOWN, DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_FORMAT_B8G8R8A8_UNORM,
                                     DXGI_FORMAT_B5G6R5_UNORM, DXGI_FORMAT_B5G5R5A1_UNORM}};

std::unique_ptr<HostDisplayTexture> D3D11HostDisplay::CreateTexture(u32 width, u32 height, u32 layers, u32 levels,
                                                                    u32 samples, HostDisplayPixelFormat format,
                                                                    const void* data, u32 data_stride,
                                                                    bool dynamic /* = false */)
{
  if (layers != 1)
    return {};

  D3D11::Texture tex;
  if (!tex.Create(m_device.Get(), width, height, levels, samples,
                  s_display_pixel_format_mapping[static_cast<u32>(format)], D3D11_BIND_SHADER_RESOURCE, data,
                  data_stride, dynamic))
  {
    return {};
  }

  return std::make_unique<D3D11HostDisplayTexture>(std::move(tex), format, dynamic);
}

void D3D11HostDisplay::UpdateTexture(HostDisplayTexture* texture, u32 x, u32 y, u32 width, u32 height,
                                     const void* texture_data, u32 texture_data_stride)
{
  D3D11HostDisplayTexture* d3d11_texture = static_cast<D3D11HostDisplayTexture*>(texture);
  if (!d3d11_texture->IsDynamic())
  {
    const CD3D11_BOX dst_box(x, y, 0, x + width, y + height, 1);
    m_context->UpdateSubresource(d3d11_texture->GetD3DTexture(), 0, &dst_box, texture_data, texture_data_stride,
                                 texture_data_stride * height);
  }
  else
  {
    D3D11_MAPPED_SUBRESOURCE sr;
    HRESULT hr = m_context->Map(d3d11_texture->GetD3DTexture(), 0, D3D11_MAP_WRITE_DISCARD, 0, &sr);
    if (FAILED(hr))
      Panic("Failed to map dynamic host display texture");

    char* dst_ptr = static_cast<char*>(sr.pData) + (y * sr.RowPitch) + (x * sizeof(u32));
    const char* src_ptr = static_cast<const char*>(texture_data);
    if (sr.RowPitch == texture_data_stride)
    {
      std::memcpy(dst_ptr, src_ptr, texture_data_stride * height);
    }
    else
    {
      for (u32 row = 0; row < height; row++)
      {
        std::memcpy(dst_ptr, src_ptr, width * sizeof(u32));
        src_ptr += texture_data_stride;
        dst_ptr += sr.RowPitch;
      }
    }

    m_context->Unmap(d3d11_texture->GetD3DTexture(), 0);
  }
}

bool D3D11HostDisplay::DownloadTexture(const void* texture_handle, HostDisplayPixelFormat texture_format, u32 x, u32 y,
                                       u32 width, u32 height, void* out_data, u32 out_data_stride)
{
  ID3D11ShaderResourceView* srv =
    const_cast<ID3D11ShaderResourceView*>(static_cast<const ID3D11ShaderResourceView*>(texture_handle));
  ComPtr<ID3D11Resource> srv_resource;
  D3D11_SHADER_RESOURCE_VIEW_DESC srv_desc;
  srv->GetResource(srv_resource.GetAddressOf());
  srv->GetDesc(&srv_desc);

  if (!m_readback_staging_texture.EnsureSize(m_context.Get(), width, height, srv_desc.Format, false))
    return false;

  m_readback_staging_texture.CopyFromTexture(m_context.Get(), srv_resource.Get(), 0, x, y, 0, 0, width, height);

  if (srv_desc.Format == DXGI_FORMAT_B5G6R5_UNORM || srv_desc.Format == DXGI_FORMAT_B5G5R5A1_UNORM)
  {
    return m_readback_staging_texture.ReadPixels<u16>(m_context.Get(), 0, 0, width, height, out_data_stride,
                                                      static_cast<u16*>(out_data));
  }
  else
  {
    return m_readback_staging_texture.ReadPixels<u32>(m_context.Get(), 0, 0, width, height, out_data_stride,
                                                      static_cast<u32*>(out_data));
  }
}

bool D3D11HostDisplay::SupportsDisplayPixelFormat(HostDisplayPixelFormat format) const
{
  const DXGI_FORMAT dfmt = s_display_pixel_format_mapping[static_cast<u32>(format)];
  if (dfmt == DXGI_FORMAT_UNKNOWN)
    return false;

  UINT support = 0;
  const UINT required = D3D11_FORMAT_SUPPORT_TEXTURE2D | D3D11_FORMAT_SUPPORT_SHADER_SAMPLE;
  return (SUCCEEDED(m_device->CheckFormatSupport(dfmt, &support)) && ((support & required) == required));
}

bool D3D11HostDisplay::BeginSetDisplayPixels(HostDisplayPixelFormat format, u32 width, u32 height, void** out_buffer,
                                             u32* out_pitch)
{
  ClearDisplayTexture();

  const DXGI_FORMAT dxgi_format = s_display_pixel_format_mapping[static_cast<u32>(format)];
  if (m_display_pixels_texture.GetWidth() < width || m_display_pixels_texture.GetHeight() < height ||
      m_display_pixels_texture.GetFormat() != dxgi_format)
  {
    if (!m_display_pixels_texture.Create(m_device.Get(), width, height, 1, 1, dxgi_format, D3D11_BIND_SHADER_RESOURCE,
                                         nullptr, 0, true))
    {
      return false;
    }
  }

  D3D11_MAPPED_SUBRESOURCE sr;
  HRESULT hr = m_context->Map(m_display_pixels_texture.GetD3DTexture(), 0, D3D11_MAP_WRITE_DISCARD, 0, &sr);
  if (FAILED(hr))
  {
    Log_ErrorPrintf("Map pixels texture failed: %08X", hr);
    return false;
  }

  *out_buffer = sr.pData;
  *out_pitch = sr.RowPitch;

  SetDisplayTexture(m_display_pixels_texture.GetD3DSRV(), format, m_display_pixels_texture.GetWidth(),
                    m_display_pixels_texture.GetHeight(), 0, 0, static_cast<u32>(width), static_cast<u32>(height));
  return true;
}

void D3D11HostDisplay::EndSetDisplayPixels()
{
  m_context->Unmap(m_display_pixels_texture.GetD3DTexture(), 0);
}

bool D3D11HostDisplay::GetHostRefreshRate(float* refresh_rate)
{
  if (m_swap_chain && IsFullscreen())
  {
    DXGI_SWAP_CHAIN_DESC desc;
    if (SUCCEEDED(m_swap_chain->GetDesc(&desc)) && desc.BufferDesc.RefreshRate.Numerator > 0 &&
        desc.BufferDesc.RefreshRate.Denominator > 0)
    {
      Log_InfoPrintf("using fs rr: %u %u", desc.BufferDesc.RefreshRate.Numerator,
                     desc.BufferDesc.RefreshRate.Denominator);
      *refresh_rate = static_cast<float>(desc.BufferDesc.RefreshRate.Numerator) /
                      static_cast<float>(desc.BufferDesc.RefreshRate.Denominator);
      return true;
    }
  }

  return HostDisplay::GetHostRefreshRate(refresh_rate);
}

void D3D11HostDisplay::SetVSync(bool enabled)
{
  m_vsync = enabled;
}

bool D3D11HostDisplay::CreateRenderDevice(const WindowInfo& wi, std::string_view adapter_name, bool debug_device,
                                          bool threaded_presentation)
{
  UINT create_flags = 0;
  if (debug_device)
    create_flags |= D3D11_CREATE_DEVICE_DEBUG;

  ComPtr<IDXGIFactory> temp_dxgi_factory;
#if WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)
  HRESULT hr = CreateDXGIFactory(IID_PPV_ARGS(temp_dxgi_factory.GetAddressOf()));
#else
  HRESULT hr = CreateDXGIFactory2(0, IID_PPV_ARGS(temp_dxgi_factory.GetAddressOf()));
#endif
  if (FAILED(hr))
  {
    Log_ErrorPrintf("Failed to create DXGI factory: 0x%08X", hr);
    return false;
  }

  u32 adapter_index;
  if (!adapter_name.empty())
  {
    AdapterAndModeList adapter_info(GetAdapterAndModeList(temp_dxgi_factory.Get()));
    for (adapter_index = 0; adapter_index < static_cast<u32>(adapter_info.adapter_names.size()); adapter_index++)
    {
      if (adapter_name == adapter_info.adapter_names[adapter_index])
        break;
    }
    if (adapter_index == static_cast<u32>(adapter_info.adapter_names.size()))
    {
      Log_WarningPrintf("Could not find adapter '%s', using first (%s)", std::string(adapter_name).c_str(),
                        adapter_info.adapter_names[0].c_str());
      adapter_index = 0;
    }
  }
  else
  {
    Log_InfoPrintf("No adapter selected, using first.");
    adapter_index = 0;
  }

  ComPtr<IDXGIAdapter> dxgi_adapter;
  hr = temp_dxgi_factory->EnumAdapters(adapter_index, dxgi_adapter.GetAddressOf());
  if (FAILED(hr))
    Log_WarningPrintf("Failed to enumerate adapter %u, using default", adapter_index);

  static constexpr std::array<D3D_FEATURE_LEVEL, 3> requested_feature_levels = {
    {D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_1, D3D_FEATURE_LEVEL_10_0}};

  hr =
    D3D11CreateDevice(dxgi_adapter.Get(), dxgi_adapter ? D3D_DRIVER_TYPE_UNKNOWN : D3D_DRIVER_TYPE_HARDWARE, nullptr,
                      create_flags, requested_feature_levels.data(), static_cast<UINT>(requested_feature_levels.size()),
                      D3D11_SDK_VERSION, m_device.GetAddressOf(), nullptr, m_context.GetAddressOf());

  // we re-grab these later, see below
  dxgi_adapter.Reset();
  temp_dxgi_factory.Reset();

  if (FAILED(hr))
  {
    Log_ErrorPrintf("Failed to create D3D device: 0x%08X", hr);
    return false;
  }

  if (debug_device && IsDebuggerPresent())
  {
    ComPtr<ID3D11InfoQueue> info;
    hr = m_device.As(&info);
    if (SUCCEEDED(hr))
    {
      info->SetBreakOnSeverity(D3D11_MESSAGE_SEVERITY_ERROR, TRUE);
      info->SetBreakOnSeverity(D3D11_MESSAGE_SEVERITY_WARNING, TRUE);
    }
  }

  // we need the specific factory for the device, otherwise MakeWindowAssociation() is flaky.
  ComPtr<IDXGIDevice> dxgi_device;
  if (FAILED(m_device.As(&dxgi_device)) || FAILED(dxgi_device->GetParent(IID_PPV_ARGS(dxgi_adapter.GetAddressOf()))) ||
      FAILED(dxgi_adapter->GetParent(IID_PPV_ARGS(m_dxgi_factory.GetAddressOf()))))
  {
    Log_WarningPrint("Failed to get parent adapter/device/factory");
    return false;
  }

  DXGI_ADAPTER_DESC adapter_desc;
  if (SUCCEEDED(dxgi_adapter->GetDesc(&adapter_desc)))
  {
    char adapter_name_buffer[128];
    const int name_length =
      WideCharToMultiByte(CP_UTF8, 0, adapter_desc.Description, static_cast<int>(std::wcslen(adapter_desc.Description)),
                          adapter_name_buffer, countof(adapter_name_buffer), 0, nullptr);
    if (name_length >= 0)
    {
      adapter_name_buffer[name_length] = 0;
      Log_InfoPrintf("D3D Adapter: %s", adapter_name_buffer);
    }
  }

  m_allow_tearing_supported = false;
  ComPtr<IDXGIFactory5> dxgi_factory5;
  hr = m_dxgi_factory.As(&dxgi_factory5);
  if (SUCCEEDED(hr))
  {
    BOOL allow_tearing_supported = false;
    hr = dxgi_factory5->CheckFeatureSupport(DXGI_FEATURE_PRESENT_ALLOW_TEARING, &allow_tearing_supported,
                                            sizeof(allow_tearing_supported));
    if (SUCCEEDED(hr))
      m_allow_tearing_supported = (allow_tearing_supported == TRUE);
  }

  m_window_info = wi;
  return true;
}

bool D3D11HostDisplay::InitializeRenderDevice(std::string_view shader_cache_directory, bool debug_device,
                                              bool threaded_presentation)
{
  if (m_window_info.type != WindowInfo::Type::Surfaceless && !CreateSwapChain(nullptr))
    return false;

  if (!CreateResources())
    return false;

  return true;
}

void D3D11HostDisplay::DestroyRenderDevice()
{
  DestroyResources();
  DestroyRenderSurface();
  m_context.Reset();
  m_device.Reset();
}

bool D3D11HostDisplay::MakeRenderContextCurrent()
{
  return true;
}

bool D3D11HostDisplay::DoneRenderContextCurrent()
{
  return true;
}

bool D3D11HostDisplay::CreateSwapChain(const DXGI_MODE_DESC* fullscreen_mode)
{
  HRESULT hr;

#if WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)
  if (m_window_info.type != WindowInfo::Type::Win32)
    return false;

  m_using_flip_model_swap_chain =
    fullscreen_mode || !g_host_interface->GetBoolSettingValue("Display", "UseBlitSwapChain", false);

  const HWND window_hwnd = reinterpret_cast<HWND>(m_window_info.window_handle);
  RECT client_rc{};
  GetClientRect(window_hwnd, &client_rc);
  const u32 width = static_cast<u32>(client_rc.right - client_rc.left);
  const u32 height = static_cast<u32>(client_rc.bottom - client_rc.top);

  DXGI_SWAP_CHAIN_DESC swap_chain_desc = {};
  swap_chain_desc.BufferDesc.Width = width;
  swap_chain_desc.BufferDesc.Height = height;
  swap_chain_desc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
  swap_chain_desc.SampleDesc.Count = 1;
  swap_chain_desc.BufferCount = 3;
  swap_chain_desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
  swap_chain_desc.OutputWindow = window_hwnd;
  swap_chain_desc.Windowed = TRUE;
  swap_chain_desc.SwapEffect = m_using_flip_model_swap_chain ? DXGI_SWAP_EFFECT_FLIP_DISCARD : DXGI_SWAP_EFFECT_DISCARD;

  m_using_allow_tearing = (m_allow_tearing_supported && m_using_flip_model_swap_chain && !fullscreen_mode);
  if (m_using_allow_tearing)
    swap_chain_desc.Flags |= DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING;

  if (fullscreen_mode)
  {
    swap_chain_desc.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    swap_chain_desc.Windowed = FALSE;
    swap_chain_desc.BufferDesc = *fullscreen_mode;
  }

  Log_InfoPrintf("Creating a %dx%d %s %s swap chain", swap_chain_desc.BufferDesc.Width,
                 swap_chain_desc.BufferDesc.Height, m_using_flip_model_swap_chain ? "flip-discard" : "discard",
                 swap_chain_desc.Windowed ? "windowed" : "full-screen");

  hr = m_dxgi_factory->CreateSwapChain(m_device.Get(), &swap_chain_desc, m_swap_chain.GetAddressOf());
  if (FAILED(hr) && m_using_flip_model_swap_chain)
  {
    Log_WarningPrintf("Failed to create a flip-discard swap chain, trying discard.");
    swap_chain_desc.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    swap_chain_desc.Flags = 0;
    m_using_flip_model_swap_chain = false;
    m_using_allow_tearing = false;

    hr = m_dxgi_factory->CreateSwapChain(m_device.Get(), &swap_chain_desc, m_swap_chain.GetAddressOf());
    if (FAILED(hr))
    {
      Log_ErrorPrintf("CreateSwapChain failed: 0x%08X", hr);
      return false;
    }
  }

  ComPtr<IDXGIFactory> dxgi_factory;
  hr = m_swap_chain->GetParent(IID_PPV_ARGS(dxgi_factory.GetAddressOf()));
  if (SUCCEEDED(hr))
  {
    hr = dxgi_factory->MakeWindowAssociation(swap_chain_desc.OutputWindow, DXGI_MWA_NO_WINDOW_CHANGES);
    if (FAILED(hr))
      Log_WarningPrintf("MakeWindowAssociation() to disable ALT+ENTER failed");
  }
#else
  if (m_window_info.type != WindowInfo::Type::WinRT)
    return false;

  ComPtr<IDXGIFactory2> factory2;
  hr = m_dxgi_factory.As(&factory2);
  if (FAILED(hr))
  {
    Log_ErrorPrintf("Failed to get DXGI factory: %08X", hr);
    return false;
  }

  DXGI_SWAP_CHAIN_DESC1 swap_chain_desc = {};
  swap_chain_desc.Width = m_window_info.surface_width;
  swap_chain_desc.Height = m_window_info.surface_height;
  swap_chain_desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
  swap_chain_desc.SampleDesc.Count = 1;
  swap_chain_desc.BufferCount = 3;
  swap_chain_desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
  swap_chain_desc.SwapEffect = m_using_flip_model_swap_chain ? DXGI_SWAP_EFFECT_FLIP_DISCARD : DXGI_SWAP_EFFECT_DISCARD;

  m_using_allow_tearing = (m_allow_tearing_supported && m_using_flip_model_swap_chain && !fullscreen_mode);
  if (m_using_allow_tearing)
    swap_chain_desc.Flags |= DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING;

  ComPtr<IDXGISwapChain1> swap_chain1;
  hr = factory2->CreateSwapChainForCoreWindow(m_device.Get(), static_cast<IUnknown*>(m_window_info.window_handle),
                                              &swap_chain_desc, nullptr, swap_chain1.GetAddressOf());
  if (FAILED(hr))
  {
    Log_ErrorPrintf("CreateSwapChainForCoreWindow failed: 0x%08X", hr);
    return false;
  }

  m_swap_chain = swap_chain1;
#endif

  return CreateSwapChainRTV();
}

bool D3D11HostDisplay::CreateSwapChainRTV()
{
  ComPtr<ID3D11Texture2D> backbuffer;
  HRESULT hr = m_swap_chain->GetBuffer(0, IID_PPV_ARGS(backbuffer.GetAddressOf()));
  if (FAILED(hr))
  {
    Log_ErrorPrintf("GetBuffer for RTV failed: 0x%08X", hr);
    return false;
  }

  D3D11_TEXTURE2D_DESC backbuffer_desc;
  backbuffer->GetDesc(&backbuffer_desc);

  CD3D11_RENDER_TARGET_VIEW_DESC rtv_desc(D3D11_RTV_DIMENSION_TEXTURE2D, backbuffer_desc.Format, 0, 0,
                                          backbuffer_desc.ArraySize);
  hr = m_device->CreateRenderTargetView(backbuffer.Get(), &rtv_desc, m_swap_chain_rtv.GetAddressOf());
  if (FAILED(hr))
  {
    Log_ErrorPrintf("CreateRenderTargetView for swap chain failed: 0x%08X", hr);
    return false;
  }

  m_window_info.surface_width = backbuffer_desc.Width;
  m_window_info.surface_height = backbuffer_desc.Height;
  Log_InfoPrintf("Swap chain buffer size: %ux%u", m_window_info.surface_width, m_window_info.surface_height);

  if (m_window_info.type == WindowInfo::Type::Win32)
  {
    BOOL fullscreen = FALSE;
    DXGI_SWAP_CHAIN_DESC desc;
    if (SUCCEEDED(m_swap_chain->GetFullscreenState(&fullscreen, nullptr)) && fullscreen &&
        SUCCEEDED(m_swap_chain->GetDesc(&desc)))
    {
      m_window_info.surface_refresh_rate = static_cast<float>(desc.BufferDesc.RefreshRate.Numerator) /
                                           static_cast<float>(desc.BufferDesc.RefreshRate.Denominator);
    }
    else
    {
      m_window_info.surface_refresh_rate = 0.0f;
    }
  }

  return true;
}

bool D3D11HostDisplay::ChangeRenderWindow(const WindowInfo& new_wi)
{
  DestroyRenderSurface();

  m_window_info = new_wi;
  return CreateSwapChain(nullptr);
}

void D3D11HostDisplay::DestroyRenderSurface()
{
  if (IsFullscreen())
    SetFullscreen(false, 0, 0, 0.0f);

  m_swap_chain_rtv.Reset();
  m_swap_chain.Reset();
}

void D3D11HostDisplay::ResizeRenderWindow(s32 new_window_width, s32 new_window_height)
{
  if (!m_swap_chain)
    return;

  m_swap_chain_rtv.Reset();

  HRESULT hr = m_swap_chain->ResizeBuffers(0, 0, 0, DXGI_FORMAT_UNKNOWN,
                                           m_using_allow_tearing ? DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING : 0);
  if (FAILED(hr))
    Log_ErrorPrintf("ResizeBuffers() failed: 0x%08X", hr);

  if (!CreateSwapChainRTV())
    Panic("Failed to recreate swap chain RTV after resize");
}

bool D3D11HostDisplay::SupportsFullscreen() const
{
  return true;
}

bool D3D11HostDisplay::IsFullscreen()
{
  BOOL is_fullscreen = FALSE;
  return (m_swap_chain && SUCCEEDED(m_swap_chain->GetFullscreenState(&is_fullscreen, nullptr)) && is_fullscreen);
}

bool D3D11HostDisplay::SetFullscreen(bool fullscreen, u32 width, u32 height, float refresh_rate)
{
  if (!m_swap_chain)
    return false;

  BOOL is_fullscreen = FALSE;
  HRESULT hr = m_swap_chain->GetFullscreenState(&is_fullscreen, nullptr);
  if (!fullscreen)
  {
    // leaving fullscreen
    if (is_fullscreen)
      return SUCCEEDED(m_swap_chain->SetFullscreenState(FALSE, nullptr));
    else
      return true;
  }

  IDXGIOutput* output;
  if (FAILED(hr = m_swap_chain->GetContainingOutput(&output)))
    return false;

  DXGI_SWAP_CHAIN_DESC current_desc;
  hr = m_swap_chain->GetDesc(&current_desc);
  if (FAILED(hr))
    return false;

  DXGI_MODE_DESC new_mode = current_desc.BufferDesc;
  new_mode.Width = width;
  new_mode.Height = height;
  new_mode.RefreshRate.Numerator = static_cast<UINT>(std::floor(refresh_rate * 1000.0f));
  new_mode.RefreshRate.Denominator = 1000u;

  DXGI_MODE_DESC closest_mode;
  if (FAILED(hr = output->FindClosestMatchingMode(&new_mode, &closest_mode, nullptr)) ||
      new_mode.Format != current_desc.BufferDesc.Format)
  {
    Log_ErrorPrintf("Failed to find closest matching mode, hr=%08X", hr);
    return false;
  }

  if (new_mode.Width == current_desc.BufferDesc.Width && new_mode.Height == current_desc.BufferDesc.Height &&
      new_mode.RefreshRate.Numerator == current_desc.BufferDesc.RefreshRate.Numerator &&
      new_mode.RefreshRate.Denominator == current_desc.BufferDesc.RefreshRate.Denominator)
  {
    Log_InfoPrintf("Fullscreen mode already set");
    return true;
  }

  m_swap_chain_rtv.Reset();
  m_swap_chain.Reset();

  if (!CreateSwapChain(&closest_mode))
  {
    Log_ErrorPrintf("Failed to create a fullscreen swap chain");
    if (!CreateSwapChain(nullptr))
      Panic("Failed to recreate windowed swap chain");

    return false;
  }

  return true;
}

bool D3D11HostDisplay::CreateResources()
{
  HRESULT hr;

  m_display_vertex_shader =
    D3D11::ShaderCompiler::CreateVertexShader(m_device.Get(), s_display_vs_bytecode, sizeof(s_display_vs_bytecode));
  m_display_pixel_shader =
    D3D11::ShaderCompiler::CreatePixelShader(m_device.Get(), s_display_ps_bytecode, sizeof(s_display_ps_bytecode));
  m_display_alpha_pixel_shader = D3D11::ShaderCompiler::CreatePixelShader(m_device.Get(), s_display_ps_alpha_bytecode,
                                                                          sizeof(s_display_ps_alpha_bytecode));
  if (!m_display_vertex_shader || !m_display_pixel_shader || !m_display_alpha_pixel_shader)
    return false;

  if (!m_display_uniform_buffer.Create(m_device.Get(), D3D11_BIND_CONSTANT_BUFFER, DISPLAY_UNIFORM_BUFFER_SIZE))
    return false;

  CD3D11_RASTERIZER_DESC rasterizer_desc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
  rasterizer_desc.CullMode = D3D11_CULL_NONE;
  hr = m_device->CreateRasterizerState(&rasterizer_desc, m_display_rasterizer_state.GetAddressOf());
  if (FAILED(hr))
    return false;

  CD3D11_DEPTH_STENCIL_DESC depth_stencil_desc = CD3D11_DEPTH_STENCIL_DESC(CD3D11_DEFAULT());
  depth_stencil_desc.DepthEnable = FALSE;
  depth_stencil_desc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
  hr = m_device->CreateDepthStencilState(&depth_stencil_desc, m_display_depth_stencil_state.GetAddressOf());
  if (FAILED(hr))
    return false;

  CD3D11_BLEND_DESC blend_desc = CD3D11_BLEND_DESC(CD3D11_DEFAULT());
  hr = m_device->CreateBlendState(&blend_desc, m_display_blend_state.GetAddressOf());
  if (FAILED(hr))
    return false;

  blend_desc.RenderTarget[0] = {TRUE,
                                D3D11_BLEND_SRC_ALPHA,
                                D3D11_BLEND_INV_SRC_ALPHA,
                                D3D11_BLEND_OP_ADD,
                                D3D11_BLEND_ONE,
                                D3D11_BLEND_ZERO,
                                D3D11_BLEND_OP_ADD,
                                D3D11_COLOR_WRITE_ENABLE_ALL};
  hr = m_device->CreateBlendState(&blend_desc, m_software_cursor_blend_state.GetAddressOf());
  if (FAILED(hr))
    return false;

  CD3D11_SAMPLER_DESC sampler_desc = CD3D11_SAMPLER_DESC(CD3D11_DEFAULT());
  sampler_desc.Filter = D3D11_FILTER_MIN_MAG_MIP_POINT;
  hr = m_device->CreateSamplerState(&sampler_desc, m_point_sampler.GetAddressOf());
  if (FAILED(hr))
    return false;

  sampler_desc.Filter = D3D11_FILTER_MIN_MAG_LINEAR_MIP_POINT;
  hr = m_device->CreateSamplerState(&sampler_desc, m_linear_sampler.GetAddressOf());
  if (FAILED(hr))
    return false;

  return true;
}

void D3D11HostDisplay::DestroyResources()
{
  m_post_processing_chain.ClearStages();
  m_post_processing_input_texture.Destroy();
  m_post_processing_stages.clear();

  m_display_uniform_buffer.Release();
  m_linear_sampler.Reset();
  m_point_sampler.Reset();
  m_display_alpha_pixel_shader.Reset();
  m_display_pixel_shader.Reset();
  m_display_vertex_shader.Reset();
  m_display_blend_state.Reset();
  m_display_depth_stencil_state.Reset();
  m_display_rasterizer_state.Reset();
}

bool D3D11HostDisplay::CreateImGuiContext()
{
  return ImGui_ImplDX11_Init(m_device.Get(), m_context.Get());
}

void D3D11HostDisplay::DestroyImGuiContext()
{
  ImGui_ImplDX11_Shutdown();
}

bool D3D11HostDisplay::UpdateImGuiFontTexture()
{
  ImGui_ImplDX11_CreateFontsTexture();
  return true;
}

bool D3D11HostDisplay::Render()
{
  if (ShouldSkipDisplayingFrame() || !m_swap_chain)
  {
    if (ImGui::GetCurrentContext())
      ImGui::Render();

    return false;
  }

  static constexpr std::array<float, 4> clear_color = {};
  m_context->ClearRenderTargetView(m_swap_chain_rtv.Get(), clear_color.data());
  m_context->OMSetRenderTargets(1, m_swap_chain_rtv.GetAddressOf(), nullptr);

  RenderDisplay();

  if (ImGui::GetCurrentContext())
    RenderImGui();

  RenderSoftwareCursor();

  if (!m_vsync && m_using_allow_tearing)
    m_swap_chain->Present(0, DXGI_PRESENT_ALLOW_TEARING);
  else
    m_swap_chain->Present(BoolToUInt32(m_vsync), 0);

  return true;
}

bool D3D11HostDisplay::RenderScreenshot(u32 width, u32 height, std::vector<u32>* out_pixels, u32* out_stride,
                                        HostDisplayPixelFormat* out_format)
{
  static constexpr DXGI_FORMAT format = DXGI_FORMAT_R8G8B8A8_UNORM;
  static constexpr HostDisplayPixelFormat hdformat = HostDisplayPixelFormat::RGBA8;

  D3D11::Texture render_texture;
  if (!render_texture.Create(m_device.Get(), width, height, 1, 1, format, D3D11_BIND_RENDER_TARGET) ||
      !m_readback_staging_texture.EnsureSize(m_context.Get(), width, height, format, false))
  {
    return false;
  }

  static constexpr std::array<float, 4> clear_color = {};
  m_context->ClearRenderTargetView(render_texture.GetD3DRTV(), clear_color.data());
  m_context->OMSetRenderTargets(1, render_texture.GetD3DRTVArray(), nullptr);

  if (HasDisplayTexture())
  {
    const auto [left, top, draw_width, draw_height] = CalculateDrawRect(width, height, 0);

    if (!m_post_processing_chain.IsEmpty())
    {
      ApplyPostProcessingChain(render_texture.GetD3DRTV(), left, top, draw_width, draw_height, m_display_texture_handle,
                               m_display_texture_width, m_display_texture_height, m_display_texture_view_x,
                               m_display_texture_view_y, m_display_texture_view_width, m_display_texture_view_height,
                               width, height);
    }
    else
    {
      RenderDisplay(left, top, draw_width, draw_height, m_display_texture_handle, m_display_texture_width,
                    m_display_texture_height, m_display_texture_view_x, m_display_texture_view_y,
                    m_display_texture_view_width, m_display_texture_view_height, m_display_linear_filtering);
    }
  }

  m_context->OMSetRenderTargets(0, nullptr, nullptr);

  m_readback_staging_texture.CopyFromTexture(m_context.Get(), render_texture, 0, 0, 0, 0, 0, width, height);

  if (!m_readback_staging_texture.Map(m_context.Get(), false))
    return false;

  const u32 stride = sizeof(u32) * width;
  out_pixels->resize(width * height);
  *out_stride = stride;
  *out_format = hdformat;

  m_readback_staging_texture.ReadPixels<u32>(0, 0, width, height, stride, out_pixels->data());
  m_readback_staging_texture.Unmap(m_context.Get());
  return true;
}

void D3D11HostDisplay::RenderImGui()
{
  ImGui::Render();
  ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
}

void D3D11HostDisplay::RenderDisplay()
{
  if (!HasDisplayTexture())
    return;

  const auto [left, top, width, height] = CalculateDrawRect(GetWindowWidth(), GetWindowHeight(), m_display_top_margin);

  if (!m_post_processing_chain.IsEmpty())
  {
    ApplyPostProcessingChain(m_swap_chain_rtv.Get(), left, top, width, height, m_display_texture_handle,
                             m_display_texture_width, m_display_texture_height, m_display_texture_view_x,
                             m_display_texture_view_y, m_display_texture_view_width, m_display_texture_view_height,
                             GetWindowWidth(), GetWindowHeight());
    return;
  }

  RenderDisplay(left, top, width, height, m_display_texture_handle, m_display_texture_width, m_display_texture_height,
                m_display_texture_view_x, m_display_texture_view_y, m_display_texture_view_width,
                m_display_texture_view_height, m_display_linear_filtering);
}

void D3D11HostDisplay::RenderDisplay(s32 left, s32 top, s32 width, s32 height, void* texture_handle, u32 texture_width,
                                     s32 texture_height, s32 texture_view_x, s32 texture_view_y, s32 texture_view_width,
                                     s32 texture_view_height, bool linear_filter)
{
  m_context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
  m_context->VSSetShader(m_display_vertex_shader.Get(), nullptr, 0);
  m_context->PSSetShader(m_display_pixel_shader.Get(), nullptr, 0);
  m_context->PSSetShaderResources(0, 1, reinterpret_cast<ID3D11ShaderResourceView**>(&texture_handle));
  m_context->PSSetSamplers(0, 1, linear_filter ? m_linear_sampler.GetAddressOf() : m_point_sampler.GetAddressOf());

  const float position_adjust = m_display_linear_filtering ? 0.5f : 0.0f;
  const float size_adjust = m_display_linear_filtering ? 1.0f : 0.0f;
  const float uniforms[4] = {
    (static_cast<float>(texture_view_x) + position_adjust) / static_cast<float>(texture_width),
    (static_cast<float>(texture_view_y) + position_adjust) / static_cast<float>(texture_height),
    (static_cast<float>(texture_view_width) - size_adjust) / static_cast<float>(texture_width),
    (static_cast<float>(texture_view_height) - size_adjust) / static_cast<float>(texture_height)};
  const auto map = m_display_uniform_buffer.Map(m_context.Get(), m_display_uniform_buffer.GetSize(), sizeof(uniforms));
  std::memcpy(map.pointer, uniforms, sizeof(uniforms));
  m_display_uniform_buffer.Unmap(m_context.Get(), sizeof(uniforms));
  m_context->VSSetConstantBuffers(0, 1, m_display_uniform_buffer.GetD3DBufferArray());

  const CD3D11_VIEWPORT vp(static_cast<float>(left), static_cast<float>(top), static_cast<float>(width),
                           static_cast<float>(height));
  m_context->RSSetViewports(1, &vp);
  m_context->RSSetState(m_display_rasterizer_state.Get());
  m_context->OMSetDepthStencilState(m_display_depth_stencil_state.Get(), 0);
  m_context->OMSetBlendState(m_display_blend_state.Get(), nullptr, 0xFFFFFFFFu);

  m_context->Draw(3, 0);
}

void D3D11HostDisplay::RenderSoftwareCursor()
{
  if (!HasSoftwareCursor())
    return;

  const auto [left, top, width, height] = CalculateSoftwareCursorDrawRect();
  RenderSoftwareCursor(left, top, width, height, m_cursor_texture.get());
}

void D3D11HostDisplay::RenderSoftwareCursor(s32 left, s32 top, s32 width, s32 height,
                                            HostDisplayTexture* texture_handle)
{
  m_context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
  m_context->VSSetShader(m_display_vertex_shader.Get(), nullptr, 0);
  m_context->PSSetShader(m_display_alpha_pixel_shader.Get(), nullptr, 0);
  m_context->PSSetShaderResources(0, 1, static_cast<D3D11HostDisplayTexture*>(texture_handle)->GetD3DSRVArray());
  m_context->PSSetSamplers(0, 1, m_linear_sampler.GetAddressOf());

  const float uniforms[4] = {0.0f, 0.0f, 1.0f, 1.0f};
  const auto map = m_display_uniform_buffer.Map(m_context.Get(), m_display_uniform_buffer.GetSize(), sizeof(uniforms));
  std::memcpy(map.pointer, uniforms, sizeof(uniforms));
  m_display_uniform_buffer.Unmap(m_context.Get(), sizeof(uniforms));
  m_context->VSSetConstantBuffers(0, 1, m_display_uniform_buffer.GetD3DBufferArray());

  const CD3D11_VIEWPORT vp(static_cast<float>(left), static_cast<float>(top), static_cast<float>(width),
                           static_cast<float>(height));
  m_context->RSSetViewports(1, &vp);
  m_context->RSSetState(m_display_rasterizer_state.Get());
  m_context->OMSetDepthStencilState(m_display_depth_stencil_state.Get(), 0);
  m_context->OMSetBlendState(m_software_cursor_blend_state.Get(), nullptr, 0xFFFFFFFFu);

  m_context->Draw(3, 0);
}

HostDisplay::AdapterAndModeList D3D11HostDisplay::StaticGetAdapterAndModeList()
{
  ComPtr<IDXGIFactory> dxgi_factory;
#if WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)
  HRESULT hr = CreateDXGIFactory(IID_PPV_ARGS(dxgi_factory.GetAddressOf()));
#else
  HRESULT hr = CreateDXGIFactory2(0, IID_PPV_ARGS(dxgi_factory.GetAddressOf()));
#endif
  if (FAILED(hr))
    return {};

  return GetAdapterAndModeList(dxgi_factory.Get());
}

HostDisplay::AdapterAndModeList D3D11HostDisplay::GetAdapterAndModeList(IDXGIFactory* dxgi_factory)
{
  AdapterAndModeList adapter_info;
  ComPtr<IDXGIAdapter> current_adapter;
  while (SUCCEEDED(dxgi_factory->EnumAdapters(static_cast<UINT>(adapter_info.adapter_names.size()),
                                              current_adapter.ReleaseAndGetAddressOf())))
  {
    DXGI_ADAPTER_DESC adapter_desc;
    std::string adapter_name;
    if (SUCCEEDED(current_adapter->GetDesc(&adapter_desc)))
    {
      char adapter_name_buffer[128];
      const int name_length = WideCharToMultiByte(CP_UTF8, 0, adapter_desc.Description,
                                                  static_cast<int>(std::wcslen(adapter_desc.Description)),
                                                  adapter_name_buffer, countof(adapter_name_buffer), 0, nullptr);
      if (name_length >= 0)
        adapter_name.assign(adapter_name_buffer, static_cast<size_t>(name_length));
      else
        adapter_name.assign("(Unknown)");
    }
    else
    {
      adapter_name.assign("(Unknown)");
    }

    if (adapter_info.fullscreen_modes.empty())
    {
      ComPtr<IDXGIOutput> output;
      if (SUCCEEDED(current_adapter->EnumOutputs(0, &output)))
      {
        UINT num_modes = 0;
        if (SUCCEEDED(output->GetDisplayModeList(DXGI_FORMAT_R8G8B8A8_UNORM, 0, &num_modes, nullptr)))
        {
          std::vector<DXGI_MODE_DESC> modes(num_modes);
          if (SUCCEEDED(output->GetDisplayModeList(DXGI_FORMAT_R8G8B8A8_UNORM, 0, &num_modes, modes.data())))
          {
            for (const DXGI_MODE_DESC& mode : modes)
            {
              adapter_info.fullscreen_modes.push_back(CommonHostInterface::GetFullscreenModeString(
                mode.Width, mode.Height,
                static_cast<float>(mode.RefreshRate.Numerator) / static_cast<float>(mode.RefreshRate.Denominator)));
            }
          }
        }
      }
    }

    // handle duplicate adapter names
    if (std::any_of(adapter_info.adapter_names.begin(), adapter_info.adapter_names.end(),
                    [&adapter_name](const std::string& other) { return (adapter_name == other); }))
    {
      std::string original_adapter_name = std::move(adapter_name);

      u32 current_extra = 2;
      do
      {
        adapter_name = StringUtil::StdStringFromFormat("%s (%u)", original_adapter_name.c_str(), current_extra);
        current_extra++;
      } while (std::any_of(adapter_info.adapter_names.begin(), adapter_info.adapter_names.end(),
                           [&adapter_name](const std::string& other) { return (adapter_name == other); }));
    }

    adapter_info.adapter_names.push_back(std::move(adapter_name));
  }

  return adapter_info;
}

HostDisplay::AdapterAndModeList D3D11HostDisplay::GetAdapterAndModeList()
{
  return GetAdapterAndModeList(m_dxgi_factory.Get());
}

bool D3D11HostDisplay::SetPostProcessingChain(const std::string_view& config)
{
  if (config.empty())
  {
    m_post_processing_input_texture.Destroy();
    m_post_processing_stages.clear();
    m_post_processing_chain.ClearStages();
    return true;
  }

  if (!m_post_processing_chain.CreateFromString(config))
    return false;

  m_post_processing_stages.clear();

  D3D11::ShaderCache shader_cache;
  shader_cache.Open(g_host_interface->GetShaderCacheBasePath(), m_device->GetFeatureLevel(), SHADER_CACHE_VERSION,
                    g_settings.gpu_use_debug_device);

  FrontendCommon::PostProcessingShaderGen shadergen(HostDisplay::RenderAPI::D3D11, true);
  u32 max_ubo_size = 0;

  for (u32 i = 0; i < m_post_processing_chain.GetStageCount(); i++)
  {
    const PostProcessingShader& shader = m_post_processing_chain.GetShaderStage(i);
    const std::string vs = shadergen.GeneratePostProcessingVertexShader(shader);
    const std::string ps = shadergen.GeneratePostProcessingFragmentShader(shader);

    PostProcessingStage stage;
    stage.uniforms_size = shader.GetUniformsSize();
    stage.vertex_shader = shader_cache.GetVertexShader(m_device.Get(), vs);
    stage.pixel_shader = shader_cache.GetPixelShader(m_device.Get(), ps);
    if (!stage.vertex_shader || !stage.pixel_shader)
    {
      Log_ErrorPrintf("Failed to compile one or more post-processing shaders, disabling.");
      m_post_processing_stages.clear();
      m_post_processing_chain.ClearStages();
      return false;
    }

    max_ubo_size = std::max(max_ubo_size, stage.uniforms_size);
    m_post_processing_stages.push_back(std::move(stage));
  }

  if (m_display_uniform_buffer.GetSize() < max_ubo_size &&
      !m_display_uniform_buffer.Create(m_device.Get(), D3D11_BIND_CONSTANT_BUFFER, max_ubo_size))
  {
    Log_ErrorPrintf("Failed to allocate %u byte constant buffer for postprocessing", max_ubo_size);
    m_post_processing_stages.clear();
    m_post_processing_chain.ClearStages();
    return false;
  }

  return true;
}

bool D3D11HostDisplay::CheckPostProcessingRenderTargets(u32 target_width, u32 target_height)
{
  DebugAssert(!m_post_processing_stages.empty());

  const DXGI_FORMAT format = DXGI_FORMAT_R8G8B8A8_UNORM;
  const u32 bind_flags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;

  if (m_post_processing_input_texture.GetWidth() != target_width ||
      m_post_processing_input_texture.GetHeight() != target_height)
  {
    if (!m_post_processing_input_texture.Create(m_device.Get(), target_width, target_height, 1, 1, format, bind_flags))
      return false;
  }

  const u32 target_count = (static_cast<u32>(m_post_processing_stages.size()) - 1);
  for (u32 i = 0; i < target_count; i++)
  {
    PostProcessingStage& pps = m_post_processing_stages[i];
    if (pps.output_texture.GetWidth() != target_width || pps.output_texture.GetHeight() != target_height)
    {
      if (!pps.output_texture.Create(m_device.Get(), target_width, target_height, 1, 1, format, bind_flags))
        return false;
    }
  }

  return true;
}

void D3D11HostDisplay::ApplyPostProcessingChain(ID3D11RenderTargetView* final_target, s32 final_left, s32 final_top,
                                                s32 final_width, s32 final_height, void* texture_handle,
                                                u32 texture_width, s32 texture_height, s32 texture_view_x,
                                                s32 texture_view_y, s32 texture_view_width, s32 texture_view_height,
                                                u32 target_width, u32 target_height)
{
  static constexpr std::array<float, 4> clear_color = {0.0f, 0.0f, 0.0f, 1.0f};

  if (!CheckPostProcessingRenderTargets(target_width, target_height))
  {
    RenderDisplay(final_left, final_top, final_width, final_height, texture_handle, texture_width, texture_height,
                  texture_view_x, texture_view_y, texture_view_width, texture_view_height, m_display_linear_filtering);
    return;
  }

  // downsample/upsample - use same viewport for remainder
  m_context->ClearRenderTargetView(m_post_processing_input_texture.GetD3DRTV(), clear_color.data());
  m_context->OMSetRenderTargets(1, m_post_processing_input_texture.GetD3DRTVArray(), nullptr);
  RenderDisplay(final_left, final_top, final_width, final_height, texture_handle, texture_width, texture_height,
                texture_view_x, texture_view_y, texture_view_width, texture_view_height, m_display_linear_filtering);

  texture_handle = m_post_processing_input_texture.GetD3DSRV();
  texture_width = m_post_processing_input_texture.GetWidth();
  texture_height = m_post_processing_input_texture.GetHeight();
  texture_view_x = final_left;
  texture_view_y = final_top;
  texture_view_width = final_width;
  texture_view_height = final_height;

  const u32 final_stage = static_cast<u32>(m_post_processing_stages.size()) - 1u;
  for (u32 i = 0; i < static_cast<u32>(m_post_processing_stages.size()); i++)
  {
    PostProcessingStage& pps = m_post_processing_stages[i];
    if (i == final_stage)
    {
      m_context->OMSetRenderTargets(1, &final_target, nullptr);
    }
    else
    {
      m_context->ClearRenderTargetView(pps.output_texture.GetD3DRTV(), clear_color.data());
      m_context->OMSetRenderTargets(1, pps.output_texture.GetD3DRTVArray(), nullptr);
    }

    m_context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    m_context->VSSetShader(pps.vertex_shader.Get(), nullptr, 0);
    m_context->PSSetShader(pps.pixel_shader.Get(), nullptr, 0);
    m_context->PSSetShaderResources(0, 1, reinterpret_cast<ID3D11ShaderResourceView**>(&texture_handle));
    m_context->PSSetSamplers(0, 1, m_point_sampler.GetAddressOf());

    const auto map =
      m_display_uniform_buffer.Map(m_context.Get(), m_display_uniform_buffer.GetSize(), pps.uniforms_size);
    m_post_processing_chain.GetShaderStage(i).FillUniformBuffer(
      map.pointer, texture_width, texture_height, texture_view_x, texture_view_y, texture_view_width,
      texture_view_height, GetWindowWidth(), GetWindowHeight(), 0.0f);
    m_display_uniform_buffer.Unmap(m_context.Get(), pps.uniforms_size);
    m_context->VSSetConstantBuffers(0, 1, m_display_uniform_buffer.GetD3DBufferArray());
    m_context->PSSetConstantBuffers(0, 1, m_display_uniform_buffer.GetD3DBufferArray());

    m_context->Draw(3, 0);

    if (i != final_stage)
      texture_handle = pps.output_texture.GetD3DSRV();
  }

  ID3D11ShaderResourceView* null_srv = nullptr;
  m_context->PSSetShaderResources(0, 1, &null_srv);
}

} // namespace FrontendCommon
