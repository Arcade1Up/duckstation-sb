#include "host_interface.h"
#include "bios.h"
#include "cdrom.h"
#include "common/audio_stream.h"
#include "common/byte_stream.h"
#include "common/file_system.h"
#include "common/image.h"
#include "common/log.h"
#include "common/string_util.h"
#include "controller.h"
#include "cpu_code_cache.h"
#include "cpu_core.h"
#include "dma.h"
#include "gpu.h"
#include "gte.h"
#include "host_display.h"
#include "pgxp.h"
#include "save_state_version.h"
#include "spu.h"
#include "system.h"
#include "texture_replacements.h"
#include <cmath>
#include <cstring>
#include <cwchar>
#include <stdlib.h>
Log_SetChannel(HostInterface);

HostInterface* g_host_interface;

HostInterface::HostInterface()
{
  Assert(!g_host_interface);
  g_host_interface = this;

  // we can get the program directory at construction time
  m_program_directory = FileSystem::GetPathDirectory(FileSystem::GetProgramPath());
}

HostInterface::~HostInterface()
{
  // system should be shut down prior to the destructor
  Assert(System::IsShutdown() && !m_audio_stream && !m_display);
  Assert(g_host_interface == this);
  g_host_interface = nullptr;
}

bool HostInterface::Initialize()
{
  return true;
}

void HostInterface::Shutdown()
{
  if (!System::IsShutdown())
    System::Shutdown();
}

void HostInterface::CreateAudioStream()
{
  Log_InfoPrintf("Creating '%s' audio stream, sample rate = %u, channels = %u, buffer size = %u",
                 Settings::GetAudioBackendName(g_settings.audio_backend), AUDIO_SAMPLE_RATE, AUDIO_CHANNELS,
                 g_settings.audio_buffer_size);

  m_audio_stream = CreateAudioStream(g_settings.audio_backend);

  if (!m_audio_stream ||
      !m_audio_stream->Reconfigure(AUDIO_SAMPLE_RATE, AUDIO_SAMPLE_RATE, AUDIO_CHANNELS, g_settings.audio_buffer_size))
  {
    ReportFormattedError("Failed to create or configure audio stream, falling back to null output.");
    m_audio_stream.reset();
    m_audio_stream = AudioStream::CreateNullAudioStream();
    m_audio_stream->Reconfigure(AUDIO_SAMPLE_RATE, AUDIO_SAMPLE_RATE, AUDIO_CHANNELS, g_settings.audio_buffer_size);
  }

  m_audio_stream->SetOutputVolume(GetAudioOutputVolume());

  if (System::IsValid())
    g_spu.SetAudioStream(m_audio_stream.get());
}

s32 HostInterface::GetAudioOutputVolume() const
{
  return g_settings.audio_output_muted ? 0 : g_settings.audio_output_volume;
}

bool HostInterface::BootSystem(std::shared_ptr<SystemBootParameters> parameters)
{
  if (!parameters->state_stream)
  {
    if (parameters->filename.empty())
      Log_InfoPrintf("Boot Filename: <BIOS/Shell>");
    else
      Log_InfoPrintf("Boot Filename: %s", parameters->filename.c_str());
  }

  if (!AcquireHostDisplay())
  {
    ReportError(g_host_interface->TranslateString("System", "Failed to acquire host display."));
    OnSystemDestroyed();
    return false;
  }

  // set host display settings
  m_display->SetDisplayLinearFiltering(g_settings.display_linear_filtering);
  m_display->SetDisplayIntegerScaling(g_settings.display_integer_scaling);
  m_display->SetDisplayStretch(g_settings.display_stretch);

  // create the audio stream. this will never fail, since we'll just fall back to null
  CreateAudioStream();

  if (!System::Boot(*parameters))
  {
    if (!System::IsStartupCancelled())
    {
      ReportError(
        g_host_interface->TranslateString("System", "System failed to boot. The log may contain more information."));
    }

    OnSystemDestroyed();
    m_audio_stream.reset();
    ReleaseHostDisplay();
    return false;
  }

  UpdateSoftwareCursor();
  OnSystemCreated();

  m_audio_stream->PauseOutput(false);
  return true;
}

void HostInterface::ResetSystem()
{
  System::Reset();
  System::ResetPerformanceCounters();
  System::ResetThrottler();
  AddOSDMessage(TranslateStdString("OSDMessage", "System reset."));
}

void HostInterface::PauseSystem(bool paused)
{
  if (paused == System::IsPaused() || System::IsShutdown())
    return;

  System::SetState(paused ? System::State::Paused : System::State::Running);
  if (!paused)
    m_audio_stream->EmptyBuffers();
  m_audio_stream->PauseOutput(paused);

  OnSystemPaused(paused);

  if (!paused)
  {
    System::ResetPerformanceCounters();
    System::ResetThrottler();
  }
}

void HostInterface::DestroySystem()
{
  if (System::IsShutdown())
    return;

  System::Shutdown();
  m_audio_stream.reset();
  UpdateSoftwareCursor();
  ReleaseHostDisplay();
  OnSystemDestroyed();
}

void HostInterface::ReportError(const char* message)
{
  Log_ErrorPrint(message);
}

void HostInterface::ReportMessage(const char* message)
{
  Log_InfoPrint(message);
}

void HostInterface::ReportDebuggerMessage(const char* message)
{
  Log_InfoPrintf("(Debugger) %s", message);
}

bool HostInterface::ConfirmMessage(const char* message)
{
  Log_WarningPrintf("ConfirmMessage(\"%s\") -> Yes", message);
  return true;
}

void HostInterface::ReportFormattedError(const char* format, ...)
{
  std::va_list ap;
  va_start(ap, format);
  std::string message = StringUtil::StdStringFromFormatV(format, ap);
  va_end(ap);

  ReportError(message.c_str());
}

void HostInterface::ReportFormattedMessage(const char* format, ...)
{
  std::va_list ap;
  va_start(ap, format);
  std::string message = StringUtil::StdStringFromFormatV(format, ap);
  va_end(ap);

  ReportMessage(message.c_str());
}

void HostInterface::ReportFormattedDebuggerMessage(const char* format, ...)
{
  std::va_list ap;
  va_start(ap, format);
  std::string message = StringUtil::StdStringFromFormatV(format, ap);
  va_end(ap);

  ReportDebuggerMessage(message.c_str());
}

bool HostInterface::ConfirmFormattedMessage(const char* format, ...)
{
  std::va_list ap;
  va_start(ap, format);
  std::string message = StringUtil::StdStringFromFormatV(format, ap);
  va_end(ap);

  return ConfirmMessage(message.c_str());
}

void HostInterface::AddOSDMessage(std::string message, float duration /* = 2.0f */)
{
  Log_InfoPrintf("OSD: %s", message.c_str());
}

void HostInterface::AddFormattedOSDMessage(float duration, const char* format, ...)
{
  std::va_list ap;
  va_start(ap, format);
  std::string message = StringUtil::StdStringFromFormatV(format, ap);
  va_end(ap);

  AddOSDMessage(std::move(message), duration);
}

std::string HostInterface::GetBIOSDirectory()
{
  std::string dir = GetStringSettingValue("BIOS", "SearchDirectory", "");
  if (!dir.empty())
    return dir;

  return GetUserDirectoryRelativePath("bios");
}

std::optional<std::vector<u8>> HostInterface::GetBIOSImage(ConsoleRegion region)
{
  std::string bios_dir = GetBIOSDirectory();
  std::string bios_name;
  switch (region)
  {
    case ConsoleRegion::NTSC_J:
      bios_name = GetStringSettingValue("BIOS", "PathNTSCJ", "");
      break;

    case ConsoleRegion::PAL:
      bios_name = GetStringSettingValue("BIOS", "PathPAL", "");
      break;

    case ConsoleRegion::NTSC_U:
    default:
      bios_name = GetStringSettingValue("BIOS", "PathNTSCU", "");
      break;
  }

  if (bios_name.empty())
  {
    // auto-detect
    return FindBIOSImageInDirectory(region, bios_dir.c_str());
  }

  // try the configured path
  std::optional<BIOS::Image> image = BIOS::LoadImageFromFile(
    StringUtil::StdStringFromFormat("%s" FS_OSPATH_SEPARATOR_STR "%s", bios_dir.c_str(), bios_name.c_str()).c_str());
  if (!image.has_value())
  {
    g_host_interface->ReportFormattedError(
      g_host_interface->TranslateString("HostInterface", "Failed to load configured BIOS file '%s'"),
      bios_name.c_str());
    return std::nullopt;
  }

  BIOS::Hash found_hash = BIOS::GetHash(*image);
  Log_DevPrintf("Hash for BIOS '%s': %s", bios_name.c_str(), found_hash.ToString().c_str());

  if (!BIOS::IsValidHashForRegion(region, found_hash))
    Log_WarningPrintf("Hash for BIOS '%s' does not match region. This may cause issues.", bios_name.c_str());

  return image;
}

std::optional<std::vector<u8>> HostInterface::FindBIOSImageInDirectory(ConsoleRegion region, const char* directory)
{
  Log_InfoPrintf("Searching for a %s BIOS in '%s'...", Settings::GetConsoleRegionDisplayName(region), directory);

  FileSystem::FindResultsArray results;
  FileSystem::FindFiles(
    directory, "*", FILESYSTEM_FIND_FILES | FILESYSTEM_FIND_HIDDEN_FILES | FILESYSTEM_FIND_RELATIVE_PATHS, &results);

  std::string fallback_path;
  std::optional<BIOS::Image> fallback_image;
  const BIOS::ImageInfo* fallback_info = nullptr;

  for (const FILESYSTEM_FIND_DATA& fd : results)
  {
    if (fd.Size != BIOS::BIOS_SIZE && fd.Size != BIOS::BIOS_SIZE_PS2 && fd.Size != BIOS::BIOS_SIZE_PS3)
    {
      Log_WarningPrintf("Skipping '%s': incorrect size", fd.FileName.c_str());
      continue;
    }

    std::string full_path(
      StringUtil::StdStringFromFormat("%s" FS_OSPATH_SEPARATOR_STR "%s", directory, fd.FileName.c_str()));

    std::optional<BIOS::Image> found_image = BIOS::LoadImageFromFile(full_path.c_str());
    if (!found_image)
      continue;

    BIOS::Hash found_hash = BIOS::GetHash(*found_image);
    Log_DevPrintf("Hash for BIOS '%s': %s", fd.FileName.c_str(), found_hash.ToString().c_str());

    const BIOS::ImageInfo* ii = BIOS::GetImageInfoForHash(found_hash);

    if (BIOS::IsValidHashForRegion(region, found_hash))
    {
      Log_InfoPrintf("Using BIOS '%s': %s", fd.FileName.c_str(), ii ? ii->description : "");
      return found_image;
    }

    // don't let an unknown bios take precedence over a known one
    if (!fallback_path.empty() && (fallback_info || !ii))
      continue;

    fallback_path = std::move(full_path);
    fallback_image = std::move(found_image);
    fallback_info = ii;
  }

  if (!fallback_image.has_value())
  {
    g_host_interface->ReportFormattedError(
      g_host_interface->TranslateString("HostInterface", "No BIOS image found for %s region"),
      Settings::GetConsoleRegionDisplayName(region));
    return std::nullopt;
  }

  if (!fallback_info)
  {
    Log_WarningPrintf("Using unknown BIOS '%s'. This may crash.", fallback_path.c_str());
  }
  else
  {
    Log_WarningPrintf("Falling back to possibly-incompatible image '%s': %s", fallback_path.c_str(),
                      fallback_info->description);
  }

  return fallback_image;
}

std::vector<std::pair<std::string, const BIOS::ImageInfo*>>
HostInterface::FindBIOSImagesInDirectory(const char* directory)
{
  std::vector<std::pair<std::string, const BIOS::ImageInfo*>> results;

  FileSystem::FindResultsArray files;
  FileSystem::FindFiles(directory, "*",
                        FILESYSTEM_FIND_FILES | FILESYSTEM_FIND_HIDDEN_FILES | FILESYSTEM_FIND_RELATIVE_PATHS, &files);

  for (FILESYSTEM_FIND_DATA& fd : files)
  {
    if (fd.Size != BIOS::BIOS_SIZE && fd.Size != BIOS::BIOS_SIZE_PS2 && fd.Size != BIOS::BIOS_SIZE_PS3)
      continue;

    std::string full_path(
      StringUtil::StdStringFromFormat("%s" FS_OSPATH_SEPARATOR_STR "%s", directory, fd.FileName.c_str()));

    std::optional<BIOS::Image> found_image = BIOS::LoadImageFromFile(full_path.c_str());
    if (!found_image)
      continue;

    BIOS::Hash found_hash = BIOS::GetHash(*found_image);
    const BIOS::ImageInfo* ii = BIOS::GetImageInfoForHash(found_hash);
    results.emplace_back(std::move(fd.FileName), ii);
  }

  return results;
}

bool HostInterface::HasAnyBIOSImages()
{
  const std::string dir = GetBIOSDirectory();
  return (FindBIOSImageInDirectory(ConsoleRegion::Auto, dir.c_str()).has_value());
}

bool HostInterface::LoadState(const char* filename)
{
  std::unique_ptr<ByteStream> stream = FileSystem::OpenFile(filename, BYTESTREAM_OPEN_READ | BYTESTREAM_OPEN_STREAMED);
  if (!stream)
    return false;

  AddFormattedOSDMessage(5.0f, TranslateString("OSDMessage", "Loading state from '%s'..."), filename);

  if (!System::IsShutdown())
  {
    if (!System::LoadState(stream.get()))
    {
      ReportFormattedError(TranslateString("OSDMessage", "Loading state from '%s' failed. Resetting."), filename);
      ResetSystem();
      return false;
    }
  }
  else
  {
    auto boot_params = std::make_shared<SystemBootParameters>();
    boot_params->state_stream = std::move(stream);
    if (!BootSystem(std::move(boot_params)))
      return false;
  }

  System::ResetPerformanceCounters();
  System::ResetThrottler();
  OnDisplayInvalidated();
  return true;
}

bool HostInterface::SaveState(const char* filename)
{
  std::unique_ptr<ByteStream> stream =
    FileSystem::OpenFile(filename, BYTESTREAM_OPEN_CREATE | BYTESTREAM_OPEN_WRITE | BYTESTREAM_OPEN_TRUNCATE |
                                     BYTESTREAM_OPEN_ATOMIC_UPDATE | BYTESTREAM_OPEN_STREAMED);
  if (!stream)
    return false;

  const bool result = System::SaveState(stream.get());
  if (!result)
  {
    ReportFormattedError(TranslateString("OSDMessage", "Saving state to '%s' failed."), filename);
    stream->Discard();
  }
  else
  {
    AddFormattedOSDMessage(5.0f, TranslateString("OSDMessage", "State saved to '%s'."), filename);
    stream->Commit();
  }

  return result;
}

void HostInterface::OnSystemCreated() {}

void HostInterface::OnSystemPaused(bool paused) {}

void HostInterface::OnSystemDestroyed() {}

void HostInterface::OnSystemPerformanceCountersUpdated() {}

void HostInterface::OnDisplayInvalidated() {}

void HostInterface::OnSystemStateSaved(bool global, s32 slot) {}

void HostInterface::OnRunningGameChanged(const std::string& path, CDImage* image, const std::string& game_code,
                                         const std::string& game_title)
{
}

void HostInterface::OnControllerTypeChanged(u32 slot) {}

std::string HostInterface::GetShaderCacheBasePath() const
{
  return GetUserDirectoryRelativePath("cache/");
}

void HostInterface::SetDefaultSettings(SettingsInterface& si)
{
  si.SetStringValue("Console", "Region", Settings::GetConsoleRegionName(Settings::DEFAULT_CONSOLE_REGION));
  si.SetBoolValue("Console", "Enable8MBRAM", false);

  si.SetFloatValue("Main", "EmulationSpeed", 1.0f);
  si.SetFloatValue("Main", "FastForwardSpeed", 0.0f);
  si.SetFloatValue("Main", "TurboSpeed", 0.0f);
  si.SetBoolValue("Main", "SyncToHostRefreshRate", false);
  si.SetBoolValue("Main", "IncreaseTimerResolution", true);
  si.SetBoolValue("Main", "InhibitScreensaver", true);
  si.SetBoolValue("Main", "StartPaused", false);
  si.SetBoolValue("Main", "StartFullscreen", false);
  si.SetBoolValue("Main", "PauseOnFocusLoss", false);
  si.SetBoolValue("Main", "PauseOnMenu", true);
  si.SetBoolValue("Main", "SaveStateOnExit", true);
  si.SetBoolValue("Main", "ConfirmPowerOff", true);
  si.SetBoolValue("Main", "LoadDevicesFromSaveStates", false);
  si.SetBoolValue("Main", "ApplyGameSettings", true);
  si.SetBoolValue("Main", "AutoLoadCheats", true);
  si.SetBoolValue("Main", "DisableAllEnhancements", false);
  si.SetBoolValue("Main", "RewindEnable", false);
  si.SetFloatValue("Main", "RewindFrequency", 10.0f);
  si.SetIntValue("Main", "RewindSaveSlots", 10);
  si.SetFloatValue("Main", "RunaheadFrameCount", 0);

  si.SetStringValue("CPU", "ExecutionMode", Settings::GetCPUExecutionModeName(Settings::DEFAULT_CPU_EXECUTION_MODE));
  si.SetIntValue("CPU", "OverclockNumerator", 1);
  si.SetIntValue("CPU", "OverclockDenominator", 1);
  si.SetBoolValue("CPU", "OverclockEnable", false);
  si.SetBoolValue("CPU", "RecompilerMemoryExceptions", false);
  si.SetBoolValue("CPU", "RecompilerBlockLinking", true);
  si.SetBoolValue("CPU", "ICache", false);
  si.SetBoolValue("CPU", "FastmemMode", Settings::GetCPUFastmemModeName(Settings::DEFAULT_CPU_FASTMEM_MODE));

  si.SetStringValue("GPU", "Renderer", Settings::GetRendererName(Settings::DEFAULT_GPU_RENDERER));
  si.SetIntValue("GPU", "ResolutionScale", 1);
  si.SetIntValue("GPU", "Multisamples", 1);
  si.SetBoolValue("GPU", "UseDebugDevice", false);
  si.SetBoolValue("GPU", "UseSoftwareRendererForReadbacks", false);
  si.SetBoolValue("GPU", "PerSampleShading", false);
  si.SetBoolValue("GPU", "UseThread", true);
  si.SetBoolValue("GPU", "ThreadedPresentation", true);
  si.SetBoolValue("GPU", "TrueColor", false);
  si.SetBoolValue("GPU", "ScaledDithering", true);
  si.SetStringValue("GPU", "TextureFilter", Settings::GetTextureFilterName(Settings::DEFAULT_GPU_TEXTURE_FILTER));
  si.SetStringValue("GPU", "DownsampleMode", Settings::GetDownsampleModeName(Settings::DEFAULT_GPU_DOWNSAMPLE_MODE));
  si.SetBoolValue("GPU", "DisableInterlacing", true);
  si.SetBoolValue("GPU", "ForceNTSCTimings", false);
  si.SetBoolValue("GPU", "WidescreenHack", false);
  si.SetBoolValue("GPU", "ChromaSmoothing24Bit", false);
  si.SetBoolValue("GPU", "PGXPEnable", false);
  si.SetBoolValue("GPU", "PGXPCulling", true);
  si.SetBoolValue("GPU", "PGXPTextureCorrection", true);
  si.SetBoolValue("GPU", "PGXPVertexCache", false);
  si.SetBoolValue("GPU", "PGXPCPU", false);
  si.SetBoolValue("GPU", "PGXPPreserveProjFP", false);
  si.SetFloatValue("GPU", "PGXPTolerance", -1.0f);
  si.SetBoolValue("GPU", "PGXPDepthBuffer", false);
  si.SetFloatValue("GPU", "PGXPDepthClearThreshold", Settings::DEFAULT_GPU_PGXP_DEPTH_THRESHOLD);

  si.SetStringValue("Display", "CropMode", Settings::GetDisplayCropModeName(Settings::DEFAULT_DISPLAY_CROP_MODE));
  si.SetIntValue("Display", "ActiveStartOffset", 0);
  si.SetIntValue("Display", "ActiveEndOffset", 0);
  si.SetIntValue("Display", "LineStartOffset", 0);
  si.SetIntValue("Display", "LineEndOffset", 0);
  si.SetStringValue("Display", "AspectRatio",
                    Settings::GetDisplayAspectRatioName(Settings::DEFAULT_DISPLAY_ASPECT_RATIO));
  si.SetIntValue("Display", "CustomAspectRatioNumerator", 4);
  si.GetIntValue("Display", "CustomAspectRatioDenominator", 3);
  si.SetBoolValue("Display", "Force4_3For24Bit", false);
  si.SetBoolValue("Display", "LinearFiltering", true);
  si.SetBoolValue("Display", "IntegerScaling", false);
  si.SetBoolValue("Display", "Stretch", false);
  si.SetBoolValue("Display", "PostProcessing", false);
  si.SetBoolValue("Display", "ShowOSDMessages", true);
  si.SetBoolValue("Display", "ShowFPS", false);
  si.SetBoolValue("Display", "ShowVPS", false);
  si.SetBoolValue("Display", "ShowSpeed", false);
  si.SetBoolValue("Display", "ShowResolution", false);
  si.SetBoolValue("Display", "ShowStatusIndicators", true);
  si.SetBoolValue("Display", "ShowEnhancements", false);
  si.SetBoolValue("Display", "Fullscreen", false);
  si.SetBoolValue("Display", "VSync", Settings::DEFAULT_VSYNC_VALUE);
  si.SetBoolValue("Display", "DisplayAllFrames", false);
  si.SetStringValue("Display", "PostProcessChain", "");
  si.SetFloatValue("Display", "MaxFPS", Settings::DEFAULT_DISPLAY_MAX_FPS);

  si.SetIntValue("CDROM", "ReadaheadSectors", Settings::DEFAULT_CDROM_READAHEAD_SECTORS);
  si.SetBoolValue("CDROM", "RegionCheck", false);
  si.SetBoolValue("CDROM", "LoadImageToRAM", false);
  si.SetBoolValue("CDROM", "MuteCDAudio", false);
  si.SetIntValue("CDROM", "ReadSpeedup", 1);
  si.SetIntValue("CDROM", "SeekSpeedup", 1);

  si.SetStringValue("Audio", "Backend", Settings::GetAudioBackendName(Settings::DEFAULT_AUDIO_BACKEND));
  si.SetIntValue("Audio", "OutputVolume", 100);
  si.SetIntValue("Audio", "FastForwardVolume", 100);
  si.SetIntValue("Audio", "BufferSize", DEFAULT_AUDIO_BUFFER_SIZE);
  si.SetBoolValue("Audio", "Resampling", true);
  si.SetIntValue("Audio", "OutputMuted", false);
  si.SetBoolValue("Audio", "Sync", true);
  si.SetBoolValue("Audio", "DumpOnBoot", false);

  si.SetStringValue("BIOS", "SearchDirectory", "");
  si.SetStringValue("BIOS", "PathNTSCU", "");
  si.SetStringValue("BIOS", "PathNTSCJ", "");
  si.SetStringValue("BIOS", "PathPAL", "");
  si.SetBoolValue("BIOS", "PatchTTYEnable", false);
  si.SetBoolValue("BIOS", "PatchFastBoot", Settings::DEFAULT_FAST_BOOT_VALUE);

  si.SetStringValue("Controller1", "Type", Settings::GetControllerTypeName(Settings::DEFAULT_CONTROLLER_1_TYPE));

  for (u32 i = 1; i < NUM_CONTROLLER_AND_CARD_PORTS; i++)
  {
    si.SetStringValue(TinyString::FromFormat("Controller%u", i + 1u), "Type",
                      Settings::GetControllerTypeName(Settings::DEFAULT_CONTROLLER_2_TYPE));
  }

  si.SetStringValue("MemoryCards", "Card1Type", Settings::GetMemoryCardTypeName(Settings::DEFAULT_MEMORY_CARD_1_TYPE));
  si.SetStringValue("MemoryCards", "Card2Type", Settings::GetMemoryCardTypeName(Settings::DEFAULT_MEMORY_CARD_2_TYPE));
  si.DeleteValue("MemoryCards", "Card1Path");
  si.DeleteValue("MemoryCards", "Card2Path");
  si.DeleteValue("MemoryCards", "Directory");
  si.SetBoolValue("MemoryCards", "UsePlaylistTitle", true);

  si.SetStringValue("ControllerPorts", "MultitapMode", Settings::GetMultitapModeName(Settings::DEFAULT_MULTITAP_MODE));

  si.SetStringValue("Logging", "LogLevel", Settings::GetLogLevelName(Settings::DEFAULT_LOG_LEVEL));
  si.SetStringValue("Logging", "LogFilter", "");
  si.SetBoolValue("Logging", "LogToConsole", Settings::DEFAULT_LOG_TO_CONSOLE);
  si.SetBoolValue("Logging", "LogToDebug", false);
  si.SetBoolValue("Logging", "LogToWindow", false);
  si.SetBoolValue("Logging", "LogToFile", false);

  si.SetBoolValue("Debug", "ShowVRAM", false);
  si.SetBoolValue("Debug", "DumpCPUToVRAMCopies", false);
  si.SetBoolValue("Debug", "DumpVRAMToCPUCopies", false);
  si.SetBoolValue("Debug", "ShowGPUState", false);
  si.SetBoolValue("Debug", "ShowCDROMState", false);
  si.SetBoolValue("Debug", "ShowSPUState", false);
  si.SetBoolValue("Debug", "ShowTimersState", false);
  si.SetBoolValue("Debug", "ShowMDECState", false);
  si.SetBoolValue("Debug", "ShowDMAState", false);

  si.SetIntValue("Hacks", "DMAMaxSliceTicks", static_cast<int>(Settings::DEFAULT_DMA_MAX_SLICE_TICKS));
  si.SetIntValue("Hacks", "DMAHaltTicks", static_cast<int>(Settings::DEFAULT_DMA_HALT_TICKS));
  si.SetIntValue("Hacks", "GPUFIFOSize", static_cast<int>(Settings::DEFAULT_GPU_FIFO_SIZE));
  si.SetIntValue("Hacks", "GPUMaxRunAhead", static_cast<int>(Settings::DEFAULT_GPU_MAX_RUN_AHEAD));
}

void HostInterface::LoadSettings(SettingsInterface& si)
{
  g_settings.Load(si);
}

void HostInterface::FixIncompatibleSettings(bool display_osd_messages)
{
  if (g_settings.disable_all_enhancements)
  {
    Log_WarningPrintf("All enhancements disabled by config setting.");
    g_settings.cpu_overclock_enable = false;
    g_settings.cpu_overclock_active = false;
    g_settings.enable_8mb_ram = false;
    g_settings.gpu_resolution_scale = 1;
    g_settings.gpu_multisamples = 1;
    g_settings.gpu_per_sample_shading = false;
    g_settings.gpu_true_color = false;
    g_settings.gpu_scaled_dithering = false;
    g_settings.gpu_texture_filter = GPUTextureFilter::Nearest;
    g_settings.gpu_disable_interlacing = false;
    g_settings.gpu_force_ntsc_timings = false;
    g_settings.gpu_widescreen_hack = false;
    g_settings.gpu_pgxp_enable = false;
    g_settings.gpu_24bit_chroma_smoothing = false;
    g_settings.cdrom_read_speedup = 1;
    g_settings.cdrom_seek_speedup = 1;
    g_settings.cdrom_mute_cd_audio = false;
    g_settings.texture_replacements.enable_vram_write_replacements = false;
    g_settings.bios_patch_fast_boot = false;
    g_settings.bios_patch_tty_enable = false;
  }

  if (g_settings.display_integer_scaling && g_settings.display_linear_filtering)
  {
    Log_WarningPrintf("Disabling linear filter due to integer upscaling.");
    g_settings.display_linear_filtering = false;
  }

  if (g_settings.display_integer_scaling && g_settings.display_stretch)
  {
    Log_WarningPrintf("Disabling stretch due to integer upscaling.");
    g_settings.display_stretch = false;
  }

  if (g_settings.gpu_pgxp_enable)
  {
    if (g_settings.gpu_renderer == GPURenderer::Software)
    {
      if (display_osd_messages)
      {
        AddOSDMessage(
          TranslateStdString("OSDMessage", "PGXP is incompatible with the software renderer, disabling PGXP."), 10.0f);
      }
      g_settings.gpu_pgxp_enable = false;
    }
  }

#ifndef WITH_MMAP_FASTMEM
  if (g_settings.cpu_fastmem_mode == CPUFastmemMode::MMap)
  {
    Log_WarningPrintf("mmap fastmem is not available on this platform, using LUT instead.");
    g_settings.cpu_fastmem_mode = CPUFastmemMode::LUT;
  }
#endif

#if defined(__ANDROID__) && defined(__arm__) && !defined(__aarch64__) && !defined(_M_ARM64)
  if (g_settings.rewind_enable)
  {
    AddOSDMessage(TranslateStdString("OSDMessage", "Rewind is not supported on 32-bit ARM for Android."), 30.0f);
    g_settings.rewind_enable = false;
  }
#endif

  // rewinding causes issues with mmap fastmem, so just use LUT
  if ((g_settings.rewind_enable || g_settings.IsRunaheadEnabled()) && g_settings.IsUsingFastmem() &&
      g_settings.cpu_fastmem_mode == CPUFastmemMode::MMap)
  {
    Log_WarningPrintf("Disabling mmap fastmem due to rewind being enabled");
    g_settings.cpu_fastmem_mode = CPUFastmemMode::LUT;
  }

  // code compilation is too slow with runahead, use the recompiler
  if (g_settings.IsRunaheadEnabled() && g_settings.IsUsingCodeCache())
  {
    Log_WarningPrintf("Code cache/recompiler disabled due to runahead");
    g_settings.cpu_execution_mode = CPUExecutionMode::Interpreter;
  }

  if (g_settings.IsRunaheadEnabled() && g_settings.rewind_enable)
  {
    Log_WarningPrintf("Rewind disabled due to runahead being enabled");
    g_settings.rewind_enable = false;
  }
}

void HostInterface::SaveSettings(SettingsInterface& si)
{
  g_settings.Save(si);
}

void HostInterface::CheckForSettingsChanges(const Settings& old_settings)
{
  if (System::IsValid() && (g_settings.gpu_renderer != old_settings.gpu_renderer ||
                            g_settings.gpu_use_debug_device != old_settings.gpu_use_debug_device ||
                            g_settings.gpu_threaded_presentation != old_settings.gpu_threaded_presentation))
  {
    AddFormattedOSDMessage(5.0f, TranslateString("OSDMessage", "Switching to %s%s GPU renderer."),
                           Settings::GetRendererName(g_settings.gpu_renderer),
                           g_settings.gpu_use_debug_device ? " (debug)" : "");
    RecreateSystem();
  }

  if (System::IsValid())
  {
    System::ClearMemorySaveStates();

    if (g_settings.cpu_overclock_active != old_settings.cpu_overclock_active ||
        (g_settings.cpu_overclock_active &&
         (g_settings.cpu_overclock_numerator != old_settings.cpu_overclock_numerator ||
          g_settings.cpu_overclock_denominator != old_settings.cpu_overclock_denominator)))
    {
      System::UpdateOverclock();
    }

    if (g_settings.audio_backend != old_settings.audio_backend ||
        g_settings.audio_buffer_size != old_settings.audio_buffer_size)
    {
      if (g_settings.audio_backend != old_settings.audio_backend)
      {
        AddFormattedOSDMessage(5.0f, TranslateString("OSDMessage", "Switching to %s audio backend."),
                               Settings::GetAudioBackendName(g_settings.audio_backend));
      }
      DebugAssert(m_audio_stream);
      m_audio_stream.reset();
      CreateAudioStream();
      m_audio_stream->PauseOutput(System::IsPaused());
    }

    if (g_settings.emulation_speed != old_settings.emulation_speed)
      System::UpdateThrottlePeriod();

    if (g_settings.cpu_execution_mode != old_settings.cpu_execution_mode ||
        g_settings.cpu_fastmem_mode != old_settings.cpu_fastmem_mode)
    {
      AddFormattedOSDMessage(
        5.0f, TranslateString("OSDMessage", "Switching to %s CPU execution mode."),
        TranslateString("CPUExecutionMode", Settings::GetCPUExecutionModeDisplayName(g_settings.cpu_execution_mode))
          .GetCharArray());
      CPU::CodeCache::Reinitialize();
      CPU::ClearICache();
    }

    if (g_settings.cpu_execution_mode == CPUExecutionMode::Recompiler &&
        (g_settings.cpu_recompiler_memory_exceptions != old_settings.cpu_recompiler_memory_exceptions ||
         g_settings.cpu_recompiler_block_linking != old_settings.cpu_recompiler_block_linking ||
         g_settings.cpu_recompiler_icache != old_settings.cpu_recompiler_icache))
    {
      AddOSDMessage(TranslateStdString("OSDMessage", "Recompiler options changed, flushing all blocks."), 5.0f);
      CPU::CodeCache::Flush();

      if (g_settings.cpu_recompiler_icache != old_settings.cpu_recompiler_icache)
        CPU::ClearICache();
    }

    m_audio_stream->SetOutputVolume(GetAudioOutputVolume());

    if (g_settings.gpu_resolution_scale != old_settings.gpu_resolution_scale ||
        g_settings.gpu_multisamples != old_settings.gpu_multisamples ||
        g_settings.gpu_per_sample_shading != old_settings.gpu_per_sample_shading ||
        g_settings.gpu_use_thread != old_settings.gpu_use_thread ||
        g_settings.gpu_use_software_renderer_for_readbacks != old_settings.gpu_use_software_renderer_for_readbacks ||
        g_settings.gpu_fifo_size != old_settings.gpu_fifo_size ||
        g_settings.gpu_max_run_ahead != old_settings.gpu_max_run_ahead ||
        g_settings.gpu_true_color != old_settings.gpu_true_color ||
        g_settings.gpu_scaled_dithering != old_settings.gpu_scaled_dithering ||
        g_settings.gpu_texture_filter != old_settings.gpu_texture_filter ||
        g_settings.gpu_disable_interlacing != old_settings.gpu_disable_interlacing ||
        g_settings.gpu_force_ntsc_timings != old_settings.gpu_force_ntsc_timings ||
        g_settings.gpu_24bit_chroma_smoothing != old_settings.gpu_24bit_chroma_smoothing ||
        g_settings.gpu_downsample_mode != old_settings.gpu_downsample_mode ||
        g_settings.display_crop_mode != old_settings.display_crop_mode ||
        g_settings.display_aspect_ratio != old_settings.display_aspect_ratio ||
        g_settings.gpu_pgxp_enable != old_settings.gpu_pgxp_enable ||
        g_settings.gpu_pgxp_depth_buffer != old_settings.gpu_pgxp_depth_buffer ||
        g_settings.display_active_start_offset != old_settings.display_active_start_offset ||
        g_settings.display_active_end_offset != old_settings.display_active_end_offset ||
        g_settings.display_line_start_offset != old_settings.display_line_start_offset ||
        g_settings.display_line_end_offset != old_settings.display_line_end_offset ||
        g_settings.rewind_enable != old_settings.rewind_enable ||
        g_settings.runahead_frames != old_settings.runahead_frames)
    {
      g_gpu->UpdateSettings();
      OnDisplayInvalidated();
    }

    if (g_settings.gpu_widescreen_hack != old_settings.gpu_widescreen_hack ||
        g_settings.display_aspect_ratio != old_settings.display_aspect_ratio ||
        (g_settings.display_aspect_ratio == DisplayAspectRatio::Custom &&
         (g_settings.display_aspect_ratio_custom_numerator != old_settings.display_aspect_ratio_custom_numerator ||
          g_settings.display_aspect_ratio_custom_denominator != old_settings.display_aspect_ratio_custom_denominator)))
    {
      GTE::UpdateAspectRatio();
    }

    if (g_settings.gpu_pgxp_enable != old_settings.gpu_pgxp_enable ||
        (g_settings.gpu_pgxp_enable && (g_settings.gpu_pgxp_culling != old_settings.gpu_pgxp_culling ||
                                        g_settings.gpu_pgxp_vertex_cache != old_settings.gpu_pgxp_vertex_cache ||
                                        g_settings.gpu_pgxp_cpu != old_settings.gpu_pgxp_cpu)))
    {
      if (g_settings.IsUsingCodeCache())
      {
        AddOSDMessage(g_settings.gpu_pgxp_enable ?
                        TranslateStdString("OSDMessage", "PGXP enabled, recompiling all blocks.") :
                        TranslateStdString("OSDMessage", "PGXP disabled, recompiling all blocks."),
                      5.0f);
        CPU::CodeCache::Flush();
      }

      if (old_settings.gpu_pgxp_enable)
        PGXP::Shutdown();

      if (g_settings.gpu_pgxp_enable)
        PGXP::Initialize();
    }

    if (g_settings.cdrom_readahead_sectors != old_settings.cdrom_readahead_sectors)
      g_cdrom.SetReadaheadSectors(g_settings.cdrom_readahead_sectors);

    if (g_settings.memory_card_types != old_settings.memory_card_types ||
        g_settings.memory_card_paths != old_settings.memory_card_paths ||
        (g_settings.memory_card_use_playlist_title != old_settings.memory_card_use_playlist_title &&
         System::HasMediaSubImages()) ||
        g_settings.memory_card_directory != old_settings.memory_card_directory)
    {
      System::UpdateMemoryCardTypes();
    }

    if (g_settings.rewind_enable != old_settings.rewind_enable ||
        g_settings.rewind_save_frequency != old_settings.rewind_save_frequency ||
        g_settings.rewind_save_slots != old_settings.rewind_save_slots ||
        g_settings.runahead_frames != old_settings.runahead_frames)
    {
      System::UpdateMemorySaveStateSettings();
    }

    if (g_settings.texture_replacements.enable_vram_write_replacements !=
          old_settings.texture_replacements.enable_vram_write_replacements ||
        g_settings.texture_replacements.preload_textures != old_settings.texture_replacements.preload_textures)
    {
      g_texture_replacements.Reload();
    }

    g_dma.SetMaxSliceTicks(g_settings.dma_max_slice_ticks);
    g_dma.SetHaltTicks(g_settings.dma_halt_ticks);
  }

  bool controllers_updated = false;
  for (u32 i = 0; i < NUM_CONTROLLER_AND_CARD_PORTS; i++)
  {
    if (g_settings.controller_types[i] != old_settings.controller_types[i])
    {
      if (!System::IsShutdown() && !controllers_updated)
      {
        System::UpdateControllers();
        System::ResetControllers();
        UpdateSoftwareCursor();
        controllers_updated = true;
      }

      OnControllerTypeChanged(i);
    }

    if (!System::IsShutdown() && !controllers_updated)
    {
      System::UpdateControllerSettings();
      UpdateSoftwareCursor();
    }
  }

  if (g_settings.multitap_mode != old_settings.multitap_mode)
    System::UpdateMultitaps();

  if (m_display && (g_settings.display_linear_filtering != old_settings.display_linear_filtering ||
                    g_settings.display_integer_scaling != old_settings.display_integer_scaling ||
                    g_settings.display_stretch != old_settings.display_stretch))
  {
    m_display->SetDisplayLinearFiltering(g_settings.display_linear_filtering);
    m_display->SetDisplayIntegerScaling(g_settings.display_integer_scaling);
    m_display->SetDisplayStretch(g_settings.display_stretch);
    OnDisplayInvalidated();
  }
}

void HostInterface::SetUserDirectoryToProgramDirectory()
{
  std::string program_path(FileSystem::GetProgramPath());
  if (program_path.empty())
    Panic("Failed to get program path.");

  std::string program_directory(FileSystem::GetPathDirectory(program_path));
  if (program_directory.empty())
    Panic("Program path is not valid");

  m_user_directory = std::move(program_directory);
}

void HostInterface::OnHostDisplayResized()
{
  if (System::IsValid())
  {
    if (g_settings.gpu_widescreen_hack && g_settings.display_aspect_ratio == DisplayAspectRatio::MatchWindow)
      GTE::UpdateAspectRatio();
  }
}

std::string HostInterface::GetUserDirectoryRelativePath(const char* format, ...) const
{
  std::va_list ap;
  va_start(ap, format);
  std::string formatted_path = StringUtil::StdStringFromFormatV(format, ap);
  va_end(ap);

  if (m_user_directory.empty())
  {
    return formatted_path;
  }
  else
  {
    return StringUtil::StdStringFromFormat("%s" FS_OSPATH_SEPARATOR_STR "%s", m_user_directory.c_str(),
                                           formatted_path.c_str());
  }
}

std::string HostInterface::GetProgramDirectoryRelativePath(const char* format, ...) const
{
  std::va_list ap;
  va_start(ap, format);
  std::string formatted_path = StringUtil::StdStringFromFormatV(format, ap);
  va_end(ap);

  if (m_program_directory.empty())
  {
    return formatted_path;
  }
  else
  {
    return StringUtil::StdStringFromFormat("%s" FS_OSPATH_SEPARATOR_STR "%s", m_program_directory.c_str(),
                                           formatted_path.c_str());
  }
}

TinyString HostInterface::GetTimestampStringForFileName()
{
  const Timestamp ts(Timestamp::Now());

  TinyString str;
  ts.ToString(str, "%Y-%m-%d_%H-%M-%S");
  return str;
}

std::string HostInterface::GetMemoryCardDirectory() const
{
  if (g_settings.memory_card_directory.empty())
    return GetUserDirectoryRelativePath("memcards");
  else
    return g_settings.memory_card_directory;
}

std::string HostInterface::GetSharedMemoryCardPath(u32 slot) const
{
  if (g_settings.memory_card_directory.empty())
  {
    return GetUserDirectoryRelativePath("memcards" FS_OSPATH_SEPARATOR_STR "shared_card_%u.mcd", slot + 1);
  }
  else
  {
    return StringUtil::StdStringFromFormat("%s" FS_OSPATH_SEPARATOR_STR "shared_card_%u.mcd",
                                           g_settings.memory_card_directory.c_str(), slot + 1);
  }
}

std::string HostInterface::GetGameMemoryCardPath(const char* game_code, u32 slot) const
{
  if (g_settings.memory_card_directory.empty())
  {
    return GetUserDirectoryRelativePath("memcards" FS_OSPATH_SEPARATOR_STR "%s_%u.mcd", game_code, slot + 1);
  }
  else
  {
    return StringUtil::StdStringFromFormat("%s" FS_OSPATH_SEPARATOR_STR "%s_%u.mcd",
                                           g_settings.memory_card_directory.c_str(), game_code, slot + 1);
  }
}

bool HostInterface::GetBoolSettingValue(const char* section, const char* key, bool default_value /*= false*/)
{
  std::string value = GetStringSettingValue(section, key, "");
  if (value.empty())
    return default_value;

  std::optional<bool> bool_value = StringUtil::FromChars<bool>(value);
  return bool_value.value_or(default_value);
}

int HostInterface::GetIntSettingValue(const char* section, const char* key, int default_value /*= 0*/)
{
  std::string value = GetStringSettingValue(section, key, "");
  if (value.empty())
    return default_value;

  std::optional<int> int_value = StringUtil::FromChars<int>(value);
  return int_value.value_or(default_value);
}

float HostInterface::GetFloatSettingValue(const char* section, const char* key, float default_value /*= 0.0f*/)
{
  std::string value = GetStringSettingValue(section, key, "");
  if (value.empty())
    return default_value;

  std::optional<float> float_value = StringUtil::FromChars<float>(value);
  return float_value.value_or(default_value);
}

TinyString HostInterface::TranslateString(const char* context, const char* str,
                                          const char* disambiguation /*= nullptr*/, int n /*= -1*/) const
{
  TinyString result(str);
  if (n >= 0)
  {
    const std::string number = std::to_string(n);
    result.Replace("%n", number.c_str());
    result.Replace("%Ln", number.c_str());
  }
  return result;
}

std::string HostInterface::TranslateStdString(const char* context, const char* str,
                                              const char* disambiguation /*= nullptr*/, int n /*= -1*/) const
{
  std::string result(str);
  if (n >= 0)
  {
    const std::string number = std::to_string(n);
    // Made to mimick Qt's behaviour
    // https://github.com/qt/qtbase/blob/255459250d450286a8c5c492dab3f6d3652171c9/src/corelib/kernel/qcoreapplication.cpp#L2099
    size_t percent_pos = 0;
    size_t len = 0;
    while ((percent_pos = result.find('%', percent_pos + len)) != std::string::npos)
    {
      len = 1;
      if (percent_pos + len == result.length())
        break;
      if (result[percent_pos + len] == 'L')
      {
        ++len;
        if (percent_pos + len == result.length())
          break;
      }
      if (result[percent_pos + len] == 'n')
      {
        ++len;
        result.replace(percent_pos, len, number);
        len = number.length();
      }
    }
  }

  return result;
}

void HostInterface::ToggleSoftwareRendering()
{
  if (System::IsShutdown() || g_settings.gpu_renderer == GPURenderer::Software)
    return;

  const GPURenderer new_renderer = g_gpu->IsHardwareRenderer() ? GPURenderer::Software : g_settings.gpu_renderer;

  AddFormattedOSDMessage(5.0f, TranslateString("OSDMessage", "Switching to %s renderer..."),
                         Settings::GetRendererDisplayName(new_renderer));
  System::RecreateGPU(new_renderer);
  OnDisplayInvalidated();
}

void HostInterface::ModifyResolutionScale(s32 increment)
{
  const u32 new_resolution_scale = std::clamp<u32>(
    static_cast<u32>(static_cast<s32>(g_settings.gpu_resolution_scale) + increment), 1, GPU::MAX_RESOLUTION_SCALE);
  if (new_resolution_scale == g_settings.gpu_resolution_scale)
    return;

  g_settings.gpu_resolution_scale = new_resolution_scale;

  if (!System::IsShutdown())
  {
    g_gpu->RestoreGraphicsAPIState();
    g_gpu->UpdateSettings();
    g_gpu->ResetGraphicsAPIState();
    System::ClearMemorySaveStates();
    OnDisplayInvalidated();
  }
}

void HostInterface::UpdateSoftwareCursor()
{
  if (System::IsShutdown())
  {
    SetMouseMode(false, false);
    m_display->ClearSoftwareCursor();
    return;
  }

  const Common::RGBA8Image* image = nullptr;
  float image_scale = 1.0f;
  bool relative_mode = false;
  bool hide_cursor = false;

  for (u32 i = 0; i < NUM_CONTROLLER_AND_CARD_PORTS; i++)
  {
    Controller* controller = System::GetController(i);
    if (controller && controller->GetSoftwareCursor(&image, &image_scale, &relative_mode))
    {
      hide_cursor = true;
      break;
    }
  }

  SetMouseMode(relative_mode, hide_cursor);

  if (image && image->IsValid())
  {
    m_display->SetSoftwareCursor(image->GetPixels(), image->GetWidth(), image->GetHeight(), image->GetByteStride(),
                                 image_scale);
  }
  else
  {
    m_display->ClearSoftwareCursor();
  }
}

void HostInterface::RecreateSystem()
{
  Assert(!System::IsShutdown());

  std::unique_ptr<ByteStream> stream = ByteStream_CreateGrowableMemoryStream(nullptr, 8 * 1024);
  if (!System::SaveState(stream.get(), 0) || !stream->SeekAbsolute(0))
  {
    ReportError("Failed to save state before system recreation. Shutting down.");
    DestroySystem();
    return;
  }

  DestroySystem();

  auto boot_params = std::make_shared<SystemBootParameters>();
  boot_params->state_stream = std::move(stream);
  if (!BootSystem(std::move(boot_params)))
  {
    ReportError("Failed to boot system after recreation.");
    return;
  }

  System::ResetPerformanceCounters();
  System::ResetThrottler();
  OnDisplayInvalidated();
}

void HostInterface::SetMouseMode(bool relative, bool hide_cursor) {}

void HostInterface::DisplayLoadingScreen(const char* message, int progress_min /*= -1*/, int progress_max /*= -1*/,
                                         int progress_value /*= -1*/)
{
  Log_InfoPrintf("Loading: %s %d of %d-%d", message, progress_value, progress_min, progress_max);
}

void HostInterface::GetGameInfo(const char* path, CDImage* image, std::string* code, std::string* title) {}
