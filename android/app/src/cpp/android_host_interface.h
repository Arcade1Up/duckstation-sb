#pragma once
#include "android_settings_interface.h"
#include "common/byte_stream.h"
#include "common/event.h"
#include "common/progress_callback.h"
#include "frontend-common/common_host_interface.h"
#include <array>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <jni.h>
#include <memory>
#include <string>
#include <thread>

struct ANativeWindow;

class Controller;

class AndroidHostInterface final : public CommonHostInterface
{
public:
  AndroidHostInterface(jobject java_object, jobject context_object, std::string user_directory);
  ~AndroidHostInterface() override;

  bool Initialize() override;
  void Shutdown() override;

  const char* GetFrontendName() const override;
  void RequestExit() override;

  void ReportError(const char* message) override;
  void ReportMessage(const char* message) override;

  std::string GetStringSettingValue(const char* section, const char* key, const char* default_value = "") override;
  bool GetBoolSettingValue(const char* section, const char* key, bool default_value = false) override;
  int GetIntSettingValue(const char* section, const char* key, int default_value = 0) override;
  float GetFloatSettingValue(const char* section, const char* key, float default_value = 0.0f) override;
  std::unique_ptr<ByteStream> OpenPackageFile(const char* path, u32 flags) override;

  bool IsEmulationThreadRunning() const { return m_emulation_thread.joinable(); }
  bool IsEmulationThreadPaused() const;
  bool StartEmulationThread(jobject emulation_activity, ANativeWindow* initial_surface,
                            std::shared_ptr<SystemBootParameters> boot_params, bool resume_state);
  void RunOnEmulationThread(std::function<void()> function, bool blocking = false);
  void PauseEmulationThread(bool paused);
  void StopEmulationThread();
  void SurfaceChanged(ANativeWindow* surface, int format, int width, int height);

  void SetControllerType(u32 index, std::string_view type_name);
  void SetControllerButtonState(u32 index, s32 button_code, bool pressed);
  void SetControllerAxisState(u32 index, s32 button_code, float value);
  void SetFastForwardEnabled(bool enabled);

  void RefreshGameList(bool invalidate_cache, bool invalidate_database, ProgressCallback* progress_callback);
  void ApplySettings(bool display_osd_messages);

  bool ImportPatchCodesFromString(const std::string& str);

protected:
  void SetUserDirectory() override;
  void LoadSettings() ;
  void UpdateInputMap();

  bool AcquireHostDisplay() override;
  void ReleaseHostDisplay() override;
  std::unique_ptr<AudioStream> CreateAudioStream(AudioBackend backend) override;

  void OnSystemPaused(bool paused) override;
  void OnSystemDestroyed() override;
  void OnRunningGameChanged();

private:
  void EmulationThreadEntryPoint(jobject emulation_activity, ANativeWindow* initial_surface,
                                 std::shared_ptr<SystemBootParameters> boot_params, bool resume_state);
  void EmulationThreadLoop();

  void CreateImGuiContext();
  void DestroyImGuiContext();

  void LoadAndConvertSettings();
  void SetVibration(bool enabled);
  void UpdateVibration();

  jobject m_java_object = {};
  jobject m_emulation_activity_object = {};

  //m_settings_interface = std::make_unique< AndroidSettingsInterface m_settings_interface;

  ANativeWindow* m_surface = nullptr;

  std::mutex m_mutex;
  std::condition_variable m_sleep_cv;
  std::deque<std::function<void()>> m_callback_queue;
  std::atomic_bool m_callbacks_outstanding{false};

  std::thread m_emulation_thread;
  std::atomic_bool m_emulation_thread_stop_request{false};

  u64 m_last_vibration_update_time = 0;
  bool m_last_vibration_state = false;
  bool m_vibration_enabled = false;
};

namespace AndroidHelpers {
JNIEnv* GetJNIEnv();
AndroidHostInterface* GetNativeClass(JNIEnv* env, jobject obj);
std::string JStringToString(JNIEnv* env, jstring str);
std::unique_ptr<GrowableMemoryByteStream> ReadInputStreamToMemory(JNIEnv* env, jobject obj, u32 chunk_size = 65536);
} // namespace AndroidHelpers

template<typename T>
class LocalRefHolder
{
public:
  LocalRefHolder() : m_env(nullptr), m_object(nullptr) {}

  LocalRefHolder(JNIEnv* env, T object) : m_env(env), m_object(object) {}

  LocalRefHolder(const LocalRefHolder<T>&) = delete;
  LocalRefHolder(LocalRefHolder&& move) : m_env(move.m_env), m_object(move.m_object)
  {
    move.m_env = nullptr;
    move.m_object = {};
  }

  ~LocalRefHolder()
  {
    if (m_object)
      m_env->DeleteLocalRef(m_object);
  }

  operator T() const { return m_object; }
  T operator*() const { return m_object; }

  LocalRefHolder& operator=(const LocalRefHolder&) = delete;
  LocalRefHolder& operator=(LocalRefHolder&& move)
  {
    if (m_object)
      m_env->DeleteLocalRef(m_object);
    m_env = move.m_env;
    m_object = move.m_object;
    move.m_env = nullptr;
    move.m_object = {};
    return *this;
  }

  T Get() const { return m_object; }

private:
  JNIEnv* m_env;
  T m_object;
};
static pthread_t gTrackBallID;
extern void *TrackBallThread(void *ptr);
extern u16 gTrackBallKeys;
extern int lTrackBallDirection[4];