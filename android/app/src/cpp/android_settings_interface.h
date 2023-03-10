#pragma once
#include "core/settings.h"
#include <jni.h>

class AndroidSettingsInterface : public SettingsInterface
{
public:
  AndroidSettingsInterface(jobject java_context);
  ~AndroidSettingsInterface();

  void Clear() override;

  int GetIntValue(const char* section, const char* key, int default_value = 0) override;
  float GetFloatValue(const char* section, const char* key, float default_value = 0.0f) override;
  bool GetBoolValue(const char* section, const char* key, bool default_value = false) override;
  std::string GetStringValue(const char* section, const char* key, const char* default_value = "") override;

  void SetIntValue(const char* section, const char* key, int value) override;
  void SetFloatValue(const char* section, const char* key, float value) override;
  void SetBoolValue(const char* section, const char* key, bool value) override;
  void SetStringValue(const char* section, const char* key, const char* value) override;
  void DeleteValue(const char* section, const char* key) override;

  std::vector<std::string> GetStringList(const char* section, const char* key) override;
  //void SetStringList(const char* section, const char* key, const std::vector<std::string>& items) override;
  bool RemoveFromStringList(const char* section, const char* key, const char* item) override;
  bool AddToStringList(const char* section, const char* key, const char* item) override;

private:
  jclass m_set_class{};
  jclass m_shared_preferences_class{};
  jobject m_java_shared_preferences{};
  jmethodID m_get_boolean{};
  jmethodID m_get_int{};
  jmethodID m_get_float{};
  jmethodID m_get_string{};
  jmethodID m_get_string_set{};
  jmethodID m_set_to_array{};
};
