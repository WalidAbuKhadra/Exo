#include "NvidiaARLoader.hpp"

#include <Windows.h>
#include <filesystem>
#include <string>

char *g_nvARSDKPath = nullptr;
static std::string g_nvARSDKPathStorage;

NvidiaARLoader::NvidiaARLoader(const std::filesystem::path &appDir) {
  auto localRoot = (appDir / "ARSDK").lexically_normal();
  auto localDllDir = localRoot / "bin";

  if (std::filesystem::exists(localDllDir)) {
    m_usingLocal = true;

    g_nvARSDKPathStorage = localDllDir.string();
    g_nvARSDKPath = const_cast<char *>(g_nvARSDKPathStorage.c_str());

    SetDllDirectoryA(localDllDir.string().c_str());

    auto modelDir = localDllDir / "models";
    if (std::filesystem::exists(modelDir)) {
      m_localModelPath = modelDir.string();
    }
  }
}

NvidiaARLoader::~NvidiaARLoader() {
  if (m_usingLocal) {
    SetDllDirectoryA(nullptr);
  }
}

const char *NvidiaARLoader::getLocalModelPath() const {
  return m_usingLocal && !m_localModelPath.empty() ? m_localModelPath.c_str() : nullptr;
}