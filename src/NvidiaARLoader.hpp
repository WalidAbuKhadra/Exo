#pragma once
#include <filesystem>
#include <string>

class NvidiaARLoader {
public:
  explicit NvidiaARLoader(const std::filesystem::path &appDir);
  ~NvidiaARLoader();

  [[nodiscard]] const char *getLocalModelPath() const;

private:
  bool m_usingLocal = false;
  std::string m_localModelPath;
};