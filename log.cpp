
#include <array>
#include <cstdarg>
#include <ctime>
#include <fstream>
#include <ios>
#include <optional>
#include <string>

static std::optional<std::ofstream> log_file;
static int log_level;
static std::array<std::string, 5> log_level_str = {"INFO", "WARN", "DEBUG",
                                                   "TRACE", "FATAL"};

void log_init(const char *log_path, int level) {
  log_file.emplace(log_path, std::ios::out | std::ios::trunc);
  log_level = level;
}

void log_raw(const char *fmt, ...) {
  if (!log_file.has_value()) {
    return;
  }
  static char buf[1000];

  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  log_file.value() << buf;
  log_file.value().flush();
}

void log_write(int level, const char *file, int line, const char *fmt, ...) {
  if (level < log_level || !log_file.has_value()) {
    return;
  }
  static char buf[1000];

  std::time_t t = std::time(nullptr);
  char timebuf[20];
  std::strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S",
                std::localtime(&t));

  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  log_file.value() << timebuf << " " << log_level_str[level] << " " << file
                   << ":" << line << " " << buf << std::endl;
  log_file.value().flush();
}
