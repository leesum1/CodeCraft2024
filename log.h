#pragma once

#include <cassert>

void log_init(const char *log_path, int level);
void log_write(int level, const char *file, int line, const char *fmt, ...);
void log_raw(const char *fmt, ...);

enum LogLevel { INFO, WARN, DEBUG, TRACE, FATAL };

#define __FILENAME__ ((__FILE__) + (SOURCE_PATH_SIZE))
#ifndef SOURCE_PATH_SIZE
#define SOURCE_PATH_SIZE 0
#endif

// #define log_info(...) log_write(INFO, __FILENAME__, __LINE__, __VA_ARGS__)
// #define log_warn(...) log_write(WARN, __FILENAME__, __LINE__, __VA_ARGS__)
// #define log_debug(...) log_write(DEBUG, __FILENAME__, __LINE__, __VA_ARGS__)
// #define log_trace(...) log_write(TRACE, __FILENAME__, __LINE__, __VA_ARGS__)
// #define log_fatal(...)                                                         \
//   do {                                                                         \
//     log_write(FATAL, __FILE__, __LINE__, __VA_ARGS__);                         \
//     assert(false);                                                             \
//   } while (0)


// #define log_assert(cond, ...)                                                  \
//   do {                                                                         \
//     if (!(cond)) {                                                             \
//       log_write(FATAL, __FILE__, __LINE__, __VA_ARGS__);                       \
//       assert(false);                                                           \
//     }                                                                          \
//   } while (0)



#define log_info(...)
#define log_warn(...)
#define log_debug(...)
#define log_trace(...)
#define log_fatal(...)
#define log_assert(cond, ...)