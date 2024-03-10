#pragma once

void log_init(const char *log_path, int level);
void log_write(int level, const char *file, int line, const char *fmt, ...);
void log_raw(const char *fmt, ...);

enum LogLevel { INFO, WARN, DEBUG, TRACE, FATAL };




#define log_info(...) log_write(INFO, __FILE__, __LINE__, __VA_ARGS__)
#define log_warn(...) log_write(WARN, __FILE__, __LINE__, __VA_ARGS__)
#define log_debug(...) log_write(DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#define log_trace(...) log_write(TRACE, __FILE__, __LINE__, __VA_ARGS__)
#define log_fatal(...) log_write(FATAL, __FILE__, __LINE__, __VA_ARGS__)
