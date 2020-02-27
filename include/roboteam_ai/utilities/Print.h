#ifndef RTT_ROBOTEAM_AI_SRC_UTILITIES_PRINT_H_
#define RTT_ROBOTEAM_AI_SRC_UTILITIES_PRINT_H_

#include <iostream>
#include <iomanip>

class Printer {
 public:
  static void print(const std::string& color, const std::string& type, const std::string& file, const std::string& func, int line, const std::string& txt) {
      auto typeTxt = "[" + type + "]";
      auto fileTxt = "[" + file + "::" + std::to_string(line) + "]";
      auto funcTxt = "[" + func + "]";

      std::cout << color << std::setw(10) << std::left;\
      std::cout << typeTxt << std::setw(32) << std::left;\
      std::cout << fileTxt << std::setw(32) << std::left;\
      std::cout << funcTxt << std::setw(32) << std::left;\
      std::cout << txt << "\033[0m" << std::endl;\
  }
};

#ifndef DEBUG_LOGS
    #define DEBUG_LOGS 1 // set debug mode
#endif

#if DEBUG_LOGS
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define rtt_debug(...) { Printer::print("\033[37m", "DEBUG", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#define rtt_error(...) { Printer::print("\033[91m","ERROR", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#define rtt_warning(...) { Printer::print("\033[93m", "WARNING", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#define rtt_info(...) { Printer::print("\033[94m", "INFO", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#define rtt_success(...) { Printer::print("\033[92m", "SUCCESS", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }

#else
    #define rtt_debug(...)
    #define rtt_error(...)
    #define rtt_warning(...)
    #define rtt_info(...)
    #define rtt_success(...)
#endif

#endif //RTT_ROBOTEAM_AI_SRC_UTILITIES_PRINT_H_
