#ifndef LOG_H
#define LOG_H

#include "spdlog/spdlog.h"
#include <string>

// Literals for directories logs appear in
#define LOG_DIR "./logs"
#define LOG_1 "/log1"
#define LOG_2 "/log2"
#define LOG_3 "/log3"

class Log {
    public: 

    Log();
    void level1_log(std::string);
    void level2_log(std::string);
    void level3_log(std::string);

    private:

    std::shared_ptr<spdlog::logger> logger1, logger2, logger3;

};

#endif
