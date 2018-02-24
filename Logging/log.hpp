#ifndef LOG_H
#define LOG_H

#include "spdlog/spdlog.h"
#include <string>

// Literals for directories logs appear in
#define LOG_DIR "./logs"

#define LOG_1_NAME "all"
#define LOG_2_NAME "internal-error"
#define LOG_3_NAME "external-error"

#define SCRIPT_PATH "../Logging/create_log.sh"

class Log {
    public: 

    Log(int);
    void log(int, std::string);

    private:
    int input_level;
    std::shared_ptr<spdlog::logger> logger;

};

#endif
