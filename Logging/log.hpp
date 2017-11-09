#ifndef LOG_H
#define LOG_H

#include "spdlog/spdlog.h"
#include <string>

// Literals for directories logs appear in
#define LOG_DIR "./logs"
#define LOG_1 "/level1"
#define LOG_2 "/level2"
#define LOG_3 "/level3"

#define LOG_1_NAME "all"
#define LOG_2_NAME "internal-error"
#define LOG_3_NAME "external-error"

#define SCRIPT_PATH "../Logging/create_log.sh"

class Log {
    public: 

    Log();
    void level1(std::string);
    void level2(std::string);
    void level3(std::string);

    private:

    std::shared_ptr<spdlog::logger> logger1, logger2, logger3;

};

#endif
