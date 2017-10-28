#ifndef LOG_H
#define LOG_H

#include "spdlog/spdlog.h"
#include <string>

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
