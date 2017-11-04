#include "log.hpp"

Log::Log() {
    auto console = spdlog::stdout_color_mt("console");
    console->info("spdlog logging to stdout") ;
    console->info("An info message example {}..", 1);

    logger1 = spdlog::basic_logger_mt("logger1", LOG_DIR LOG_1);
    logger2 = spdlog::basic_logger_mt("logger2", LOG_DIR LOG_2);
    logger3 = spdlog::basic_logger_mt("logger3", LOG_DIR LOG_3);
}

void Log::level1_log(std::string msg) {
    logger1->info(msg);
}

void Log::level2_log(std::string msg) {
    logger2->info(msg);
    logger1->info(msg);
}

void Log::level3_log(std::string msg) {
    logger3->info(msg);
    logger1->info(msg);
}