#include "logger_wrapper/logger_wrapper.hpp"

namespace logger_wrapper
{

    LoggerWrapper::LoggerWrapper(   const rclcpp::Logger& node_logger, 
                                    rclcpp::Clock::SharedPtr clock)
    : node_logger_(node_logger), clock_(clock)
    {}

    LoggerWrapper::~LoggerWrapper()
    {
    }

    /* Normal Logging */

    void LoggerWrapper::logInfo(const std::string& str){
        RCLCPP_INFO(node_logger_, "%s", 
        str.c_str());
    }

    void LoggerWrapper::logWarn(const std::string& str){
        RCLCPP_WARN(node_logger_, "%s", 
        str.c_str());
    }

    void LoggerWrapper::logError(const std::string& str){
        RCLCPP_ERROR(node_logger_, "%s", 
        str.c_str());
    }

    void LoggerWrapper::logFatal(const std::string& str){
        RCLCPP_FATAL(node_logger_, "%s", 
        str.c_str());
    }

    /* Throttled logging */

    void LoggerWrapper::logInfoThrottle(const std::string& str, double period){
        RCLCPP_INFO_THROTTLE(node_logger_, *clock_, period*1000.0 ,
                            "%s", 
                            str.c_str());
    }

    void LoggerWrapper::logWarnThrottle(const std::string& str, double period){
        RCLCPP_WARN_THROTTLE(node_logger_, *clock_, period*1000.0 ,
                            "%s", 
                            str.c_str());
        
    }

    void LoggerWrapper::logErrorThrottle(const std::string& str, double period){
        RCLCPP_ERROR_THROTTLE(node_logger_, *clock_, period*1000.0 ,
                            "%s", 
                            str.c_str());
        
    }

    void LoggerWrapper::logFatalThrottle(const std::string& str, double period){
        RCLCPP_FATAL_THROTTLE(node_logger_, *clock_, period*1000.0 , 
                            "%s", 
                            str.c_str());
    }

}  // namespace logger_wrapper
