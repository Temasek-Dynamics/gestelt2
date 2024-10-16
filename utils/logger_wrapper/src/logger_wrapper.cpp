/****************************************************************************
 * MIT License
 *  
 *	Copyright (c) 2024 John Tan. All rights reserved.
 *
 *	Permission is hereby granted, free of charge, to any person obtaining a copy
 *	of this software and associated documentation files (the "Software"), to deal
 *	in the Software without restriction, including without limitation the rights
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the Software is
 *	furnished to do so, subject to the following conditions:
 *
 *	The above copyright notice and this permission notice shall be included in all
 *	copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *	SOFTWARE.
 *
 ****************************************************************************/

#include "logger_wrapper/logger_wrapper.hpp"

namespace logger_wrapper
{

    LoggerWrapper::LoggerWrapper(   const rclcpp::Logger& node_logger, 
                                    rclcpp::Clock::SharedPtr clock)
    : node_logger_(node_logger), clock_(clock)
    {}

    LoggerWrapper::~LoggerWrapper()
    {}

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
