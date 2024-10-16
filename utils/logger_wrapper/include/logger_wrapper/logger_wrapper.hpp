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

#ifndef LOGGER_WRAPPER__LOGGER_WRAPPER_HPP_
#define LOGGER_WRAPPER__LOGGER_WRAPPER_HPP_

#include "logger_wrapper/visibility_control.h"

#include <rclcpp/rclcpp.hpp>

template<typename ... Args>
std::string strFmt( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

namespace logger_wrapper
{

class LoggerWrapper
{
public:
  LoggerWrapper(const rclcpp::Logger& node_logger, 
                rclcpp::Clock::SharedPtr clock);

  virtual ~LoggerWrapper();

  void logInfo(const std::string& str);

  void logWarn(const std::string& str);

  void logError(const std::string& str);

  void logFatal(const std::string& str);

  /* Throttled logging */

  void logInfoThrottle(const std::string& str, double period);

  void logWarnThrottle(const std::string& str, double period);

  void logErrorThrottle(const std::string& str, double period);

  void logFatalThrottle(const std::string& str, double period);

private:
  rclcpp::Logger node_logger_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace logger_wrapper

#endif  // LOGGER_WRAPPER__LOGGER_WRAPPER_HPP_
