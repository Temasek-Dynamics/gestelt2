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

#ifndef _LOGGER_WRAPPER_TIMER_HPP_
#define _LOGGER_WRAPPER_TIMER_HPP_

#include <iostream>
#include <chrono>
#include <mutex>

#include <logger_wrapper/definitions.hpp>
#include <rclcpp/rclcpp.hpp>

namespace logger_wrapper
{

class Timer
{
public:

  // Constructor.
  // name: Name of timer used for display
  // num_indent: A specified number of prepended spaces for visual clarity when displaying
  Timer(rclcpp::Logger logger, const std::string& name, int num_indent = 0)
  : name_(name), logger_(logger)
  {
    indent_str_ = std::string(num_indent, ' ');
  } 

  // start timer
  bool start() {
    if (timer_running_){ // True if timer is not running
      // std::cout << "Timer " << name_ << " is already running, ignoring start() function call!" << std::endl;
      return false;
    }

    std::lock_guard<std::mutex> cmd_guard(timer_mutex_);

    t_start_cpu_ = std::chrono::high_resolution_clock::now();
    t_start_wall_ = std::chrono::system_clock::now();
    timer_running_ = true;

    return true;
  }

  // stop timer
  bool stop(bool print_dur = false) {
    if (!timer_running_){ // True if timer is not running
      RCLCPP_INFO(logger_, "%sTimer[%s] is not running, ignoring stop() function call! \n", 
            indent_str_.c_str(), name_.c_str());

      return false;
    }

    std::lock_guard<std::mutex> cmd_guard(timer_mutex_);

    double t_dur_cpu = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::high_resolution_clock::now() - t_start_cpu_).count() * 1000.0;
    double t_dur_wall = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::system_clock::now() - t_start_wall_).count() * 1000.0;

    timer_running_ = false;

    t_cum_dur_cpu_ += t_dur_cpu;
    t_cum_dur_wall_ += t_dur_wall;

    if (print_dur) {

      RCLCPP_INFO(logger_, "%sTimer[%s](wall): %f ms \n" , 
            indent_str_.c_str(), (name_).c_str(), t_dur_wall);
    }

    iterations_++;

    return true;
  }

  // Get cumulative wall time
  double getWallCum(bool print_dur = false) const {
    if (print_dur) {

      RCLCPP_INFO(logger_, "%sTimer[%s](cum,wall): %f ms \n" , 
            indent_str_.c_str(), (name_).c_str(), t_cum_dur_wall_);
    }

    return t_cum_dur_wall_;
  }

  // Get average wall time
  double getWallAvg(bool print_dur = false) const {
    double t_avg_dur_wall = t_cum_dur_wall_ / iterations_;
    if (print_dur) {
      RCLCPP_INFO(logger_, "%sTimer[%s](avg,wall): %f ms \n" , 
            indent_str_.c_str(), (name_).c_str(), t_avg_dur_wall);
    }

    return t_avg_dur_wall;
  }

  // Get cumulative CPU Ticks 
  double getCPUCum(bool print_dur = false) const {
    if (print_dur) {
      RCLCPP_INFO(logger_, "%sTimer[%s](cum,cpu): %f ms \n" , 
            indent_str_.c_str(), (name_).c_str(), t_cum_dur_cpu_);
    }

    return t_cum_dur_cpu_;
  }

  // Get average CPU ticks
  double getCPUAvg(bool print_dur = false) const {
    double t_avg_dur_cpu = t_cum_dur_cpu_ / iterations_;

    if (print_dur) {
      RCLCPP_INFO(logger_, "%sTimer[%s](avg,cpu): %f ms \n" , 
            indent_str_.c_str(), (name_).c_str(), t_avg_dur_cpu);

    }

    return t_avg_dur_cpu;
  }

private:
  std::string name_; // name of timer
  rclcpp::Logger logger_; // ROS Logger 
  bool timer_running_{false}; // Indicates if timer is running

  std::chrono::time_point<std::chrono::high_resolution_clock> t_start_cpu_; // cpu time 
  std::chrono::time_point<std::chrono::system_clock>          t_start_wall_; // wall time

  double t_cum_dur_cpu_{0.0}; // Accumulative duration in ms
  double t_cum_dur_wall_{0.0}; // Accumulative duration in ms

  std::mutex timer_mutex_; // mutex for timer

  /* Display purposes */
  std::string indent_str_{""}; // String to represent indentation
  
  /* Memory */
  long iterations_{0};  // Number of start/stop cycles, used to generate average time

}; // class Timer

}  // namespace logger_wrapper

#endif // _LOGGER_WRAPPER_TIMER_HPP_
