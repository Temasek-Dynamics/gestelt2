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

#include <iostream>
#include <uavsm.hpp>

// ----------------------------------------------------------------------------
// Base state: default implementations
//

void UAV::react(Idle_E const &) {
  std::cout << "Initialize event ignored" << std::endl;
}

void UAV::react(TakeOff_E const &) {
  std::cout << "TakeOff event ignored" << std::endl;
}

void UAV::react(Land_E const &) {
  std::cout << "Land event ignored" << std::endl;
}

void UAV::react(Hover_E const &) {
  std::cout << "Hover event ignored" << std::endl;
}

void UAV::react(StartMission_E const &) {
  std::cout << "Start mission event ignored" << std::endl;
}

void UAV::react(StopMission_E const &) {
  std::cout << "Stop mission event ignored" << std::endl;
}

void UAV::react(EmergencyStop_E const &) {
  std::cout << "Emergency stop engaged!" << std::endl;
  transit<EmergencyStop>();
}

double UAV::take_off_height_ = 0.0;
int UAV::offb_ctrl_mode_  = 0;

// ----------------------------------------------------------------------------
// Initial state definition
//
FSM_INITIAL_STATE(UAV, Unconnected)

