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

#ifndef UAVSM_HPP_
#define UAVSM_HPP_

#include <tinyfsm.hpp>

#include <gestelt_interfaces/srv/uav_command.hpp>

#include <iostream>

// ----------------------------------------------------------------------------
// Event declarations
//

struct CommandEvent : tinyfsm::Event
{
  double value; // Active in states [TakeOff]. Value which takes the context of the event. For Taking off, it is the height to take off to.
  int mode; // Active in states [Mission].
};

struct Idle_E : CommandEvent { };
struct TakeOff_E : CommandEvent { };
struct Land_E    : CommandEvent { };
struct Hover_E : CommandEvent { };
struct StartMission_E : CommandEvent { };
struct StopMission_E : CommandEvent { };
struct EmergencyStop_E : CommandEvent { };

// ----------------------------------------------------------------------------
// UAV (FSM base class) declaration
//

class UAV
: public tinyfsm::Fsm<UAV>
{
  /* NOTE: react(), entry() and exit() functions need to be accessible
   * from tinyfsm::Fsm class. You might as well declare friendship to
   * tinyfsm::Fsm, and make these functions private:
   *
   * friend class Fsm;
   */
public:

  /* default reaction for unhandled events */
  void react(tinyfsm::Event const &) { };

  /* React() methods to be defined in each state*/
  virtual void react(Idle_E const &);
  virtual void react(TakeOff_E const &);
  virtual void react(Land_E  const &);
  virtual void react(Hover_E const &);
  virtual void react(StartMission_E const &);
  virtual void react(StopMission_E const &);

  // Emergency stop react() method is default for all states 
  void         react(EmergencyStop_E  const &);

  virtual void entry(void) { };  /* entry actions in some states */
  void         exit(void)  { };  /* no exit actions at all */

  /* Get offboard control mode */
  int const getControlMode() const {
    return offb_ctrl_mode_;
  }

  /* Get offboard control mode */
  double const getTakeoffHeight() const{
    return take_off_height_;
  }

protected:
  static double take_off_height_;
  static int offb_ctrl_mode_;
};


// forward declarations
class Unconnected;
class Idle; 
class Landing;
class TakingOff;
class Hovering; 
class Mission; 
class EmergencyStop;

// ----------------------------------------------------------------------------
// State: Unconnected
//    Reacts to: Idle_E
//

class Unconnected
: public UAV
{
  void entry() override {
  }

  void react(Idle_E const & e) override {
    // Connected to PX4
    transit<Idle>();
  };
};

// ----------------------------------------------------------------------------
// State: Idle
//    Reacts to: TakeOff_E
//

class Idle
: public UAV
{
  void entry() override {
  }

  void react(TakeOff_E const & e) override {
    take_off_height_ = e.value;
    transit<TakingOff>();
  };

};

// ----------------------------------------------------------------------------
// State: Landing
//    Reacts to: Idle_E
//

class Landing
: public UAV
{
  void entry() override {
  }

  void react(Idle_E const & e) override {
    transit<Idle>();
  };

};

// ----------------------------------------------------------------------------
// State: TakingOff
//    Reacts to: Land_E, Hover_E
//

class TakingOff
: public UAV
{
  void entry() override {
  }

  void react(Land_E const & e) override {
    transit<Landing>();
  };

  void react(Hover_E const & e) override {
    transit<Hovering>();
  };

};

// ----------------------------------------------------------------------------
// State: Hovering
//    Reacts to: Land_E, StartMission_E
//

class Hovering
: public UAV
{
  void entry() override {
  }

  void react(Land_E const & e) override {
    transit<Landing>();
  };

  void react(StartMission_E const & e) override {
    offb_ctrl_mode_ = e.mode;
    transit<Mission>();
  };
};


// ----------------------------------------------------------------------------
// State: Mission
//    Reacts to: Land_E, StopMission_E
//

class Mission
: public UAV
{
  void entry() override {
  }

  void react(Land_E const & e) override {
    transit<Landing>();
  };

  void react(StopMission_E const & e) override {
    transit<Hovering>();
  };
};


// ----------------------------------------------------------------------------
// State: EmergencyStop
//    Reacts to: 
//

class EmergencyStop
: public UAV
{
  void entry() override {
  }
};

// ----------------------------------------------------------------------------
// Event dispatch
//

using fsm_list = tinyfsm::FsmList<UAV>;

/** dispatch event to "UAV" State Machine*/
template<typename E>
void sendEvent(E const & event)
{
  fsm_list::template dispatch<E>(event);
}

// ----------------------------------------------------------------------------
// Helper methods
//

// Note: Anything occupying the same name as the FSM states above will lead to a compilation error!

/* Enum for UAV state */
enum UAVStateEnum
{
  UNCONNECTED,    // 0
  IDLE,           // 1
  LANDING,        // 2
  TAKINGOFF,      // 3
  HOVERING,       // 4
  MISSION,        // 5
  EMERGENCYSTOP,  // 6
  UNDEFINED       // 7
};

inline UAVStateEnum getUAVState() {
	// Check all states
	if (UAV::is_in_state<Unconnected>()){
		return UAVStateEnum::UNCONNECTED;
	}
	else if (UAV::is_in_state<Idle>()){
		return UAVStateEnum::IDLE;
	}
	else if (UAV::is_in_state<Landing>()){
		return UAVStateEnum::LANDING;
	}
	else if (UAV::is_in_state<TakingOff>()){
		return UAVStateEnum::TAKINGOFF;
	}
	else if (UAV::is_in_state<Hovering>()){
		return UAVStateEnum::HOVERING;
	}
	else if (UAV::is_in_state<Mission>()){
		return UAVStateEnum::MISSION;
	}
	else if (UAV::is_in_state<EmergencyStop>()){
		return UAVStateEnum::EMERGENCYSTOP;
	}
	else {
		return UAVStateEnum::UNDEFINED;
	}
}

/** @brief getUAVStateString interprets the input server state **/
inline const std::string getUAVStateString() 
{
  switch (getUAVState())
  {
    case UAVStateEnum::UNCONNECTED:     return "UNCONNECTED";
    case UAVStateEnum::IDLE:     return "IDLE";
    case UAVStateEnum::LANDING:  return "LANDING";
    case UAVStateEnum::TAKINGOFF:     return "TAKINGOFF";
    case UAVStateEnum::HOVERING:    return "HOVERING";
    case UAVStateEnum::MISSION:  return "MISSION";
    case UAVStateEnum::EMERGENCYSTOP:   return "EMERGENCYSTOP";
    default:                    return "UNDEFINED";
  }
}

inline void sendUAVCommandEvent(const int& cmd, const double& value, const int& mode)
{
  switch (cmd)
  {
    case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_TAKEOFF:  
      {
        std::cout << "Sending take off event" << std::endl;
        auto evt = TakeOff_E();
        evt.value = value;
        sendEvent(evt);
      }
      break;
    case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_LAND:  
      {      
        std::cout << "Sending land event" << std::endl;
        auto evt = Land_E();
        sendEvent(evt);
      }
      break;
    case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_START_MISSION:  
      {      
        std::cout << "Sending start mission event" << std::endl;
        auto evt = StartMission_E();
        evt.mode = mode;
        sendEvent(evt);
      }
      break;
    case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_STOP_MISSION:  
      {      
        std::cout << "Sending stop mission event" << std::endl;
        auto evt = StopMission_E();
        sendEvent(evt);
      }
      break;
    case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_EMERGENCY_STOP:  
      {      
        std::cout << "Sending emergency stop event" << std::endl;
        auto evt = EmergencyStop_E();
        sendEvent(evt);
      }
      break;
    default:
      // Do nothing                    
      break;
  }
}

#endif //UAVSM_HPP_
