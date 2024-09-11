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

#include <iostream>

// ----------------------------------------------------------------------------
// Event declarations
//

struct CommandEvent : tinyfsm::Event
{
  double val; // Active in states [TakeOff], [Hover]. Value which takes the context of the event. For Taking off, it is the height to take off to. For hover, it is the height to hover at.
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

protected:

  // static constexpr int initial_floor = 0;
  // static int current_floor;
  // static int dest_floor;
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
    std::cout<< "Entered Unconnected " << std::endl;
    // Do nothing, wait for connection to Flight Controller Unit
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
    std::cout<< "Entered Idle " << std::endl;
    // Disarm
    // MotorDisarm()
  }

  void react(TakeOff_E const & e) override {
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
    std::cout<< "Entered Landing " << std::endl;
    // send_event(MotorLand());
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
    std::cout<< "Entered TakingOff " << std::endl;
    // send_event(MotorArm());
    // send_event(MotorTakeOff());
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
    std::cout<< "Entered Hovering " << std::endl;
    // send_event(MotorHover());
  }

  void react(Land_E const & e) override {
    transit<Landing>();
  };

  void react(StartMission_E const & e) override {
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
    std::cout<< "Entered Mission " << std::endl;
    // send_event(MotorMission());
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
    std::cout<< "Entered EmergencyStop " << std::endl;
    // send_event(MotorEmergencyStop());
  }
};

// ----------------------------------------------------------------------------
// Event dispatch
//

using fsm_list = tinyfsm::FsmList<UAV>;

/** dispatch event to "UAV" */
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

/* Enum for UAV Event */
enum UAVCommandEnum
{
  TAKEOFF_E,           // 0
  LAND_E,    // 1
  HOVER_E,        // 2
  STARTMISSION_E,      // 3
  STOPMISSION_E,       // 4
  EMERGENCYSTOP_E        // 5
};

inline UAVStateEnum getUAVState() {
	// Check all states
	if (UAV::is_in_state<Unconnected>()){
		std::cout << "Unconnected" << std::endl;
		return UAVStateEnum::UNCONNECTED;
	}
	else if (UAV::is_in_state<Idle>()){
		std::cout << "Idle" << std::endl;

		return UAVStateEnum::IDLE;
	}
	else if (UAV::is_in_state<Landing>()){
		std::cout << "Landing" << std::endl;

		return UAVStateEnum::LANDING;
	}
	else if (UAV::is_in_state<TakingOff>()){
		std::cout << "TakingOff" << std::endl;

		return UAVStateEnum::TAKINGOFF;
	}
	else if (UAV::is_in_state<Hovering>()){
		std::cout << "Hovering" << std::endl;

		return UAVStateEnum::HOVERING;
	}
	else if (UAV::is_in_state<Mission>()){
		std::cout << "Mission" << std::endl;

		return UAVStateEnum::MISSION;
	}
	else if (UAV::is_in_state<EmergencyStop>()){
		std::cout << "EmergencyStop" << std::endl;

		return UAVStateEnum::EMERGENCYSTOP;
	}
	else {
		std::cout << "Undefined UAV state" << std::endl;

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

inline CommandEvent UAVCommandToEvent(const int& cmd)
{
  switch (cmd)
  {
    case UAVCommandEnum::TAKEOFF_E:     return TakeOff_E();
    case UAVCommandEnum::LAND_E:  return Land_E();
    case UAVCommandEnum::HOVER_E:    return Hover_E();
    case UAVCommandEnum::STARTMISSION_E:  return StartMission_E();
    case UAVCommandEnum::STOPMISSION_E:  return StartMission_E();
    case UAVCommandEnum::EMERGENCYSTOP_E:   return EmergencyStop_E();
    default:                    return Land_E();
  }
}

#endif //UAVSM_HPP_
