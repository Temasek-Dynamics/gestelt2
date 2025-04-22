@0xf290bd03f63a5e98;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct MavState {

    enum FlightModePX4 {
        unknown @0;
        ready @1;
        takeoff @2;
        hold @3;
        mission @4;
        returnToLaunch @5;
        land @6;
        offboard @7;
        followMe @8;
        manual @9;
        altitude @10;
        position @11;
        acro @12;
        stabilized @13;
        rattitude @14;
    }
    header @0 :import "header.capnp".Header;

    connected @1 :Bool;
    armed @2 :Bool;
    guided @3 :Bool;
    manualInput @4 :Bool;
    modePX4 @5 :FlightModePX4;
    systemStatus @6 :UInt8;

}