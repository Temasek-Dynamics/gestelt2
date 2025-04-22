@0xbb3159a4b85007b2;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("vkc");

struct SystemInfo {
    ramUsed @0 :UInt64;             # Number of bytes in the RAM currently being used.
    ramTotal @1 :UInt64;            # Total number of bytes in the RAM.
    avgTemperature @2 :Float32;     # Average temperature of all electrical components in the system (in Celsius).
    maxTemperature @3 :Float32;     # Max temperature of all electrical components in the system (in Celsius).
    cpuUsages @4 :List(Float32);    # Usage fraction of each CPU where each value is between 0.0 to 1.0.
}

struct CameraDriverInfo {
    id @0 : UInt32;                 # ID of the camera driver.
    temperature @1: Float32;        # Temperature of the hardware VPU that processes camera data (in Celsius).
    status @2: Text;                # Detailed status of the camera driver while it is alive.
    configuration @3: Text;         # Current configuration of the camera driver.
}

struct VioInfo {
    id @0 : UInt32;                 # ID of the VIO.
    configuration @1: Text;         # Current configuration of the VIO.
    status @2: Text;                # Detailed status of the VIO while it is alive.
}

struct SystemHeartbeat {
    info: union {                   # Heartbeat comes in one of these forms (may come independently).
        system @0 : SystemInfo;
        cameraDriver @1: CameraDriverInfo;
        vio @2: VioInfo;
    }
}


struct CameraCommand {
    id @0 : UInt32;                 # ID of the camera driver.
    detail : union {
        stop @1 : Void;             # Command is a stop command.
        reload @2: Text;            # Command is a reload command with the configuration name to be used attached.
    }
}

struct VioCommand {
    id @0 : UInt32;                 # ID of the VIO.
    detail : union {
        stop @1 : Void;             # Command is a stop command.
        reload @2: Text;            # Command is a reload command with the configuration name to be used attached.
    }
}

struct SystemCommand {
    variant: union {
        system @0 : Void;                   # Reserved for future use.
        cameraDriver @1 : CameraCommand;
        vio @2 : VioCommand;
    }
}