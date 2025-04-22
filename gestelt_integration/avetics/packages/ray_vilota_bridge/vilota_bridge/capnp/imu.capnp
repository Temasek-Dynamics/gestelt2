@0xebb5b4dfd4cfc717;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct ImuInstrinsic {
    gyroNoiseStd @0:import "vector3.capnp".Vector3d;
    accelNoiseStd @1:import "vector3.capnp".Vector3d;
    gyroBiasStd @2:import "vector3.capnp".Vector3d;
    accelBiasStd @3:import "vector3.capnp".Vector3d;

    updateRate @4 :Int32;
    timeOffsetNs @5 :Int64; # camera + timeOffsetNs = imu

    lastModified @6 :UInt64; # UTC time in nanosecond
}

struct Imu {
    header @0 :import "header.capnp".Header;

    linearAcceleration @1 :import "vector3.capnp".Vector3d;
    angularVelocity @2 :import "vector3.capnp".Vector3d;

    streamName @3 :Text;

    intrinsic @4 :ImuInstrinsic;
    extrinsic @5 :import "sensorextrinsic.capnp".SensorExtrinsic;

    # metadata
    seqIncrement @6 :Int32; # should be 1 typically, greater than 1 means jump, negative means regression
}