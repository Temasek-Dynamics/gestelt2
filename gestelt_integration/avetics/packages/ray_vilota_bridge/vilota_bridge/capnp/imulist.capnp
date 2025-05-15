@0x8b45b1c7f76a5302;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct ImuList {
    list @0 :List(import "imu.capnp".Imu);
}