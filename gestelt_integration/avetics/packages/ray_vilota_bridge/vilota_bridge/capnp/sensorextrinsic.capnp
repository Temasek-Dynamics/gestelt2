@0xe1581d24055e468a;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct SensorExtrinsic {
    bodyFrame @0 :import "se3.capnp".Se3;
    imuFrame @1 :import "se3.capnp".Se3;

    # metadata
    lastModified @2 :UInt64; # UTC time in nanosecond
}