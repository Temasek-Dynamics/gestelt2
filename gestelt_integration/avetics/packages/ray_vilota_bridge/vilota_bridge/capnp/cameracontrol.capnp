@0x8b94c342930b4a59;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct CameraControl {
    header @0 :import "header.capnp".Header;

    streaming @1 :Int8; # 0 means stop streaming, 1 means start streaming, -1 means unchanged

    exposureUSec @2 :Int32; # range 1 - 33000, 0 means auto exposure, -1 means unchanged
    gain @3 :Int32; # range 100 - 1600, 0 means auto exposure, -1 means unchanged
    exposureCompensation @4 :Int8; # range -9 to 9, setting outside the range to disable

    sensorIdx @5 :Int8; # physical camera index, -1 means all cameras
    # streamName @6 :Text; # leave it empty for controlling all cameras
}