@0xa5565e1abe1d96a9;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct Image {

    enum Encoding {
        mono8 @0;
        mono16 @1;
        yuv420 @2;
        bgr8 @3;
        jpeg @4;
    }

    header @0 :import "header.capnp".Header;

    encoding @1 :Encoding;
    width @2 :UInt32;
    height @3 :UInt32;
    step @4 :UInt32; # bytes in a row
    data @5 :Data;
    
    # metadata
    exposureUSec @6 :UInt32;
    gain @7 :UInt32;

    sensorIdx @8 :Int8; # physical camera index
    streamName @9 :Text;

    intrinsic @10 :import "cameraintrinsic.capnp".CameraIntrinsic;
    extrinsic @11 :import "sensorextrinsic.capnp".SensorExtrinsic;

}