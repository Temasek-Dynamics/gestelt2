@0xe3148e3e52c75da3;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("vkc");

struct Field {

    enum NumericType {
        float32 @0;
        float64 @1;
        int8 @2;
        int16 @3;
        int32 @4;
        int64 @5;
        uint8 @6;
        uint16 @7;
        uint32 @8;
        uint64 @9;
    }

    name @0 :Text;
    offset @1 :UInt32; # byte offset from start of data buffer
    type @2 :NumericType;
}

struct PointCloud {
    header @0 :import "header.capnp".Header;
    pose @1 :import "odometry3d.capnp".Odometry3d;
    pointStride @2 :UInt32; # number of bytes between points in `points`
    fields @3 :List(Field); # fields in each point
    points @4 :Data;
}
