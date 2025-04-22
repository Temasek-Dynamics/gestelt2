@0x9df95ede2a032978;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

# https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html

struct Twist3 {
    linear @0 :import "vector3.capnp".Vector3d;
    angular @1 :import "vector3.capnp".Vector3d;
}