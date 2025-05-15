@0xa3b88992c3a363f6;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct Vector2f {
    x @0 :Float32;
    y @1 :Float32;
}

struct Vector2d {
    x @0 :Float64;
    y @1 :Float64;
}
