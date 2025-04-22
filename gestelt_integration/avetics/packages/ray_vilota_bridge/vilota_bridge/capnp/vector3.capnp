@0xb582b480fc4d3622;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct Vector3f {
    x @0 :Float32;
    y @1 :Float32;
    z @2 :Float32; 
}

struct Vector3d {
    x @0 :Float64;
    y @1 :Float64;
    z @2 :Float64; 
}