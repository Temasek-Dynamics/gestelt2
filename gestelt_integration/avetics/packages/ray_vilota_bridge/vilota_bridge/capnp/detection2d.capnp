@0x94839fd8bdb7c208;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct Detection2d {

    labelIdx @0 :UInt32;
    xmin @1 :Float32;
    xmax @2 :Float32;
    ymin @3 :Float32;
    ymax @4 :Float32;
    confidence @5 :Float32;

}

struct Detections2d {

    header @0 :import "header.capnp".Header;

    labels @1 :List(Text);

    image @2 :import "image.capnp".Image;

    detections @3 :List(Detection2d);

    
}