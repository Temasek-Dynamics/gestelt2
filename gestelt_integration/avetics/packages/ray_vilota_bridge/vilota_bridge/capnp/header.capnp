@0xc61974417b74b4cc;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct Header {

    enum ClockDomain {
        monotonic @0;
        realtime @1;
    }

    seq @0 :UInt64;
    stamp @1 :UInt64; # time when data is created (captured, measured etc)
    frameId @2 :Text; # not intended to be used, only for compatibility of ros message header
    clockDomain @3 :ClockDomain;
    latencyDevice @4 :UInt64; # latency introduced until reception at device CPU
    latencyHost @5 :UInt64; # latency introducted until reception at host CPU
}