package node.communication.routing.vicinityhole;

import node.communication.routing.boundhole.BoundHole;

//
// TODO auto-generated module
//
simple VHR extends BoundHole
{
    parameters:
        @display("i=block/star");
        @class(VHR);
        storageOpt = default(1);
        double broadcastFactor = default(1.5);
        double scaleFactor = default(0.5);
}
