package node.communication.routing.holevicinityrouting;

import node.communication.routing.boundhole.BoundHole;

simple ICH extends BoundHole
{
    parameters:
        @display("i=block/star");
        @class(ICH);
        storageOpt = default(1);
        double broadcastFactor = default(10);
        double scaleFactorCave = default(1.5);
        double scaleFactorConvex = default(1.3);
}