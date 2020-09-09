package node.communication.routing.holevicinityrouting2;

import node.communication.routing.boundhole.BoundHole;

simple ICH2 extends BoundHole
{
    parameters:
        @display("i=block/star");
        @class(ICH2);
        storageOpt = default(1);
        double broadcastFactor = default(10);
        double scaleFactorCave = default(1.3);
        double scaleFactorConvex = default(1.3);
}