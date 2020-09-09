//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package node.communication.routing.boundhole;

// import node.communication.routing.base.RoutingBaseAlgorithm;


//
// TODO auto-generated module
//
simple BoundHole like node.communication.routing.iRouting//RoutingBaseAlgorithm
{
    parameters:     
        // double range @unit("m") = default(40m);
        bool collectTraceInfo = default (false);
        int maxNetFrameSize = default (0);					// bytes, 0 means no limit
        int netDataFrameOverhead = default (14);			// bytes
        int netBufferSize = default (32);					// number of messages

        int storageOpt @enum(0,1) = default(0); // 0 for STORAGE_ALL, 1 for STORAGE_ONE
        int limitMaxHop = default(80); 
        int limitMinHop = default(10);
        double startTentRule @unit(s) = default(50s);
        double startBoundHole @unit(s) = default(60s);

    gates:
        output toCommunicationModule;
	    output toMacModule;
	    input fromCommunicationModule;
        input fromMacModule;
        input fromCommModuleResourceMgr;
        input fromDirect @directIn;
}
