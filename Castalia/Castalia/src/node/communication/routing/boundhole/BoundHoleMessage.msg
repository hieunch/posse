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

cplusplus {{
    #include "GeoMathHelper.h"
    #include "RoutingPacket_m.h"
    #include <vector>
    typedef std::vector<int> NodeVector;
}}

class noncobject Point;
class noncobject NodeVector;
class RoutingPacket;

enum BoundHoleType {
    BOUNDHOLE_BOUNDHOLE = 0x01;
    BOUNDHOLE_REFRESH = 0x02;
}

packet BoundHolePacket extends RoutingPacket {
    int sourceId;
    int destinationId;
    int packetId;
    int type;
    Point previousNode;
    int index; // for refresh type only
    NodeVector boundholeNodes;
}
