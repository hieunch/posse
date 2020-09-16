/****************************************************************************
 *  Copyright: National ICT Australia,  2007 - 2011                         *
 *  Developed at the ATP lab, Networked Systems research theme              *
 *  Author(s): Yuriy Tselishchev, Athanassios Boulis                        *
 *  This file is distributed under the terms in the attached LICENSE file.  *
 *  If you do not find this file, copies can be found by writing to:        *
 *                                                                          *
 *      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia             *
 *      Attention:  License Inquiry.                                        *
 *                                                                          *
 ****************************************************************************/

#include "VirtualRouting.h"

bool VirtualRouting::initialized = false;
void VirtualRouting::initialize()
{
  GlobalLocationService::initialize(getParentModule()->getParentModule()->getParentModule());
  if (!initialized) {
    initialized = true;
    srand(time(NULL));
  }
	maxNetFrameSize = par("maxNetFrameSize");
	netDataFrameOverhead = par("netDataFrameOverhead");
	netBufferSize = par("netBufferSize");
	numPacketReceived = 0;
	deathTime = 10000;

	/* Get a valid references to the Resources Manager module and the
	 * Radio module, so that we can make direct calls to their public methods
	 */
	radioModule = check_and_cast <Radio*>(getParentModule()->getSubmodule("Radio"));
	resMgrModule = check_and_cast <ResourceManager*>(getParentModule()->getParentModule()->getSubmodule("ResourceManager"));


	if (!resMgrModule || !radioModule)
		opp_error("\n Virtual Routing init: Error in geting a valid reference module(s).");

	self = getParentModule()->getParentModule()->getIndex();
    neighborTable = GlobalLocationService::getNeighborTable(self);
    selfLocation = GlobalLocationService::getLocation(self);
	// create the routing level address using self
	stringstream out; out << self; 	selfAddress = out.str();

	cpuClockDrift = resMgrModule->getCPUClockDrift();
	setTimerDrift(cpuClockDrift);
	pktHistory.clear();

	disabled = true;
	currentSequenceNumber = 0;

	declareOutput("Buffer overflow");
}

// A function to send control messages to lower layers
void VirtualRouting::toMacLayer(cMessage * msg)
{
	if (msg->getKind() == NETWORK_LAYER_PACKET)
		opp_error("toMacLayer() function used incorrectly to send NETWORK_LAYER_PACKET without destination MAC address");
	send(msg, "toMacModule");
}

// A function to send packets to MAC, requires a destination address
void VirtualRouting::toMacLayer(cPacket * pkt, int destination)
{
	RoutingPacket *netPacket = check_and_cast <RoutingPacket*>(pkt);

  // energy ---------
	double byteLength = netPacket->getByteLength();
	// for (int i=0; i<10; i++) trace() << "OK " << destination;
	Point nextHopLocation = GlobalLocationService::getLocation(destination);
	// for (int i=0; i<10; i++) trace() << "OK2";
	double distance = G::distance(selfLocation, nextHopLocation);
	double powerNeeded = 3 * 3600* (50e-8 * byteLength * 8 + 100e-12 * byteLength * 8 * distance * distance);
	trace() << "byteLength " << byteLength << " powerNeeded " << powerNeeded;
	resMgrModule->consumeEnergy(powerNeeded);
	// --------------

	netPacket->getNetMacInfoExchange().nextHop = destination;
	if (destination == -1) {
		// for (int i=0; i<10; i++) trace() << "toMacModule";
	    send(netPacket, "toMacModule");
		// for (int i=0; i<10; i++) trace() << "finished";
	} else {
//        debugLine(selfLocation, GlobalLocationService::getLocation(destination), "black");
        cModule* desModule = getParentModule()->getParentModule()->getParentModule()->getSubmodule("node", destination)
            ->getSubmodule("Communication")->getSubmodule("Routing");
			// for (int i=0; i<10; i++) trace() << "toMacModule";
        sendDirect(netPacket, desModule, "fromDirect");
		// for (int i=0; i<10; i++) trace() << "finished";
	}

//
}

void VirtualRouting::toApplicationLayer(cMessage * msg)
{
	send(msg, "toCommunicationModule");
}

void VirtualRouting::encapsulatePacket(cPacket * pkt, cPacket * appPkt)
{
	RoutingPacket *netPkt = check_and_cast <RoutingPacket*>(pkt);
	// set the size to just the overhead. encapsulate(appPkt)
	// will add the size of the app packet automatically
	netPkt->setByteLength(netDataFrameOverhead);
	netPkt->setKind(NETWORK_LAYER_PACKET);
	netPkt->setSource(SELF_NETWORK_ADDRESS);
	netPkt->setSequenceNumber(currentSequenceNumber++);
	netPkt->encapsulate(appPkt);
}

cPacket* VirtualRouting::decapsulatePacket(cPacket * pkt)
{
	RoutingPacket *netPkt = check_and_cast <RoutingPacket*>(pkt);
	ApplicationPacket *appPkt = check_and_cast <ApplicationPacket*>(netPkt->decapsulate());

	appPkt->getAppNetInfoExchange().RSSI = netPkt->getNetMacInfoExchange().RSSI;
	appPkt->getAppNetInfoExchange().LQI = netPkt->getNetMacInfoExchange().LQI;
	appPkt->getAppNetInfoExchange().source = netPkt->getSource();
	return appPkt;
}

void VirtualRouting::handleMessage(cMessage * msg)
{
	int msgKind = msg->getKind();
	if (disabled && msgKind != NODE_STARTUP)
	{
		delete msg;
		return;
	}

	switch (msgKind) {

		case NODE_STARTUP:
		{
			disabled = false;
			send(new cMessage("Network --> Mac startup message", NODE_STARTUP), "toMacModule");
			startup();
			break;
		}

		case APPLICATION_PACKET:
		{
			ApplicationPacket *appPacket = check_and_cast <ApplicationPacket*>(msg);
			if (maxNetFrameSize > 0 && maxNetFrameSize < appPacket->getByteLength() + netDataFrameOverhead)
			{
				trace() << "Oversized packet dropped. Size:" << appPacket->getByteLength() <<
				    ", Network layer overhead:" << netDataFrameOverhead <<
				    ", max Network packet size:" << maxNetFrameSize;
				break;
			}
			trace1() << "Received [" << appPacket->getName() << "] from application layer";

			/* Control is now passed to a specific routing protocol by calling fromApplicationLayer()
			 * Notice that after the call we RETURN (not BREAK) so that the packet is not deleted.
			 * This is done since the packet will most likely be encapsulated and forwarded to the
			 * MAC layer. If the protocol specific function wants to discard the packet is has
			 * to delete it.
			 */
			numPacketReceived++;
			fromApplicationLayer(appPacket, appPacket->getAppNetInfoExchange().destination.c_str());
			return;
		}

		case NETWORK_LAYER_PACKET:
		{
			RoutingPacket *netPacket = check_and_cast <RoutingPacket*>(msg);
			/* trace() << "Received [" << netPacket->getName() << "] from MAC layer"; */
			NetMacInfoExchange_type info = netPacket->getNetMacInfoExchange();

			/* Control is now passed to a specific routing protocol by calling fromMacLayer()
			 * Notice that after the call we BREAK so that the NET packet gets deleted.
			 * This will not delete the encapsulated APP packet if it gets decapsulated
			 * by fromMacLayer(), i.e., the normal/expected action.
			 */
			numPacketReceived++;
			GlobalLocationService::increaseNumReceived(self);
      // energy ---------
      double byteLength = netPacket->getByteLength();
      Point nextHopLocation = GlobalLocationService::getLocation(info.lastHop);
      double distance = G::distance(selfLocation, nextHopLocation);
      double powerNeeded = 3 * 3600*(50e-8 * byteLength * 8);
      resMgrModule->consumeEnergy(powerNeeded);
      // --------------




			if (netPacket->getTTL() > 0) {
			  string dst(netPacket->getDestination());


			  netPacket->setTTL(netPacket->getTTL() - 1);
			  netPacket->setHopCount(netPacket->getHopCount() + 1);
			//   netPacket->setDistanceCount(netPacket->getDistanceCount() + distance);
        if (dst.compare(SELF_NETWORK_ADDRESS) == 0) {
          double hopCount = (double) netPacket->getHopCount();
		//   double distanceCount = netPacket->getDistanceCount();
          int s = atoi(netPacket->getSource());
          int d = atoi(netPacket->getDestination());
          double ratio = hopCount / GlobalLocationService::numHopShortestPath(s, d);
		  trace() << "hop ratio " << ratio << " " << hopCount << " " << GlobalLocationService::numHopShortestPath(s, d);
          sumRatioEndCount += ratio;
		  maxRatio = max(maxRatio, ratio);
		//   sumDistanceEndCount += distanceCount;
          endCount++;
        }
			  fromMacLayer(netPacket, info.lastHop, info.RSSI, info.LQI);
			}

			break;
		}

		case TIMER_SERVICE:{
			handleTimerMessage(msg);
			break;
		}

		case MAC_CONTROL_MESSAGE:{
			handleMacControlMessage(msg);
			return; // msg not deleted
		}

		case RADIO_CONTROL_MESSAGE:{
			handleRadioControlMessage(msg);
			return; // msg not deleted
		}

		case MAC_CONTROL_COMMAND:{
			toMacLayer(msg);
			return; // msg not deleted
		}

		case RADIO_CONTROL_COMMAND:{
			toMacLayer(msg);
			return; // msg not deleted
		}

		case NETWORK_CONTROL_COMMAND:{
			handleNetworkControlCommand(msg);
			break;
		}

		case OUT_OF_ENERGY:{
			disabled = true;
			for (int i=0; i<5; i++) trace1() << "OUT_OF_ENERGY";
			deathTime = SIMTIME_DBL(simTime());
			isPaused = true;
			GlobalLocationService::removeNode(self);
			for (int i=0; i<5; i++) trace1() << "removed";
			setTimer(RESUME_APPLICATION, 10);

			for (int i=0; i<GlobalLocationService::numNodes; i++) {
				if (i == self) continue;
				cModule* desModule = getParentModule()->getParentModule()->getParentModule()->getSubmodule("node", i)
					->getSubmodule("Communication")->getSubmodule("Routing");
				sendDirect(new cMessage("Destroy node message", REMOVE_NODE), desModule, "fromDirect");
			}
			break;
		}

		case RESUME_APPLICATION:{
			isPaused = false;
			break;
		}

		case REMOVE_NODE:{
			if (!disabled) handleRemoveNodeMessage(msg);
			break;
		}

		case DESTROY_NODE:{
			disabled = true;
			break;
		}

		default:{
			opp_error("Network module recieved unexpected message: [%s]", msg->getName());
		}
	}

	delete msg;
}

// handleMacControlMessage needs to either process and DELETE the message OR forward it
void VirtualRouting::handleMacControlMessage(cMessage * msg)
{
	toApplicationLayer(msg);
}

// handleRadioControlMessage needs to either process and DELETE the message OR forward it
void VirtualRouting::handleRadioControlMessage(cMessage * msg)
{
	toApplicationLayer(msg);
}

void VirtualRouting::finish()
{
//  if (self == 801 || self == 0)
//    log() << resMgrModule->getSpentEnergy() << " days " << self;

  trace1() << "WSN_EVENT STATISTICS " << "id:" << self << " totalPacketReceived:" << numPacketReceived << " "
    << "estimateLifetime:" << deathTime << " x:" << selfLocation.x() << " y:" << selfLocation.y()
    << " sumHopRatio:" << sumRatioEndCount << " maxRatio:" << maxRatio << " sumDistanceRatio:" << sumDistanceEndCount << " endPointCount:" << endCount
    << " energyConsumed:" << resMgrModule->getSpentEnergy();

	CastaliaModule::finish();
	cPacket *pkt;
	// clear the buffer from all remaining packets
	while (!TXBuffer.empty()) {
		pkt = TXBuffer.front();
		TXBuffer.pop();
		cancelAndDelete(pkt);
	}
}

int VirtualRouting::bufferPacket(cPacket * rcvFrame)
{
	if ((int)TXBuffer.size() >= netBufferSize) {
		collectOutput("Buffer overflow");
		cancelAndDelete(rcvFrame);
		return 0;
	} else {
		TXBuffer.push(rcvFrame);
		trace() << "Packet buffered from application layer, buffer state: " <<
				TXBuffer.size() << "/" << netBufferSize;
		return 1;
	}
}

int VirtualRouting::resolveNetworkAddress(const char *netAddr)
{
	if (!netAddr[0] || netAddr[0] < '0' || netAddr[0] > '9')
		return BROADCAST_MAC_ADDRESS;
	return atoi(netAddr);
}

bool VirtualRouting::isNotDuplicatePacket(cPacket * pkt)
{
	//extract source address and sequence number from the packet
	RoutingPacket *netPkt = check_and_cast <RoutingPacket*>(pkt);
	int src = resolveNetworkAddress(netPkt->getSource());
	unsigned int sn = netPkt->getSequenceNumber();

	//resize packet history vector if necessary
	if (src >= (int)pktHistory.size())
		pktHistory.resize(src+1);

	//search for this sequence number in the list, corresponding to address 'src'
	list<unsigned int>::iterator it1;
	for (it1 = pktHistory[src].begin(); it1 != pktHistory[src].end(); it1++) {
		//if such sequence number is found, packet is duplicate
		if (*it1 == sn) return false;
	}

	//no such sequence number found, this is new packet
	pktHistory[src].push_front(sn);
	if (pktHistory[src].size() > PACKET_HISTORY_SIZE)
		pktHistory[src].pop_back();
	return true;
}

void VirtualRouting::debugLine(double x1, double y1, double x2, double y2, string color) {
  trace() << "WSN_EVENT DRAW LINE " << x1 << " " << y1 << " " << x2 << " " << y2 << " " << color;
}
void VirtualRouting::debugLine(Point p1, Point p2, string color) {
  debugLine(p1.x(), p1.y(), p2.x(), p2.y(), color);
}
void VirtualRouting::debugCircle(double centerX, double centerY, double radius, string color) {
  trace() << "WSN_EVENT DRAW CIRCLE " << centerX << " " << centerY << " " << radius << " " << color;
}
void VirtualRouting::debugCircle(Point center, double radius, string color) {
  debugCircle(center.x(), center.y(), radius, color);
}
void VirtualRouting::debugPoint(double x1, double x2, string color) {
  trace() << "WSN_EVENT DRAW POINT " << x1 << " " << x2 << " " << color;
}
void VirtualRouting::debugPoint(Point p, string color) {
  debugPoint(p.x(), p.y(), color);
}

void VirtualRouting::debugArc(Point from, Point to, double radius, string color) {
  trace() << "WSN_EVENT DRAW ARC " << from.x() << " " << from.y() << " " << to.x() << " "
    << to.y() << " " << radius << " " << color;
}

void VirtualRouting::debugPolygon(vector<Point> ps, string color) {
  for (int i = 0; i < ps.size(); i++) {
    Point from = ps[i];
    Point to = ps[(i + 1) % ps.size()];
    debugLine(from, to, color);
  }
}

void VirtualRouting::debugPath(vector<Point> ps, string color) {
  for (int i = 0; i < ps.size() - 1; i++) {
    Point from = ps[i];
    Point to = ps[(i + 1) % ps.size()];
    debugLine(from, to, color);
  }
}

bool VirtualRouting::reached(Point location) {
  for (auto n: neighborTable) {
    if (G::distance(n.location, location) < G::distance(selfLocation, location)) {
      return false;
    }
  }

  return G::distance(selfLocation, location) < RADIO_RANGE;
}

int VirtualRouting::getRandomNumber(int from, int to) {
//    srand(time(0));
//    int result = from + rand() % (to - from + 1);
//    return result;
    int result = intrand(to - from + 1) + from;
    return result;
}
