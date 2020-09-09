//
// Generated file, do not edit! Created by nedtool 4.6 from src/node/communication/routing/boundhole/BoundHoleMessage.msg.
//

#ifndef _BOUNDHOLEMESSAGE_M_H_
#define _BOUNDHOLEMESSAGE_M_H_

#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0406
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



// cplusplus {{
    #include "GeoMathHelper.h"
    #include "RoutingPacket_m.h"
    #include <vector>
    typedef std::vector<int> NodeVector;
// }}

/**
 * Enum generated from <tt>src/node/communication/routing/boundhole/BoundHoleMessage.msg:27</tt> by nedtool.
 * <pre>
 * enum BoundHoleType
 * {
 * 
 *     BOUNDHOLE_BOUNDHOLE = 0x01;
 *     BOUNDHOLE_REFRESH = 0x02;
 * }
 * </pre>
 */
enum BoundHoleType {
    BOUNDHOLE_BOUNDHOLE = 0x01,
    BOUNDHOLE_REFRESH = 0x02
};

/**
 * Class generated from <tt>src/node/communication/routing/boundhole/BoundHoleMessage.msg:32</tt> by nedtool.
 * <pre>
 * packet BoundHolePacket extends RoutingPacket
 * {
 *     int sourceId;
 *     int destinationId;
 *     int packetId;
 *     int type;
 *     Point previousNode;
 *     int index; // for refresh type only
 *     NodeVector boundholeNodes;
 * }
 * </pre>
 */
class BoundHolePacket : public ::RoutingPacket
{
  protected:
    int sourceId_var;
    int destinationId_var;
    int packetId_var;
    int type_var;
    Point previousNode_var;
    int index_var;
    NodeVector boundholeNodes_var;

  private:
    void copy(const BoundHolePacket& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const BoundHolePacket&);

  public:
    BoundHolePacket(const char *name=NULL, int kind=0);
    BoundHolePacket(const BoundHolePacket& other);
    virtual ~BoundHolePacket();
    BoundHolePacket& operator=(const BoundHolePacket& other);
    virtual BoundHolePacket *dup() const {return new BoundHolePacket(*this);}
    virtual void parsimPack(cCommBuffer *b);
    virtual void parsimUnpack(cCommBuffer *b);

    // field getter/setter methods
    virtual int getSourceId() const;
    virtual void setSourceId(int sourceId);
    virtual int getDestinationId() const;
    virtual void setDestinationId(int destinationId);
    virtual int getPacketId() const;
    virtual void setPacketId(int packetId);
    virtual int getType() const;
    virtual void setType(int type);
    virtual Point& getPreviousNode();
    virtual const Point& getPreviousNode() const {return const_cast<BoundHolePacket*>(this)->getPreviousNode();}
    virtual void setPreviousNode(const Point& previousNode);
    virtual int getIndex() const;
    virtual void setIndex(int index);
    virtual NodeVector& getBoundholeNodes();
    virtual const NodeVector& getBoundholeNodes() const {return const_cast<BoundHolePacket*>(this)->getBoundholeNodes();}
    virtual void setBoundholeNodes(const NodeVector& boundholeNodes);
};

inline void doPacking(cCommBuffer *b, BoundHolePacket& obj) {obj.parsimPack(b);}
inline void doUnpacking(cCommBuffer *b, BoundHolePacket& obj) {obj.parsimUnpack(b);}


#endif // ifndef _BOUNDHOLEMESSAGE_M_H_

