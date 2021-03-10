//
// Generated file, do not edit! Created by nedtool 4.6 from src/node/communication/routing/RoutingPacket.msg.
//

#ifndef _ROUTINGPACKET_M_H_
#define _ROUTINGPACKET_M_H_

#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0406
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



/**
 * Struct generated from src/node/communication/routing/RoutingPacket.msg:21 by nedtool.
 */
struct NetMacInfoExchange_type
{
    NetMacInfoExchange_type();
    double RSSI;
    double LQI;
    int nextHop;
    int lastHop;
};

void doPacking(cCommBuffer *b, NetMacInfoExchange_type& a);
void doUnpacking(cCommBuffer *b, NetMacInfoExchange_type& a);

/**
 * Class generated from <tt>src/node/communication/routing/RoutingPacket.msg:32</tt> by nedtool.
 * <pre>
 * packet RoutingPacket
 * {
 *     NetMacInfoExchange_type netMacInfoExchange;
 * 
 *     int TTL = 10000;
 *     int hopCount = 0;
 *     double distanceCount = 0;
 *     bool isDataPacket = false;
 *     string source;
 *     string destination;
 *     unsigned int sequenceNumber;
 * }
 * </pre>
 */
class RoutingPacket : public ::cPacket
{
  protected:
    NetMacInfoExchange_type netMacInfoExchange_var;
    int TTL_var;
    int hopCount_var;
    double distanceCount_var;
    bool isDataPacket_var;
    opp_string source_var;
    opp_string destination_var;
    unsigned int sequenceNumber_var;

  private:
    void copy(const RoutingPacket& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const RoutingPacket&);

  public:
    RoutingPacket(const char *name=NULL, int kind=0);
    RoutingPacket(const RoutingPacket& other);
    virtual ~RoutingPacket();
    RoutingPacket& operator=(const RoutingPacket& other);
    virtual RoutingPacket *dup() const {return new RoutingPacket(*this);}
    virtual void parsimPack(cCommBuffer *b);
    virtual void parsimUnpack(cCommBuffer *b);

    // field getter/setter methods
    virtual NetMacInfoExchange_type& getNetMacInfoExchange();
    virtual const NetMacInfoExchange_type& getNetMacInfoExchange() const {return const_cast<RoutingPacket*>(this)->getNetMacInfoExchange();}
    virtual void setNetMacInfoExchange(const NetMacInfoExchange_type& netMacInfoExchange);
    virtual int getTTL() const;
    virtual void setTTL(int TTL);
    virtual int getHopCount() const;
    virtual void setHopCount(int hopCount);
    virtual double getDistanceCount() const;
    virtual void setDistanceCount(double distanceCount);
    virtual bool getIsDataPacket() const;
    virtual void setIsDataPacket(bool isDataPacket);
    virtual const char * getSource() const;
    virtual void setSource(const char * source);
    virtual const char * getDestination() const;
    virtual void setDestination(const char * destination);
    virtual unsigned int getSequenceNumber() const;
    virtual void setSequenceNumber(unsigned int sequenceNumber);
};

inline void doPacking(cCommBuffer *b, RoutingPacket& obj) {obj.parsimPack(b);}
inline void doUnpacking(cCommBuffer *b, RoutingPacket& obj) {obj.parsimUnpack(b);}


#endif // ifndef _ROUTINGPACKET_M_H_

