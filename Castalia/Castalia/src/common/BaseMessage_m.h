//
// Generated file, do not edit! Created by nedtool 4.6 from src/common/BaseMessage.msg.
//

#ifndef _WSN_BASEMESSAGE_M_H_
#define _WSN_BASEMESSAGE_M_H_

#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0406
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif


namespace wsn {

/**
 * Class generated from <tt>src/common/BaseMessage.msg:21</tt> by nedtool.
 * <pre>
 * //
 * // The purpose of this packet is to log packets's ID into trace file
 * // because the packet ID of omnet++ simulator is unclear
 * //
 * packet BaseMessage
 * {
 *     string packetID;
 *     int hopCount;
 * }
 * </pre>
 */
class BaseMessage : public ::cPacket
{
  protected:
    opp_string packetID_var;
    int hopCount_var;

  private:
    void copy(const BaseMessage& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const BaseMessage&);

  public:
    BaseMessage(const char *name=NULL, int kind=0);
    BaseMessage(const BaseMessage& other);
    virtual ~BaseMessage();
    BaseMessage& operator=(const BaseMessage& other);
    virtual BaseMessage *dup() const {return new BaseMessage(*this);}
    virtual void parsimPack(cCommBuffer *b);
    virtual void parsimUnpack(cCommBuffer *b);

    // field getter/setter methods
    virtual const char * getPacketID() const;
    virtual void setPacketID(const char * packetID);
    virtual int getHopCount() const;
    virtual void setHopCount(int hopCount);
};

inline void doPacking(cCommBuffer *b, BaseMessage& obj) {obj.parsimPack(b);}
inline void doUnpacking(cCommBuffer *b, BaseMessage& obj) {obj.parsimUnpack(b);}

} // namespace wsn

#endif // ifndef _WSN_BASEMESSAGE_M_H_
