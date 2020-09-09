//
// Generated file, do not edit! Created by nedtool 4.6 from src/node/communication/routing/holevicinityrouting/ICHMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "ICHMessage_m.h"

USING_NAMESPACE


// Another default rule (prevents compiler from choosing base class' doPacking())
template<typename T>
void doPacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doPacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}

template<typename T>
void doUnpacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doUnpacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}




// Template rule for outputting std::vector<T> types
template<typename T, typename A>
inline std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec)
{
    out.put('{');
    for(typename std::vector<T,A>::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        if (it != vec.begin()) {
            out.put(','); out.put(' ');
        }
        out << *it;
    }
    out.put('}');
    
    char buf[32];
    sprintf(buf, " (size=%u)", (unsigned int)vec.size());
    out.write(buf, strlen(buf));
    return out;
}

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
inline std::ostream& operator<<(std::ostream& out,const T&) {return out;}

EXECUTE_ON_STARTUP(
    cEnum *e = cEnum::find("RollingBallForwardingMode");
    if (!e) enums.getInstance()->add(e = new cEnum("RollingBallForwardingMode"));
    e->insert(GREEDY_ROUTING, "GREEDY_ROUTING");
    e->insert(ROLLINGBALL_ROUTING, "ROLLINGBALL_ROUTING");
);

EXECUTE_ON_STARTUP(
    cEnum *e = cEnum::find("RollingBallPacketDef");
    if (!e) enums.getInstance()->add(e = new cEnum("RollingBallPacketDef"));
    e->insert(ROLLINGBALL_DATA_PACKET, "ROLLINGBALL_DATA_PACKET");
);

Register_Class(ICHDataPacket);

ICHDataPacket::ICHDataPacket(const char *name, int kind) : ::RoutingPacket(name,kind)
{
    this->packetId_var = 0;
    this->apIndex_var = 0;
    this->scaleFactor_var = 0;
    this->RollingBallPacketKind_var = 0;
    this->routingMode_var = 0;
}

ICHDataPacket::ICHDataPacket(const ICHDataPacket& other) : ::RoutingPacket(other)
{
    copy(other);
}

ICHDataPacket::~ICHDataPacket()
{
}

ICHDataPacket& ICHDataPacket::operator=(const ICHDataPacket& other)
{
    if (this==&other) return *this;
    ::RoutingPacket::operator=(other);
    copy(other);
    return *this;
}

void ICHDataPacket::copy(const ICHDataPacket& other)
{
    this->packetId_var = other.packetId_var;
    this->sourceLocation_var = other.sourceLocation_var;
    this->destLocation_var = other.destLocation_var;
    this->anchor_var = other.anchor_var;
    this->apIndex_var = other.apIndex_var;
    this->homotheticCenter_var = other.homotheticCenter_var;
    this->homotheticArray_var = other.homotheticArray_var;
    this->scaleFactor_var = other.scaleFactor_var;
    this->scaleFactorArray_var = other.scaleFactorArray_var;
    this->shortestPath_var = other.shortestPath_var;
    this->RollingBallPacketKind_var = other.RollingBallPacketKind_var;
    this->routingMode_var = other.routingMode_var;
    this->stuckLocation_var = other.stuckLocation_var;
    this->ballCenter_var = other.ballCenter_var;
    this->previousLocation_var = other.previousLocation_var;
}

void ICHDataPacket::parsimPack(cCommBuffer *b)
{
    ::RoutingPacket::parsimPack(b);
    doPacking(b,this->packetId_var);
    doPacking(b,this->sourceLocation_var);
    doPacking(b,this->destLocation_var);
    doPacking(b,this->anchor_var);
    doPacking(b,this->apIndex_var);
    doPacking(b,this->homotheticCenter_var);
    doPacking(b,this->homotheticArray_var);
    doPacking(b,this->scaleFactor_var);
    doPacking(b,this->scaleFactorArray_var);
    doPacking(b,this->shortestPath_var);
    doPacking(b,this->RollingBallPacketKind_var);
    doPacking(b,this->routingMode_var);
    doPacking(b,this->stuckLocation_var);
    doPacking(b,this->ballCenter_var);
    doPacking(b,this->previousLocation_var);
}

void ICHDataPacket::parsimUnpack(cCommBuffer *b)
{
    ::RoutingPacket::parsimUnpack(b);
    doUnpacking(b,this->packetId_var);
    doUnpacking(b,this->sourceLocation_var);
    doUnpacking(b,this->destLocation_var);
    doUnpacking(b,this->anchor_var);
    doUnpacking(b,this->apIndex_var);
    doUnpacking(b,this->homotheticCenter_var);
    doUnpacking(b,this->homotheticArray_var);
    doUnpacking(b,this->scaleFactor_var);
    doUnpacking(b,this->scaleFactorArray_var);
    doUnpacking(b,this->shortestPath_var);
    doUnpacking(b,this->RollingBallPacketKind_var);
    doUnpacking(b,this->routingMode_var);
    doUnpacking(b,this->stuckLocation_var);
    doUnpacking(b,this->ballCenter_var);
    doUnpacking(b,this->previousLocation_var);
}

int ICHDataPacket::getPacketId() const
{
    return packetId_var;
}

void ICHDataPacket::setPacketId(int packetId)
{
    this->packetId_var = packetId;
}

Point& ICHDataPacket::getSourceLocation()
{
    return sourceLocation_var;
}

void ICHDataPacket::setSourceLocation(const Point& sourceLocation)
{
    this->sourceLocation_var = sourceLocation;
}

Point& ICHDataPacket::getDestLocation()
{
    return destLocation_var;
}

void ICHDataPacket::setDestLocation(const Point& destLocation)
{
    this->destLocation_var = destLocation;
}

Point& ICHDataPacket::getAnchor()
{
    return anchor_var;
}

void ICHDataPacket::setAnchor(const Point& anchor)
{
    this->anchor_var = anchor;
}

unsigned int ICHDataPacket::getApIndex() const
{
    return apIndex_var;
}

void ICHDataPacket::setApIndex(unsigned int apIndex)
{
    this->apIndex_var = apIndex;
}

Point& ICHDataPacket::getHomotheticCenter()
{
    return homotheticCenter_var;
}

void ICHDataPacket::setHomotheticCenter(const Point& homotheticCenter)
{
    this->homotheticCenter_var = homotheticCenter;
}

PointVector& ICHDataPacket::getHomotheticArray()
{
    return homotheticArray_var;
}

void ICHDataPacket::setHomotheticArray(const PointVector& homotheticArray)
{
    this->homotheticArray_var = homotheticArray;
}

double ICHDataPacket::getScaleFactor() const
{
    return scaleFactor_var;
}

void ICHDataPacket::setScaleFactor(double scaleFactor)
{
    this->scaleFactor_var = scaleFactor;
}

doubleVector& ICHDataPacket::getScaleFactorArray()
{
    return scaleFactorArray_var;
}

void ICHDataPacket::setScaleFactorArray(const doubleVector& scaleFactorArray)
{
    this->scaleFactorArray_var = scaleFactorArray;
}

PointVector& ICHDataPacket::getShortestPath()
{
    return shortestPath_var;
}

void ICHDataPacket::setShortestPath(const PointVector& shortestPath)
{
    this->shortestPath_var = shortestPath;
}

int ICHDataPacket::getRollingBallPacketKind() const
{
    return RollingBallPacketKind_var;
}

void ICHDataPacket::setRollingBallPacketKind(int RollingBallPacketKind)
{
    this->RollingBallPacketKind_var = RollingBallPacketKind;
}

int ICHDataPacket::getRoutingMode() const
{
    return routingMode_var;
}

void ICHDataPacket::setRoutingMode(int routingMode)
{
    this->routingMode_var = routingMode;
}

Point& ICHDataPacket::getStuckLocation()
{
    return stuckLocation_var;
}

void ICHDataPacket::setStuckLocation(const Point& stuckLocation)
{
    this->stuckLocation_var = stuckLocation;
}

Point& ICHDataPacket::getBallCenter()
{
    return ballCenter_var;
}

void ICHDataPacket::setBallCenter(const Point& ballCenter)
{
    this->ballCenter_var = ballCenter;
}

Point& ICHDataPacket::getPreviousLocation()
{
    return previousLocation_var;
}

void ICHDataPacket::setPreviousLocation(const Point& previousLocation)
{
    this->previousLocation_var = previousLocation;
}

class ICHDataPacketDescriptor : public cClassDescriptor
{
  public:
    ICHDataPacketDescriptor();
    virtual ~ICHDataPacketDescriptor();

    virtual bool doesSupport(cObject *obj) const;
    virtual const char *getProperty(const char *propertyname) const;
    virtual int getFieldCount(void *object) const;
    virtual const char *getFieldName(void *object, int field) const;
    virtual int findField(void *object, const char *fieldName) const;
    virtual unsigned int getFieldTypeFlags(void *object, int field) const;
    virtual const char *getFieldTypeString(void *object, int field) const;
    virtual const char *getFieldProperty(void *object, int field, const char *propertyname) const;
    virtual int getArraySize(void *object, int field) const;

    virtual std::string getFieldAsString(void *object, int field, int i) const;
    virtual bool setFieldAsString(void *object, int field, int i, const char *value) const;

    virtual const char *getFieldStructName(void *object, int field) const;
    virtual void *getFieldStructPointer(void *object, int field, int i) const;
};

Register_ClassDescriptor(ICHDataPacketDescriptor);

ICHDataPacketDescriptor::ICHDataPacketDescriptor() : cClassDescriptor("ICHDataPacket", "RoutingPacket")
{
}

ICHDataPacketDescriptor::~ICHDataPacketDescriptor()
{
}

bool ICHDataPacketDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<ICHDataPacket *>(obj)!=NULL;
}

const char *ICHDataPacketDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int ICHDataPacketDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 15+basedesc->getFieldCount(object) : 15;
}

unsigned int ICHDataPacketDescriptor::getFieldTypeFlags(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeFlags(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISEDITABLE,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISEDITABLE,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
    };
    return (field>=0 && field<15) ? fieldTypeFlags[field] : 0;
}

const char *ICHDataPacketDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "packetId",
        "sourceLocation",
        "destLocation",
        "anchor",
        "apIndex",
        "homotheticCenter",
        "homotheticArray",
        "scaleFactor",
        "scaleFactorArray",
        "shortestPath",
        "RollingBallPacketKind",
        "routingMode",
        "stuckLocation",
        "ballCenter",
        "previousLocation",
    };
    return (field>=0 && field<15) ? fieldNames[field] : NULL;
}

int ICHDataPacketDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='p' && strcmp(fieldName, "packetId")==0) return base+0;
    if (fieldName[0]=='s' && strcmp(fieldName, "sourceLocation")==0) return base+1;
    if (fieldName[0]=='d' && strcmp(fieldName, "destLocation")==0) return base+2;
    if (fieldName[0]=='a' && strcmp(fieldName, "anchor")==0) return base+3;
    if (fieldName[0]=='a' && strcmp(fieldName, "apIndex")==0) return base+4;
    if (fieldName[0]=='h' && strcmp(fieldName, "homotheticCenter")==0) return base+5;
    if (fieldName[0]=='h' && strcmp(fieldName, "homotheticArray")==0) return base+6;
    if (fieldName[0]=='s' && strcmp(fieldName, "scaleFactor")==0) return base+7;
    if (fieldName[0]=='s' && strcmp(fieldName, "scaleFactorArray")==0) return base+8;
    if (fieldName[0]=='s' && strcmp(fieldName, "shortestPath")==0) return base+9;
    if (fieldName[0]=='R' && strcmp(fieldName, "RollingBallPacketKind")==0) return base+10;
    if (fieldName[0]=='r' && strcmp(fieldName, "routingMode")==0) return base+11;
    if (fieldName[0]=='s' && strcmp(fieldName, "stuckLocation")==0) return base+12;
    if (fieldName[0]=='b' && strcmp(fieldName, "ballCenter")==0) return base+13;
    if (fieldName[0]=='p' && strcmp(fieldName, "previousLocation")==0) return base+14;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *ICHDataPacketDescriptor::getFieldTypeString(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeString(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldTypeStrings[] = {
        "int",
        "Point",
        "Point",
        "Point",
        "unsigned int",
        "Point",
        "PointVector",
        "double",
        "doubleVector",
        "PointVector",
        "int",
        "int",
        "Point",
        "Point",
        "Point",
    };
    return (field>=0 && field<15) ? fieldTypeStrings[field] : NULL;
}

const char *ICHDataPacketDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldProperty(object, field, propertyname);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 10:
            if (!strcmp(propertyname,"enum")) return "RollingBallPacketDef";
            return NULL;
        case 11:
            if (!strcmp(propertyname,"enum")) return "RollingBallForwardingMode";
            return NULL;
        default: return NULL;
    }
}

int ICHDataPacketDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    ICHDataPacket *pp = (ICHDataPacket *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string ICHDataPacketDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    ICHDataPacket *pp = (ICHDataPacket *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getPacketId());
        case 1: {std::stringstream out; out << pp->getSourceLocation(); return out.str();}
        case 2: {std::stringstream out; out << pp->getDestLocation(); return out.str();}
        case 3: {std::stringstream out; out << pp->getAnchor(); return out.str();}
        case 4: return ulong2string(pp->getApIndex());
        case 5: {std::stringstream out; out << pp->getHomotheticCenter(); return out.str();}
        case 6: {std::stringstream out; out << pp->getHomotheticArray(); return out.str();}
        case 7: return double2string(pp->getScaleFactor());
        case 8: {std::stringstream out; out << pp->getScaleFactorArray(); return out.str();}
        case 9: {std::stringstream out; out << pp->getShortestPath(); return out.str();}
        case 10: return long2string(pp->getRollingBallPacketKind());
        case 11: return long2string(pp->getRoutingMode());
        case 12: {std::stringstream out; out << pp->getStuckLocation(); return out.str();}
        case 13: {std::stringstream out; out << pp->getBallCenter(); return out.str();}
        case 14: {std::stringstream out; out << pp->getPreviousLocation(); return out.str();}
        default: return "";
    }
}

bool ICHDataPacketDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    ICHDataPacket *pp = (ICHDataPacket *)object; (void)pp;
    switch (field) {
        case 0: pp->setPacketId(string2long(value)); return true;
        case 4: pp->setApIndex(string2ulong(value)); return true;
        case 7: pp->setScaleFactor(string2double(value)); return true;
        case 10: pp->setRollingBallPacketKind(string2long(value)); return true;
        case 11: pp->setRoutingMode(string2long(value)); return true;
        default: return false;
    }
}

const char *ICHDataPacketDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 1: return opp_typename(typeid(Point));
        case 2: return opp_typename(typeid(Point));
        case 3: return opp_typename(typeid(Point));
        case 5: return opp_typename(typeid(Point));
        case 6: return opp_typename(typeid(PointVector));
        case 8: return opp_typename(typeid(doubleVector));
        case 9: return opp_typename(typeid(PointVector));
        case 12: return opp_typename(typeid(Point));
        case 13: return opp_typename(typeid(Point));
        case 14: return opp_typename(typeid(Point));
        default: return NULL;
    };
}

void *ICHDataPacketDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    ICHDataPacket *pp = (ICHDataPacket *)object; (void)pp;
    switch (field) {
        case 1: return (void *)(&pp->getSourceLocation()); break;
        case 2: return (void *)(&pp->getDestLocation()); break;
        case 3: return (void *)(&pp->getAnchor()); break;
        case 5: return (void *)(&pp->getHomotheticCenter()); break;
        case 6: return (void *)(&pp->getHomotheticArray()); break;
        case 8: return (void *)(&pp->getScaleFactorArray()); break;
        case 9: return (void *)(&pp->getShortestPath()); break;
        case 12: return (void *)(&pp->getStuckLocation()); break;
        case 13: return (void *)(&pp->getBallCenter()); break;
        case 14: return (void *)(&pp->getPreviousLocation()); break;
        default: return NULL;
    }
}


