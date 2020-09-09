//
// Generated file, do not edit! Created by nedtool 4.6 from src/node/communication/routing/stableRouting/StableRoutingPacket.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "StableRoutingPacket_m.h"

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
    cEnum *e = cEnum::find("StablePacketDef");
    if (!e) enums.getInstance()->add(e = new cEnum("StablePacketDef"));
    e->insert(STABLE_DATA_PACKET, "STABLE_DATA_PACKET");
);

EXECUTE_ON_STARTUP(
    cEnum *e = cEnum::find("RollingBallForwardingMode");
    if (!e) enums.getInstance()->add(e = new cEnum("RollingBallForwardingMode"));
    e->insert(GREEDY_ROUTING, "GREEDY_ROUTING");
    e->insert(ROLLINGBALL_ROUTING, "ROLLINGBALL_ROUTING");
);

Register_Class(StablePacket);

StablePacket::StablePacket(const char *name, int kind) : ::RoutingPacket(name,kind)
{
    this->packetId_var = 0;
    this->previousId_var = 0;
    this->StablePacketKind_var = 0;
    this->routingMode_var = 0;
    this->outCavernRadius_var = -1;
    this->aroundHoleRadius_var = -1;
    this->inCavernRadius_var = -1;
}

StablePacket::StablePacket(const StablePacket& other) : ::RoutingPacket(other)
{
    copy(other);
}

StablePacket::~StablePacket()
{
}

StablePacket& StablePacket::operator=(const StablePacket& other)
{
    if (this==&other) return *this;
    ::RoutingPacket::operator=(other);
    copy(other);
    return *this;
}

void StablePacket::copy(const StablePacket& other)
{
    this->packetId_var = other.packetId_var;
    this->previousId_var = other.previousId_var;
    this->StablePacketKind_var = other.StablePacketKind_var;
    this->destLocation_var = other.destLocation_var;
    this->sourceLocation_var = other.sourceLocation_var;
    this->ballCenter_var = other.ballCenter_var;
    this->stuckLocation_var = other.stuckLocation_var;
    this->routingMode_var = other.routingMode_var;
    this->nextStoppingPlace_var = other.nextStoppingPlace_var;
    this->startStableLocation_var = other.startStableLocation_var;
    this->outCavernRadius_var = other.outCavernRadius_var;
    this->aroundHoleRadius_var = other.aroundHoleRadius_var;
    this->inCavernRadius_var = other.inCavernRadius_var;
}

void StablePacket::parsimPack(cCommBuffer *b)
{
    ::RoutingPacket::parsimPack(b);
    doPacking(b,this->packetId_var);
    doPacking(b,this->previousId_var);
    doPacking(b,this->StablePacketKind_var);
    doPacking(b,this->destLocation_var);
    doPacking(b,this->sourceLocation_var);
    doPacking(b,this->ballCenter_var);
    doPacking(b,this->stuckLocation_var);
    doPacking(b,this->routingMode_var);
    doPacking(b,this->nextStoppingPlace_var);
    doPacking(b,this->startStableLocation_var);
    doPacking(b,this->outCavernRadius_var);
    doPacking(b,this->aroundHoleRadius_var);
    doPacking(b,this->inCavernRadius_var);
}

void StablePacket::parsimUnpack(cCommBuffer *b)
{
    ::RoutingPacket::parsimUnpack(b);
    doUnpacking(b,this->packetId_var);
    doUnpacking(b,this->previousId_var);
    doUnpacking(b,this->StablePacketKind_var);
    doUnpacking(b,this->destLocation_var);
    doUnpacking(b,this->sourceLocation_var);
    doUnpacking(b,this->ballCenter_var);
    doUnpacking(b,this->stuckLocation_var);
    doUnpacking(b,this->routingMode_var);
    doUnpacking(b,this->nextStoppingPlace_var);
    doUnpacking(b,this->startStableLocation_var);
    doUnpacking(b,this->outCavernRadius_var);
    doUnpacking(b,this->aroundHoleRadius_var);
    doUnpacking(b,this->inCavernRadius_var);
}

int StablePacket::getPacketId() const
{
    return packetId_var;
}

void StablePacket::setPacketId(int packetId)
{
    this->packetId_var = packetId;
}

int StablePacket::getPreviousId() const
{
    return previousId_var;
}

void StablePacket::setPreviousId(int previousId)
{
    this->previousId_var = previousId;
}

int StablePacket::getStablePacketKind() const
{
    return StablePacketKind_var;
}

void StablePacket::setStablePacketKind(int StablePacketKind)
{
    this->StablePacketKind_var = StablePacketKind;
}

Point& StablePacket::getDestLocation()
{
    return destLocation_var;
}

void StablePacket::setDestLocation(const Point& destLocation)
{
    this->destLocation_var = destLocation;
}

Point& StablePacket::getSourceLocation()
{
    return sourceLocation_var;
}

void StablePacket::setSourceLocation(const Point& sourceLocation)
{
    this->sourceLocation_var = sourceLocation;
}

Point& StablePacket::getBallCenter()
{
    return ballCenter_var;
}

void StablePacket::setBallCenter(const Point& ballCenter)
{
    this->ballCenter_var = ballCenter;
}

Point& StablePacket::getStuckLocation()
{
    return stuckLocation_var;
}

void StablePacket::setStuckLocation(const Point& stuckLocation)
{
    this->stuckLocation_var = stuckLocation;
}

int StablePacket::getRoutingMode() const
{
    return routingMode_var;
}

void StablePacket::setRoutingMode(int routingMode)
{
    this->routingMode_var = routingMode;
}

Point& StablePacket::getNextStoppingPlace()
{
    return nextStoppingPlace_var;
}

void StablePacket::setNextStoppingPlace(const Point& nextStoppingPlace)
{
    this->nextStoppingPlace_var = nextStoppingPlace;
}

Point& StablePacket::getStartStableLocation()
{
    return startStableLocation_var;
}

void StablePacket::setStartStableLocation(const Point& startStableLocation)
{
    this->startStableLocation_var = startStableLocation;
}

double StablePacket::getOutCavernRadius() const
{
    return outCavernRadius_var;
}

void StablePacket::setOutCavernRadius(double outCavernRadius)
{
    this->outCavernRadius_var = outCavernRadius;
}

double StablePacket::getAroundHoleRadius() const
{
    return aroundHoleRadius_var;
}

void StablePacket::setAroundHoleRadius(double aroundHoleRadius)
{
    this->aroundHoleRadius_var = aroundHoleRadius;
}

double StablePacket::getInCavernRadius() const
{
    return inCavernRadius_var;
}

void StablePacket::setInCavernRadius(double inCavernRadius)
{
    this->inCavernRadius_var = inCavernRadius;
}

class StablePacketDescriptor : public cClassDescriptor
{
  public:
    StablePacketDescriptor();
    virtual ~StablePacketDescriptor();

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

Register_ClassDescriptor(StablePacketDescriptor);

StablePacketDescriptor::StablePacketDescriptor() : cClassDescriptor("StablePacket", "RoutingPacket")
{
}

StablePacketDescriptor::~StablePacketDescriptor()
{
}

bool StablePacketDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<StablePacket *>(obj)!=NULL;
}

const char *StablePacketDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int StablePacketDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 13+basedesc->getFieldCount(object) : 13;
}

unsigned int StablePacketDescriptor::getFieldTypeFlags(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeFlags(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISEDITABLE,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<13) ? fieldTypeFlags[field] : 0;
}

const char *StablePacketDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "packetId",
        "previousId",
        "StablePacketKind",
        "destLocation",
        "sourceLocation",
        "ballCenter",
        "stuckLocation",
        "routingMode",
        "nextStoppingPlace",
        "startStableLocation",
        "outCavernRadius",
        "aroundHoleRadius",
        "inCavernRadius",
    };
    return (field>=0 && field<13) ? fieldNames[field] : NULL;
}

int StablePacketDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='p' && strcmp(fieldName, "packetId")==0) return base+0;
    if (fieldName[0]=='p' && strcmp(fieldName, "previousId")==0) return base+1;
    if (fieldName[0]=='S' && strcmp(fieldName, "StablePacketKind")==0) return base+2;
    if (fieldName[0]=='d' && strcmp(fieldName, "destLocation")==0) return base+3;
    if (fieldName[0]=='s' && strcmp(fieldName, "sourceLocation")==0) return base+4;
    if (fieldName[0]=='b' && strcmp(fieldName, "ballCenter")==0) return base+5;
    if (fieldName[0]=='s' && strcmp(fieldName, "stuckLocation")==0) return base+6;
    if (fieldName[0]=='r' && strcmp(fieldName, "routingMode")==0) return base+7;
    if (fieldName[0]=='n' && strcmp(fieldName, "nextStoppingPlace")==0) return base+8;
    if (fieldName[0]=='s' && strcmp(fieldName, "startStableLocation")==0) return base+9;
    if (fieldName[0]=='o' && strcmp(fieldName, "outCavernRadius")==0) return base+10;
    if (fieldName[0]=='a' && strcmp(fieldName, "aroundHoleRadius")==0) return base+11;
    if (fieldName[0]=='i' && strcmp(fieldName, "inCavernRadius")==0) return base+12;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *StablePacketDescriptor::getFieldTypeString(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeString(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldTypeStrings[] = {
        "int",
        "int",
        "int",
        "Point",
        "Point",
        "Point",
        "Point",
        "int",
        "Point",
        "Point",
        "double",
        "double",
        "double",
    };
    return (field>=0 && field<13) ? fieldTypeStrings[field] : NULL;
}

const char *StablePacketDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldProperty(object, field, propertyname);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 2:
            if (!strcmp(propertyname,"enum")) return "StablePacketDef";
            return NULL;
        case 7:
            if (!strcmp(propertyname,"enum")) return "RollingBallForwardingMode";
            return NULL;
        default: return NULL;
    }
}

int StablePacketDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    StablePacket *pp = (StablePacket *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string StablePacketDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    StablePacket *pp = (StablePacket *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getPacketId());
        case 1: return long2string(pp->getPreviousId());
        case 2: return long2string(pp->getStablePacketKind());
        case 3: {std::stringstream out; out << pp->getDestLocation(); return out.str();}
        case 4: {std::stringstream out; out << pp->getSourceLocation(); return out.str();}
        case 5: {std::stringstream out; out << pp->getBallCenter(); return out.str();}
        case 6: {std::stringstream out; out << pp->getStuckLocation(); return out.str();}
        case 7: return long2string(pp->getRoutingMode());
        case 8: {std::stringstream out; out << pp->getNextStoppingPlace(); return out.str();}
        case 9: {std::stringstream out; out << pp->getStartStableLocation(); return out.str();}
        case 10: return double2string(pp->getOutCavernRadius());
        case 11: return double2string(pp->getAroundHoleRadius());
        case 12: return double2string(pp->getInCavernRadius());
        default: return "";
    }
}

bool StablePacketDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    StablePacket *pp = (StablePacket *)object; (void)pp;
    switch (field) {
        case 0: pp->setPacketId(string2long(value)); return true;
        case 1: pp->setPreviousId(string2long(value)); return true;
        case 2: pp->setStablePacketKind(string2long(value)); return true;
        case 7: pp->setRoutingMode(string2long(value)); return true;
        case 10: pp->setOutCavernRadius(string2double(value)); return true;
        case 11: pp->setAroundHoleRadius(string2double(value)); return true;
        case 12: pp->setInCavernRadius(string2double(value)); return true;
        default: return false;
    }
}

const char *StablePacketDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 3: return opp_typename(typeid(Point));
        case 4: return opp_typename(typeid(Point));
        case 5: return opp_typename(typeid(Point));
        case 6: return opp_typename(typeid(Point));
        case 8: return opp_typename(typeid(Point));
        case 9: return opp_typename(typeid(Point));
        default: return NULL;
    };
}

void *StablePacketDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    StablePacket *pp = (StablePacket *)object; (void)pp;
    switch (field) {
        case 3: return (void *)(&pp->getDestLocation()); break;
        case 4: return (void *)(&pp->getSourceLocation()); break;
        case 5: return (void *)(&pp->getBallCenter()); break;
        case 6: return (void *)(&pp->getStuckLocation()); break;
        case 8: return (void *)(&pp->getNextStoppingPlace()); break;
        case 9: return (void *)(&pp->getStartStableLocation()); break;
        default: return NULL;
    }
}

Register_Class(DiscoverHolePacket);

DiscoverHolePacket::DiscoverHolePacket(const char *name, int kind) : ::RoutingPacket(name,kind)
{
    this->originatorId_var = 0;
    this->previousId_var = 0;
    this->path_var = 0;
}

DiscoverHolePacket::DiscoverHolePacket(const DiscoverHolePacket& other) : ::RoutingPacket(other)
{
    copy(other);
}

DiscoverHolePacket::~DiscoverHolePacket()
{
}

DiscoverHolePacket& DiscoverHolePacket::operator=(const DiscoverHolePacket& other)
{
    if (this==&other) return *this;
    ::RoutingPacket::operator=(other);
    copy(other);
    return *this;
}

void DiscoverHolePacket::copy(const DiscoverHolePacket& other)
{
    this->originatorId_var = other.originatorId_var;
    this->ballCenter_var = other.ballCenter_var;
    this->previousId_var = other.previousId_var;
    this->path_var = other.path_var;
    this->caverns_var = other.caverns_var;
    this->hole_var = other.hole_var;
}

void DiscoverHolePacket::parsimPack(cCommBuffer *b)
{
    ::RoutingPacket::parsimPack(b);
    doPacking(b,this->originatorId_var);
    doPacking(b,this->ballCenter_var);
    doPacking(b,this->previousId_var);
    doPacking(b,this->path_var);
    doPacking(b,this->caverns_var);
    doPacking(b,this->hole_var);
}

void DiscoverHolePacket::parsimUnpack(cCommBuffer *b)
{
    ::RoutingPacket::parsimUnpack(b);
    doUnpacking(b,this->originatorId_var);
    doUnpacking(b,this->ballCenter_var);
    doUnpacking(b,this->previousId_var);
    doUnpacking(b,this->path_var);
    doUnpacking(b,this->caverns_var);
    doUnpacking(b,this->hole_var);
}

int DiscoverHolePacket::getOriginatorId() const
{
    return originatorId_var;
}

void DiscoverHolePacket::setOriginatorId(int originatorId)
{
    this->originatorId_var = originatorId;
}

Point& DiscoverHolePacket::getBallCenter()
{
    return ballCenter_var;
}

void DiscoverHolePacket::setBallCenter(const Point& ballCenter)
{
    this->ballCenter_var = ballCenter;
}

int DiscoverHolePacket::getPreviousId() const
{
    return previousId_var;
}

void DiscoverHolePacket::setPreviousId(int previousId)
{
    this->previousId_var = previousId;
}

const char * DiscoverHolePacket::getPath() const
{
    return path_var.c_str();
}

void DiscoverHolePacket::setPath(const char * path)
{
    this->path_var = path;
}

CavernVector& DiscoverHolePacket::getCaverns()
{
    return caverns_var;
}

void DiscoverHolePacket::setCaverns(const CavernVector& caverns)
{
    this->caverns_var = caverns;
}

PointVector& DiscoverHolePacket::getHole()
{
    return hole_var;
}

void DiscoverHolePacket::setHole(const PointVector& hole)
{
    this->hole_var = hole;
}

class DiscoverHolePacketDescriptor : public cClassDescriptor
{
  public:
    DiscoverHolePacketDescriptor();
    virtual ~DiscoverHolePacketDescriptor();

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

Register_ClassDescriptor(DiscoverHolePacketDescriptor);

DiscoverHolePacketDescriptor::DiscoverHolePacketDescriptor() : cClassDescriptor("DiscoverHolePacket", "RoutingPacket")
{
}

DiscoverHolePacketDescriptor::~DiscoverHolePacketDescriptor()
{
}

bool DiscoverHolePacketDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<DiscoverHolePacket *>(obj)!=NULL;
}

const char *DiscoverHolePacketDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int DiscoverHolePacketDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 6+basedesc->getFieldCount(object) : 6;
}

unsigned int DiscoverHolePacketDescriptor::getFieldTypeFlags(void *object, int field) const
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
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
    };
    return (field>=0 && field<6) ? fieldTypeFlags[field] : 0;
}

const char *DiscoverHolePacketDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "originatorId",
        "ballCenter",
        "previousId",
        "path",
        "caverns",
        "hole",
    };
    return (field>=0 && field<6) ? fieldNames[field] : NULL;
}

int DiscoverHolePacketDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='o' && strcmp(fieldName, "originatorId")==0) return base+0;
    if (fieldName[0]=='b' && strcmp(fieldName, "ballCenter")==0) return base+1;
    if (fieldName[0]=='p' && strcmp(fieldName, "previousId")==0) return base+2;
    if (fieldName[0]=='p' && strcmp(fieldName, "path")==0) return base+3;
    if (fieldName[0]=='c' && strcmp(fieldName, "caverns")==0) return base+4;
    if (fieldName[0]=='h' && strcmp(fieldName, "hole")==0) return base+5;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *DiscoverHolePacketDescriptor::getFieldTypeString(void *object, int field) const
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
        "int",
        "string",
        "CavernVector",
        "PointVector",
    };
    return (field>=0 && field<6) ? fieldTypeStrings[field] : NULL;
}

const char *DiscoverHolePacketDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldProperty(object, field, propertyname);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        default: return NULL;
    }
}

int DiscoverHolePacketDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    DiscoverHolePacket *pp = (DiscoverHolePacket *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string DiscoverHolePacketDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    DiscoverHolePacket *pp = (DiscoverHolePacket *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getOriginatorId());
        case 1: {std::stringstream out; out << pp->getBallCenter(); return out.str();}
        case 2: return long2string(pp->getPreviousId());
        case 3: return oppstring2string(pp->getPath());
        case 4: {std::stringstream out; out << pp->getCaverns(); return out.str();}
        case 5: {std::stringstream out; out << pp->getHole(); return out.str();}
        default: return "";
    }
}

bool DiscoverHolePacketDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    DiscoverHolePacket *pp = (DiscoverHolePacket *)object; (void)pp;
    switch (field) {
        case 0: pp->setOriginatorId(string2long(value)); return true;
        case 2: pp->setPreviousId(string2long(value)); return true;
        case 3: pp->setPath((value)); return true;
        default: return false;
    }
}

const char *DiscoverHolePacketDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 1: return opp_typename(typeid(Point));
        case 4: return opp_typename(typeid(CavernVector));
        case 5: return opp_typename(typeid(PointVector));
        default: return NULL;
    };
}

void *DiscoverHolePacketDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    DiscoverHolePacket *pp = (DiscoverHolePacket *)object; (void)pp;
    switch (field) {
        case 1: return (void *)(&pp->getBallCenter()); break;
        case 4: return (void *)(&pp->getCaverns()); break;
        case 5: return (void *)(&pp->getHole()); break;
        default: return NULL;
    }
}


