//
// Generated file, do not edit! Created by nedtool 4.6 from src/node/communication/routing/mlpRouting/MlpRoutingPacket.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "MlpRoutingPacket_m.h"

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
    cEnum *e = cEnum::find("MlpPacketDef");
    if (!e) enums.getInstance()->add(e = new cEnum("MlpPacketDef"));
    e->insert(MLP_DATA_PACKET, "MLP_DATA_PACKET");
);

EXECUTE_ON_STARTUP(
    cEnum *e = cEnum::find("MLPForwardingMode");
    if (!e) enums.getInstance()->add(e = new cEnum("MLPForwardingMode"));
    e->insert(MLP_GREEDY_ROUTING, "MLP_GREEDY_ROUTING");
    e->insert(MLP_ROLLINGBALL_ROUTING, "MLP_ROLLINGBALL_ROUTING");
);

Register_Class(MlpPacket);

MlpPacket::MlpPacket(const char *name, int kind) : ::RoutingPacket(name,kind)
{
    this->packetId_var = 0;
    this->previousId_var = 0;
    this->MlpPacketKind_var = 0;
    this->routingMode_var = 0;
    this->nextStoppingPlaceId_var = 0;
    this->outDelta_var = 99999;
    this->aroundHoleRadius_var = 99999;
    this->inDelta_var = 99999;
}

MlpPacket::MlpPacket(const MlpPacket& other) : ::RoutingPacket(other)
{
    copy(other);
}

MlpPacket::~MlpPacket()
{
}

MlpPacket& MlpPacket::operator=(const MlpPacket& other)
{
    if (this==&other) return *this;
    ::RoutingPacket::operator=(other);
    copy(other);
    return *this;
}

void MlpPacket::copy(const MlpPacket& other)
{
    this->packetId_var = other.packetId_var;
    this->previousId_var = other.previousId_var;
    this->MlpPacketKind_var = other.MlpPacketKind_var;
    this->destLocation_var = other.destLocation_var;
    this->sourceLocation_var = other.sourceLocation_var;
    this->ballCenter_var = other.ballCenter_var;
    this->stuckLocation_var = other.stuckLocation_var;
    this->routingMode_var = other.routingMode_var;
    this->previousStoppingPlace_var = other.previousStoppingPlace_var;
    this->nextStoppingPlace_var = other.nextStoppingPlace_var;
    this->nextStoppingPlaceId_var = other.nextStoppingPlaceId_var;
    this->path_var = other.path_var;
    this->startStableLocation_var = other.startStableLocation_var;
    this->outDelta_var = other.outDelta_var;
    this->aroundHoleRadius_var = other.aroundHoleRadius_var;
    this->inDelta_var = other.inDelta_var;
}

void MlpPacket::parsimPack(cCommBuffer *b)
{
    ::RoutingPacket::parsimPack(b);
    doPacking(b,this->packetId_var);
    doPacking(b,this->previousId_var);
    doPacking(b,this->MlpPacketKind_var);
    doPacking(b,this->destLocation_var);
    doPacking(b,this->sourceLocation_var);
    doPacking(b,this->ballCenter_var);
    doPacking(b,this->stuckLocation_var);
    doPacking(b,this->routingMode_var);
    doPacking(b,this->previousStoppingPlace_var);
    doPacking(b,this->nextStoppingPlace_var);
    doPacking(b,this->nextStoppingPlaceId_var);
    doPacking(b,this->path_var);
    doPacking(b,this->startStableLocation_var);
    doPacking(b,this->outDelta_var);
    doPacking(b,this->aroundHoleRadius_var);
    doPacking(b,this->inDelta_var);
}

void MlpPacket::parsimUnpack(cCommBuffer *b)
{
    ::RoutingPacket::parsimUnpack(b);
    doUnpacking(b,this->packetId_var);
    doUnpacking(b,this->previousId_var);
    doUnpacking(b,this->MlpPacketKind_var);
    doUnpacking(b,this->destLocation_var);
    doUnpacking(b,this->sourceLocation_var);
    doUnpacking(b,this->ballCenter_var);
    doUnpacking(b,this->stuckLocation_var);
    doUnpacking(b,this->routingMode_var);
    doUnpacking(b,this->previousStoppingPlace_var);
    doUnpacking(b,this->nextStoppingPlace_var);
    doUnpacking(b,this->nextStoppingPlaceId_var);
    doUnpacking(b,this->path_var);
    doUnpacking(b,this->startStableLocation_var);
    doUnpacking(b,this->outDelta_var);
    doUnpacking(b,this->aroundHoleRadius_var);
    doUnpacking(b,this->inDelta_var);
}

int MlpPacket::getPacketId() const
{
    return packetId_var;
}

void MlpPacket::setPacketId(int packetId)
{
    this->packetId_var = packetId;
}

int MlpPacket::getPreviousId() const
{
    return previousId_var;
}

void MlpPacket::setPreviousId(int previousId)
{
    this->previousId_var = previousId;
}

int MlpPacket::getMlpPacketKind() const
{
    return MlpPacketKind_var;
}

void MlpPacket::setMlpPacketKind(int MlpPacketKind)
{
    this->MlpPacketKind_var = MlpPacketKind;
}

Point& MlpPacket::getDestLocation()
{
    return destLocation_var;
}

void MlpPacket::setDestLocation(const Point& destLocation)
{
    this->destLocation_var = destLocation;
}

Point& MlpPacket::getSourceLocation()
{
    return sourceLocation_var;
}

void MlpPacket::setSourceLocation(const Point& sourceLocation)
{
    this->sourceLocation_var = sourceLocation;
}

Point& MlpPacket::getBallCenter()
{
    return ballCenter_var;
}

void MlpPacket::setBallCenter(const Point& ballCenter)
{
    this->ballCenter_var = ballCenter;
}

Point& MlpPacket::getStuckLocation()
{
    return stuckLocation_var;
}

void MlpPacket::setStuckLocation(const Point& stuckLocation)
{
    this->stuckLocation_var = stuckLocation;
}

int MlpPacket::getRoutingMode() const
{
    return routingMode_var;
}

void MlpPacket::setRoutingMode(int routingMode)
{
    this->routingMode_var = routingMode;
}

Point& MlpPacket::getPreviousStoppingPlace()
{
    return previousStoppingPlace_var;
}

void MlpPacket::setPreviousStoppingPlace(const Point& previousStoppingPlace)
{
    this->previousStoppingPlace_var = previousStoppingPlace;
}

Point& MlpPacket::getNextStoppingPlace()
{
    return nextStoppingPlace_var;
}

void MlpPacket::setNextStoppingPlace(const Point& nextStoppingPlace)
{
    this->nextStoppingPlace_var = nextStoppingPlace;
}

int MlpPacket::getNextStoppingPlaceId() const
{
    return nextStoppingPlaceId_var;
}

void MlpPacket::setNextStoppingPlaceId(int nextStoppingPlaceId)
{
    this->nextStoppingPlaceId_var = nextStoppingPlaceId;
}

PointVector& MlpPacket::getPath()
{
    return path_var;
}

void MlpPacket::setPath(const PointVector& path)
{
    this->path_var = path;
}

Point& MlpPacket::getStartStableLocation()
{
    return startStableLocation_var;
}

void MlpPacket::setStartStableLocation(const Point& startStableLocation)
{
    this->startStableLocation_var = startStableLocation;
}

int MlpPacket::getOutDelta() const
{
    return outDelta_var;
}

void MlpPacket::setOutDelta(int outDelta)
{
    this->outDelta_var = outDelta;
}

double MlpPacket::getAroundHoleRadius() const
{
    return aroundHoleRadius_var;
}

void MlpPacket::setAroundHoleRadius(double aroundHoleRadius)
{
    this->aroundHoleRadius_var = aroundHoleRadius;
}

int MlpPacket::getInDelta() const
{
    return inDelta_var;
}

void MlpPacket::setInDelta(int inDelta)
{
    this->inDelta_var = inDelta;
}

class MlpPacketDescriptor : public cClassDescriptor
{
  public:
    MlpPacketDescriptor();
    virtual ~MlpPacketDescriptor();

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

Register_ClassDescriptor(MlpPacketDescriptor);

MlpPacketDescriptor::MlpPacketDescriptor() : cClassDescriptor("MlpPacket", "RoutingPacket")
{
}

MlpPacketDescriptor::~MlpPacketDescriptor()
{
}

bool MlpPacketDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<MlpPacket *>(obj)!=NULL;
}

const char *MlpPacketDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int MlpPacketDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 16+basedesc->getFieldCount(object) : 16;
}

unsigned int MlpPacketDescriptor::getFieldTypeFlags(void *object, int field) const
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
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<16) ? fieldTypeFlags[field] : 0;
}

const char *MlpPacketDescriptor::getFieldName(void *object, int field) const
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
        "MlpPacketKind",
        "destLocation",
        "sourceLocation",
        "ballCenter",
        "stuckLocation",
        "routingMode",
        "previousStoppingPlace",
        "nextStoppingPlace",
        "nextStoppingPlaceId",
        "path",
        "startStableLocation",
        "outDelta",
        "aroundHoleRadius",
        "inDelta",
    };
    return (field>=0 && field<16) ? fieldNames[field] : NULL;
}

int MlpPacketDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='p' && strcmp(fieldName, "packetId")==0) return base+0;
    if (fieldName[0]=='p' && strcmp(fieldName, "previousId")==0) return base+1;
    if (fieldName[0]=='M' && strcmp(fieldName, "MlpPacketKind")==0) return base+2;
    if (fieldName[0]=='d' && strcmp(fieldName, "destLocation")==0) return base+3;
    if (fieldName[0]=='s' && strcmp(fieldName, "sourceLocation")==0) return base+4;
    if (fieldName[0]=='b' && strcmp(fieldName, "ballCenter")==0) return base+5;
    if (fieldName[0]=='s' && strcmp(fieldName, "stuckLocation")==0) return base+6;
    if (fieldName[0]=='r' && strcmp(fieldName, "routingMode")==0) return base+7;
    if (fieldName[0]=='p' && strcmp(fieldName, "previousStoppingPlace")==0) return base+8;
    if (fieldName[0]=='n' && strcmp(fieldName, "nextStoppingPlace")==0) return base+9;
    if (fieldName[0]=='n' && strcmp(fieldName, "nextStoppingPlaceId")==0) return base+10;
    if (fieldName[0]=='p' && strcmp(fieldName, "path")==0) return base+11;
    if (fieldName[0]=='s' && strcmp(fieldName, "startStableLocation")==0) return base+12;
    if (fieldName[0]=='o' && strcmp(fieldName, "outDelta")==0) return base+13;
    if (fieldName[0]=='a' && strcmp(fieldName, "aroundHoleRadius")==0) return base+14;
    if (fieldName[0]=='i' && strcmp(fieldName, "inDelta")==0) return base+15;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *MlpPacketDescriptor::getFieldTypeString(void *object, int field) const
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
        "int",
        "PointVector",
        "Point",
        "int",
        "double",
        "int",
    };
    return (field>=0 && field<16) ? fieldTypeStrings[field] : NULL;
}

const char *MlpPacketDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldProperty(object, field, propertyname);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 2:
            if (!strcmp(propertyname,"enum")) return "MlpPacketDef";
            return NULL;
        case 7:
            if (!strcmp(propertyname,"enum")) return "MLPForwardingMode";
            return NULL;
        default: return NULL;
    }
}

int MlpPacketDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    MlpPacket *pp = (MlpPacket *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string MlpPacketDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    MlpPacket *pp = (MlpPacket *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getPacketId());
        case 1: return long2string(pp->getPreviousId());
        case 2: return long2string(pp->getMlpPacketKind());
        case 3: {std::stringstream out; out << pp->getDestLocation(); return out.str();}
        case 4: {std::stringstream out; out << pp->getSourceLocation(); return out.str();}
        case 5: {std::stringstream out; out << pp->getBallCenter(); return out.str();}
        case 6: {std::stringstream out; out << pp->getStuckLocation(); return out.str();}
        case 7: return long2string(pp->getRoutingMode());
        case 8: {std::stringstream out; out << pp->getPreviousStoppingPlace(); return out.str();}
        case 9: {std::stringstream out; out << pp->getNextStoppingPlace(); return out.str();}
        case 10: return long2string(pp->getNextStoppingPlaceId());
        case 11: {std::stringstream out; out << pp->getPath(); return out.str();}
        case 12: {std::stringstream out; out << pp->getStartStableLocation(); return out.str();}
        case 13: return long2string(pp->getOutDelta());
        case 14: return double2string(pp->getAroundHoleRadius());
        case 15: return long2string(pp->getInDelta());
        default: return "";
    }
}

bool MlpPacketDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    MlpPacket *pp = (MlpPacket *)object; (void)pp;
    switch (field) {
        case 0: pp->setPacketId(string2long(value)); return true;
        case 1: pp->setPreviousId(string2long(value)); return true;
        case 2: pp->setMlpPacketKind(string2long(value)); return true;
        case 7: pp->setRoutingMode(string2long(value)); return true;
        case 10: pp->setNextStoppingPlaceId(string2long(value)); return true;
        case 13: pp->setOutDelta(string2long(value)); return true;
        case 14: pp->setAroundHoleRadius(string2double(value)); return true;
        case 15: pp->setInDelta(string2long(value)); return true;
        default: return false;
    }
}

const char *MlpPacketDescriptor::getFieldStructName(void *object, int field) const
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
        case 11: return opp_typename(typeid(PointVector));
        case 12: return opp_typename(typeid(Point));
        default: return NULL;
    };
}

void *MlpPacketDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    MlpPacket *pp = (MlpPacket *)object; (void)pp;
    switch (field) {
        case 3: return (void *)(&pp->getDestLocation()); break;
        case 4: return (void *)(&pp->getSourceLocation()); break;
        case 5: return (void *)(&pp->getBallCenter()); break;
        case 6: return (void *)(&pp->getStuckLocation()); break;
        case 8: return (void *)(&pp->getPreviousStoppingPlace()); break;
        case 9: return (void *)(&pp->getNextStoppingPlace()); break;
        case 11: return (void *)(&pp->getPath()); break;
        case 12: return (void *)(&pp->getStartStableLocation()); break;
        default: return NULL;
    }
}


