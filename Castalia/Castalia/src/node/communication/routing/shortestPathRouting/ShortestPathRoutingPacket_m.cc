//
// Generated file, do not edit! Created by nedtool 4.6 from src/node/communication/routing/shortestPathRouting/ShortestPathRoutingPacket.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "ShortestPathRoutingPacket_m.h"

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
    cEnum *e = cEnum::find("ShortestPathPacketDef");
    if (!e) enums.getInstance()->add(e = new cEnum("ShortestPathPacketDef"));
    e->insert(SHORTEST_PATH_DATA_PACKET, "SHORTEST_PATH_DATA_PACKET");
);

EXECUTE_ON_STARTUP(
    cEnum *e = cEnum::find("RollingBallForwardingMode");
    if (!e) enums.getInstance()->add(e = new cEnum("RollingBallForwardingMode"));
    e->insert(GREEDY_ROUTING, "GREEDY_ROUTING");
    e->insert(ROLLINGBALL_ROUTING, "ROLLINGBALL_ROUTING");
);

Register_Class(ShortestPathRoutingPacket);

ShortestPathRoutingPacket::ShortestPathRoutingPacket(const char *name, int kind) : ::RoutingPacket(name,kind)
{
    this->packetId_var = 0;
    this->previousId_var = 0;
    this->ShortestPathPacketKind_var = 0;
    this->routingMode_var = 0;
}

ShortestPathRoutingPacket::ShortestPathRoutingPacket(const ShortestPathRoutingPacket& other) : ::RoutingPacket(other)
{
    copy(other);
}

ShortestPathRoutingPacket::~ShortestPathRoutingPacket()
{
}

ShortestPathRoutingPacket& ShortestPathRoutingPacket::operator=(const ShortestPathRoutingPacket& other)
{
    if (this==&other) return *this;
    ::RoutingPacket::operator=(other);
    copy(other);
    return *this;
}

void ShortestPathRoutingPacket::copy(const ShortestPathRoutingPacket& other)
{
    this->packetId_var = other.packetId_var;
    this->previousId_var = other.previousId_var;
    this->ShortestPathPacketKind_var = other.ShortestPathPacketKind_var;
    this->destLocation_var = other.destLocation_var;
    this->sourceLocation_var = other.sourceLocation_var;
    this->ballCenter_var = other.ballCenter_var;
    this->stuckLocation_var = other.stuckLocation_var;
    this->routingMode_var = other.routingMode_var;
    this->nextStoppingPlace_var = other.nextStoppingPlace_var;
    this->startShortestPathLocation_var = other.startShortestPathLocation_var;
}

void ShortestPathRoutingPacket::parsimPack(cCommBuffer *b)
{
    ::RoutingPacket::parsimPack(b);
    doPacking(b,this->packetId_var);
    doPacking(b,this->previousId_var);
    doPacking(b,this->ShortestPathPacketKind_var);
    doPacking(b,this->destLocation_var);
    doPacking(b,this->sourceLocation_var);
    doPacking(b,this->ballCenter_var);
    doPacking(b,this->stuckLocation_var);
    doPacking(b,this->routingMode_var);
    doPacking(b,this->nextStoppingPlace_var);
    doPacking(b,this->startShortestPathLocation_var);
}

void ShortestPathRoutingPacket::parsimUnpack(cCommBuffer *b)
{
    ::RoutingPacket::parsimUnpack(b);
    doUnpacking(b,this->packetId_var);
    doUnpacking(b,this->previousId_var);
    doUnpacking(b,this->ShortestPathPacketKind_var);
    doUnpacking(b,this->destLocation_var);
    doUnpacking(b,this->sourceLocation_var);
    doUnpacking(b,this->ballCenter_var);
    doUnpacking(b,this->stuckLocation_var);
    doUnpacking(b,this->routingMode_var);
    doUnpacking(b,this->nextStoppingPlace_var);
    doUnpacking(b,this->startShortestPathLocation_var);
}

int ShortestPathRoutingPacket::getPacketId() const
{
    return packetId_var;
}

void ShortestPathRoutingPacket::setPacketId(int packetId)
{
    this->packetId_var = packetId;
}

int ShortestPathRoutingPacket::getPreviousId() const
{
    return previousId_var;
}

void ShortestPathRoutingPacket::setPreviousId(int previousId)
{
    this->previousId_var = previousId;
}

int ShortestPathRoutingPacket::getShortestPathPacketKind() const
{
    return ShortestPathPacketKind_var;
}

void ShortestPathRoutingPacket::setShortestPathPacketKind(int ShortestPathPacketKind)
{
    this->ShortestPathPacketKind_var = ShortestPathPacketKind;
}

Point& ShortestPathRoutingPacket::getDestLocation()
{
    return destLocation_var;
}

void ShortestPathRoutingPacket::setDestLocation(const Point& destLocation)
{
    this->destLocation_var = destLocation;
}

Point& ShortestPathRoutingPacket::getSourceLocation()
{
    return sourceLocation_var;
}

void ShortestPathRoutingPacket::setSourceLocation(const Point& sourceLocation)
{
    this->sourceLocation_var = sourceLocation;
}

Point& ShortestPathRoutingPacket::getBallCenter()
{
    return ballCenter_var;
}

void ShortestPathRoutingPacket::setBallCenter(const Point& ballCenter)
{
    this->ballCenter_var = ballCenter;
}

Point& ShortestPathRoutingPacket::getStuckLocation()
{
    return stuckLocation_var;
}

void ShortestPathRoutingPacket::setStuckLocation(const Point& stuckLocation)
{
    this->stuckLocation_var = stuckLocation;
}

int ShortestPathRoutingPacket::getRoutingMode() const
{
    return routingMode_var;
}

void ShortestPathRoutingPacket::setRoutingMode(int routingMode)
{
    this->routingMode_var = routingMode;
}

Point& ShortestPathRoutingPacket::getNextStoppingPlace()
{
    return nextStoppingPlace_var;
}

void ShortestPathRoutingPacket::setNextStoppingPlace(const Point& nextStoppingPlace)
{
    this->nextStoppingPlace_var = nextStoppingPlace;
}

Point& ShortestPathRoutingPacket::getStartShortestPathLocation()
{
    return startShortestPathLocation_var;
}

void ShortestPathRoutingPacket::setStartShortestPathLocation(const Point& startShortestPathLocation)
{
    this->startShortestPathLocation_var = startShortestPathLocation;
}

class ShortestPathRoutingPacketDescriptor : public cClassDescriptor
{
  public:
    ShortestPathRoutingPacketDescriptor();
    virtual ~ShortestPathRoutingPacketDescriptor();

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

Register_ClassDescriptor(ShortestPathRoutingPacketDescriptor);

ShortestPathRoutingPacketDescriptor::ShortestPathRoutingPacketDescriptor() : cClassDescriptor("ShortestPathRoutingPacket", "RoutingPacket")
{
}

ShortestPathRoutingPacketDescriptor::~ShortestPathRoutingPacketDescriptor()
{
}

bool ShortestPathRoutingPacketDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<ShortestPathRoutingPacket *>(obj)!=NULL;
}

const char *ShortestPathRoutingPacketDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int ShortestPathRoutingPacketDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 10+basedesc->getFieldCount(object) : 10;
}

unsigned int ShortestPathRoutingPacketDescriptor::getFieldTypeFlags(void *object, int field) const
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
    };
    return (field>=0 && field<10) ? fieldTypeFlags[field] : 0;
}

const char *ShortestPathRoutingPacketDescriptor::getFieldName(void *object, int field) const
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
        "ShortestPathPacketKind",
        "destLocation",
        "sourceLocation",
        "ballCenter",
        "stuckLocation",
        "routingMode",
        "nextStoppingPlace",
        "startShortestPathLocation",
    };
    return (field>=0 && field<10) ? fieldNames[field] : NULL;
}

int ShortestPathRoutingPacketDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='p' && strcmp(fieldName, "packetId")==0) return base+0;
    if (fieldName[0]=='p' && strcmp(fieldName, "previousId")==0) return base+1;
    if (fieldName[0]=='S' && strcmp(fieldName, "ShortestPathPacketKind")==0) return base+2;
    if (fieldName[0]=='d' && strcmp(fieldName, "destLocation")==0) return base+3;
    if (fieldName[0]=='s' && strcmp(fieldName, "sourceLocation")==0) return base+4;
    if (fieldName[0]=='b' && strcmp(fieldName, "ballCenter")==0) return base+5;
    if (fieldName[0]=='s' && strcmp(fieldName, "stuckLocation")==0) return base+6;
    if (fieldName[0]=='r' && strcmp(fieldName, "routingMode")==0) return base+7;
    if (fieldName[0]=='n' && strcmp(fieldName, "nextStoppingPlace")==0) return base+8;
    if (fieldName[0]=='s' && strcmp(fieldName, "startShortestPathLocation")==0) return base+9;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *ShortestPathRoutingPacketDescriptor::getFieldTypeString(void *object, int field) const
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
    };
    return (field>=0 && field<10) ? fieldTypeStrings[field] : NULL;
}

const char *ShortestPathRoutingPacketDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldProperty(object, field, propertyname);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 2:
            if (!strcmp(propertyname,"enum")) return "ShortestPathPacketDef";
            return NULL;
        case 7:
            if (!strcmp(propertyname,"enum")) return "RollingBallForwardingMode";
            return NULL;
        default: return NULL;
    }
}

int ShortestPathRoutingPacketDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    ShortestPathRoutingPacket *pp = (ShortestPathRoutingPacket *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string ShortestPathRoutingPacketDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    ShortestPathRoutingPacket *pp = (ShortestPathRoutingPacket *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getPacketId());
        case 1: return long2string(pp->getPreviousId());
        case 2: return long2string(pp->getShortestPathPacketKind());
        case 3: {std::stringstream out; out << pp->getDestLocation(); return out.str();}
        case 4: {std::stringstream out; out << pp->getSourceLocation(); return out.str();}
        case 5: {std::stringstream out; out << pp->getBallCenter(); return out.str();}
        case 6: {std::stringstream out; out << pp->getStuckLocation(); return out.str();}
        case 7: return long2string(pp->getRoutingMode());
        case 8: {std::stringstream out; out << pp->getNextStoppingPlace(); return out.str();}
        case 9: {std::stringstream out; out << pp->getStartShortestPathLocation(); return out.str();}
        default: return "";
    }
}

bool ShortestPathRoutingPacketDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    ShortestPathRoutingPacket *pp = (ShortestPathRoutingPacket *)object; (void)pp;
    switch (field) {
        case 0: pp->setPacketId(string2long(value)); return true;
        case 1: pp->setPreviousId(string2long(value)); return true;
        case 2: pp->setShortestPathPacketKind(string2long(value)); return true;
        case 7: pp->setRoutingMode(string2long(value)); return true;
        default: return false;
    }
}

const char *ShortestPathRoutingPacketDescriptor::getFieldStructName(void *object, int field) const
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

void *ShortestPathRoutingPacketDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    ShortestPathRoutingPacket *pp = (ShortestPathRoutingPacket *)object; (void)pp;
    switch (field) {
        case 3: return (void *)(&pp->getDestLocation()); break;
        case 4: return (void *)(&pp->getSourceLocation()); break;
        case 5: return (void *)(&pp->getBallCenter()); break;
        case 6: return (void *)(&pp->getStuckLocation()); break;
        case 8: return (void *)(&pp->getNextStoppingPlace()); break;
        case 9: return (void *)(&pp->getStartShortestPathLocation()); break;
        default: return NULL;
    }
}

Register_Class(ShortestPathDiscoverHolePacket);

ShortestPathDiscoverHolePacket::ShortestPathDiscoverHolePacket(const char *name, int kind) : ::RoutingPacket(name,kind)
{
    this->originatorId_var = 0;
    this->previousId_var = 0;
    this->path_var = 0;
}

ShortestPathDiscoverHolePacket::ShortestPathDiscoverHolePacket(const ShortestPathDiscoverHolePacket& other) : ::RoutingPacket(other)
{
    copy(other);
}

ShortestPathDiscoverHolePacket::~ShortestPathDiscoverHolePacket()
{
}

ShortestPathDiscoverHolePacket& ShortestPathDiscoverHolePacket::operator=(const ShortestPathDiscoverHolePacket& other)
{
    if (this==&other) return *this;
    ::RoutingPacket::operator=(other);
    copy(other);
    return *this;
}

void ShortestPathDiscoverHolePacket::copy(const ShortestPathDiscoverHolePacket& other)
{
    this->originatorId_var = other.originatorId_var;
    this->ballCenter_var = other.ballCenter_var;
    this->previousId_var = other.previousId_var;
    this->path_var = other.path_var;
}

void ShortestPathDiscoverHolePacket::parsimPack(cCommBuffer *b)
{
    ::RoutingPacket::parsimPack(b);
    doPacking(b,this->originatorId_var);
    doPacking(b,this->ballCenter_var);
    doPacking(b,this->previousId_var);
    doPacking(b,this->path_var);
}

void ShortestPathDiscoverHolePacket::parsimUnpack(cCommBuffer *b)
{
    ::RoutingPacket::parsimUnpack(b);
    doUnpacking(b,this->originatorId_var);
    doUnpacking(b,this->ballCenter_var);
    doUnpacking(b,this->previousId_var);
    doUnpacking(b,this->path_var);
}

int ShortestPathDiscoverHolePacket::getOriginatorId() const
{
    return originatorId_var;
}

void ShortestPathDiscoverHolePacket::setOriginatorId(int originatorId)
{
    this->originatorId_var = originatorId;
}

Point& ShortestPathDiscoverHolePacket::getBallCenter()
{
    return ballCenter_var;
}

void ShortestPathDiscoverHolePacket::setBallCenter(const Point& ballCenter)
{
    this->ballCenter_var = ballCenter;
}

int ShortestPathDiscoverHolePacket::getPreviousId() const
{
    return previousId_var;
}

void ShortestPathDiscoverHolePacket::setPreviousId(int previousId)
{
    this->previousId_var = previousId;
}

const char * ShortestPathDiscoverHolePacket::getPath() const
{
    return path_var.c_str();
}

void ShortestPathDiscoverHolePacket::setPath(const char * path)
{
    this->path_var = path;
}

class ShortestPathDiscoverHolePacketDescriptor : public cClassDescriptor
{
  public:
    ShortestPathDiscoverHolePacketDescriptor();
    virtual ~ShortestPathDiscoverHolePacketDescriptor();

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

Register_ClassDescriptor(ShortestPathDiscoverHolePacketDescriptor);

ShortestPathDiscoverHolePacketDescriptor::ShortestPathDiscoverHolePacketDescriptor() : cClassDescriptor("ShortestPathDiscoverHolePacket", "RoutingPacket")
{
}

ShortestPathDiscoverHolePacketDescriptor::~ShortestPathDiscoverHolePacketDescriptor()
{
}

bool ShortestPathDiscoverHolePacketDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<ShortestPathDiscoverHolePacket *>(obj)!=NULL;
}

const char *ShortestPathDiscoverHolePacketDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int ShortestPathDiscoverHolePacketDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 4+basedesc->getFieldCount(object) : 4;
}

unsigned int ShortestPathDiscoverHolePacketDescriptor::getFieldTypeFlags(void *object, int field) const
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
    };
    return (field>=0 && field<4) ? fieldTypeFlags[field] : 0;
}

const char *ShortestPathDiscoverHolePacketDescriptor::getFieldName(void *object, int field) const
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
    };
    return (field>=0 && field<4) ? fieldNames[field] : NULL;
}

int ShortestPathDiscoverHolePacketDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='o' && strcmp(fieldName, "originatorId")==0) return base+0;
    if (fieldName[0]=='b' && strcmp(fieldName, "ballCenter")==0) return base+1;
    if (fieldName[0]=='p' && strcmp(fieldName, "previousId")==0) return base+2;
    if (fieldName[0]=='p' && strcmp(fieldName, "path")==0) return base+3;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *ShortestPathDiscoverHolePacketDescriptor::getFieldTypeString(void *object, int field) const
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
    };
    return (field>=0 && field<4) ? fieldTypeStrings[field] : NULL;
}

const char *ShortestPathDiscoverHolePacketDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
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

int ShortestPathDiscoverHolePacketDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    ShortestPathDiscoverHolePacket *pp = (ShortestPathDiscoverHolePacket *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string ShortestPathDiscoverHolePacketDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    ShortestPathDiscoverHolePacket *pp = (ShortestPathDiscoverHolePacket *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getOriginatorId());
        case 1: {std::stringstream out; out << pp->getBallCenter(); return out.str();}
        case 2: return long2string(pp->getPreviousId());
        case 3: return oppstring2string(pp->getPath());
        default: return "";
    }
}

bool ShortestPathDiscoverHolePacketDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    ShortestPathDiscoverHolePacket *pp = (ShortestPathDiscoverHolePacket *)object; (void)pp;
    switch (field) {
        case 0: pp->setOriginatorId(string2long(value)); return true;
        case 2: pp->setPreviousId(string2long(value)); return true;
        case 3: pp->setPath((value)); return true;
        default: return false;
    }
}

const char *ShortestPathDiscoverHolePacketDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 1: return opp_typename(typeid(Point));
        default: return NULL;
    };
}

void *ShortestPathDiscoverHolePacketDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    ShortestPathDiscoverHolePacket *pp = (ShortestPathDiscoverHolePacket *)object; (void)pp;
    switch (field) {
        case 1: return (void *)(&pp->getBallCenter()); break;
        default: return NULL;
    }
}


