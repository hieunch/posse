//
// Generated file, do not edit! Created by nedtool 4.6 from src/node/communication/routing/alba-r/RainbowRoutingPacket.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "RainbowRoutingPacket_m.h"

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
    cEnum *e = cEnum::find("RainbowPacketDef");
    if (!e) enums.getInstance()->add(e = new cEnum("RainbowPacketDef"));
    e->insert(RAINBOW_DATA_PACKET, "RAINBOW_DATA_PACKET");
);

Register_Class(RainbowPacket);

RainbowPacket::RainbowPacket(const char *name, int kind) : ::RoutingPacket(name,kind)
{
    this->packetId_var = 0;
    this->RainbowPacketKind_var = 0;
}

RainbowPacket::RainbowPacket(const RainbowPacket& other) : ::RoutingPacket(other)
{
    copy(other);
}

RainbowPacket::~RainbowPacket()
{
}

RainbowPacket& RainbowPacket::operator=(const RainbowPacket& other)
{
    if (this==&other) return *this;
    ::RoutingPacket::operator=(other);
    copy(other);
    return *this;
}

void RainbowPacket::copy(const RainbowPacket& other)
{
    this->packetId_var = other.packetId_var;
    this->RainbowPacketKind_var = other.RainbowPacketKind_var;
    this->destLocation_var = other.destLocation_var;
}

void RainbowPacket::parsimPack(cCommBuffer *b)
{
    ::RoutingPacket::parsimPack(b);
    doPacking(b,this->packetId_var);
    doPacking(b,this->RainbowPacketKind_var);
    doPacking(b,this->destLocation_var);
}

void RainbowPacket::parsimUnpack(cCommBuffer *b)
{
    ::RoutingPacket::parsimUnpack(b);
    doUnpacking(b,this->packetId_var);
    doUnpacking(b,this->RainbowPacketKind_var);
    doUnpacking(b,this->destLocation_var);
}

int RainbowPacket::getPacketId() const
{
    return packetId_var;
}

void RainbowPacket::setPacketId(int packetId)
{
    this->packetId_var = packetId;
}

int RainbowPacket::getRainbowPacketKind() const
{
    return RainbowPacketKind_var;
}

void RainbowPacket::setRainbowPacketKind(int RainbowPacketKind)
{
    this->RainbowPacketKind_var = RainbowPacketKind;
}

Point& RainbowPacket::getDestLocation()
{
    return destLocation_var;
}

void RainbowPacket::setDestLocation(const Point& destLocation)
{
    this->destLocation_var = destLocation;
}

class RainbowPacketDescriptor : public cClassDescriptor
{
  public:
    RainbowPacketDescriptor();
    virtual ~RainbowPacketDescriptor();

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

Register_ClassDescriptor(RainbowPacketDescriptor);

RainbowPacketDescriptor::RainbowPacketDescriptor() : cClassDescriptor("RainbowPacket", "RoutingPacket")
{
}

RainbowPacketDescriptor::~RainbowPacketDescriptor()
{
}

bool RainbowPacketDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<RainbowPacket *>(obj)!=NULL;
}

const char *RainbowPacketDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int RainbowPacketDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 3+basedesc->getFieldCount(object) : 3;
}

unsigned int RainbowPacketDescriptor::getFieldTypeFlags(void *object, int field) const
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
        FD_ISCOMPOUND,
    };
    return (field>=0 && field<3) ? fieldTypeFlags[field] : 0;
}

const char *RainbowPacketDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "packetId",
        "RainbowPacketKind",
        "destLocation",
    };
    return (field>=0 && field<3) ? fieldNames[field] : NULL;
}

int RainbowPacketDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='p' && strcmp(fieldName, "packetId")==0) return base+0;
    if (fieldName[0]=='R' && strcmp(fieldName, "RainbowPacketKind")==0) return base+1;
    if (fieldName[0]=='d' && strcmp(fieldName, "destLocation")==0) return base+2;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *RainbowPacketDescriptor::getFieldTypeString(void *object, int field) const
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
        "Point",
    };
    return (field>=0 && field<3) ? fieldTypeStrings[field] : NULL;
}

const char *RainbowPacketDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldProperty(object, field, propertyname);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 1:
            if (!strcmp(propertyname,"enum")) return "RainbowPacketDef";
            return NULL;
        default: return NULL;
    }
}

int RainbowPacketDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    RainbowPacket *pp = (RainbowPacket *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string RainbowPacketDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    RainbowPacket *pp = (RainbowPacket *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getPacketId());
        case 1: return long2string(pp->getRainbowPacketKind());
        case 2: {std::stringstream out; out << pp->getDestLocation(); return out.str();}
        default: return "";
    }
}

bool RainbowPacketDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    RainbowPacket *pp = (RainbowPacket *)object; (void)pp;
    switch (field) {
        case 0: pp->setPacketId(string2long(value)); return true;
        case 1: pp->setRainbowPacketKind(string2long(value)); return true;
        default: return false;
    }
}

const char *RainbowPacketDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 2: return opp_typename(typeid(Point));
        default: return NULL;
    };
}

void *RainbowPacketDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    RainbowPacket *pp = (RainbowPacket *)object; (void)pp;
    switch (field) {
        case 2: return (void *)(&pp->getDestLocation()); break;
        default: return NULL;
    }
}


