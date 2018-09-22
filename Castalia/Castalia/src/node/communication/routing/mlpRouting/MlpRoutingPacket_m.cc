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

Register_Class(MlpPacket);

MlpPacket::MlpPacket(const char *name, int kind) : ::RoutingPacket(name,kind)
{
    this->packetId_var = 0;
    this->previousId_var = 0;
    this->MlpPacketKind_var = 0;
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
}

void MlpPacket::parsimPack(cCommBuffer *b)
{
    ::RoutingPacket::parsimPack(b);
    doPacking(b,this->packetId_var);
    doPacking(b,this->previousId_var);
    doPacking(b,this->MlpPacketKind_var);
    doPacking(b,this->destLocation_var);
    doPacking(b,this->sourceLocation_var);
}

void MlpPacket::parsimUnpack(cCommBuffer *b)
{
    ::RoutingPacket::parsimUnpack(b);
    doUnpacking(b,this->packetId_var);
    doUnpacking(b,this->previousId_var);
    doUnpacking(b,this->MlpPacketKind_var);
    doUnpacking(b,this->destLocation_var);
    doUnpacking(b,this->sourceLocation_var);
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
    return basedesc ? 5+basedesc->getFieldCount(object) : 5;
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
    };
    return (field>=0 && field<5) ? fieldTypeFlags[field] : 0;
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
    };
    return (field>=0 && field<5) ? fieldNames[field] : NULL;
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
    };
    return (field>=0 && field<5) ? fieldTypeStrings[field] : NULL;
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
        default: return NULL;
    }
}


