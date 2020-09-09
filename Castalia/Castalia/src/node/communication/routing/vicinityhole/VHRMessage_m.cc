//
// Generated file, do not edit! Created by nedtool 4.6 from src/node/communication/routing/vicinityhole/VHRMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "VHRMessage_m.h"

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

Register_Class(VHRDataPacket);

VHRDataPacket::VHRDataPacket(const char *name, int kind) : ::RoutingPacket(name,kind)
{
    this->packetId_var = 0;
    this->apIndex_var = 0;
}

VHRDataPacket::VHRDataPacket(const VHRDataPacket& other) : ::RoutingPacket(other)
{
    copy(other);
}

VHRDataPacket::~VHRDataPacket()
{
}

VHRDataPacket& VHRDataPacket::operator=(const VHRDataPacket& other)
{
    if (this==&other) return *this;
    ::RoutingPacket::operator=(other);
    copy(other);
    return *this;
}

void VHRDataPacket::copy(const VHRDataPacket& other)
{
    this->packetId_var = other.packetId_var;
    this->path_var = other.path_var;
    this->apIndex_var = other.apIndex_var;
    this->destLocation_var = other.destLocation_var;
}

void VHRDataPacket::parsimPack(cCommBuffer *b)
{
    ::RoutingPacket::parsimPack(b);
    doPacking(b,this->packetId_var);
    doPacking(b,this->path_var);
    doPacking(b,this->apIndex_var);
    doPacking(b,this->destLocation_var);
}

void VHRDataPacket::parsimUnpack(cCommBuffer *b)
{
    ::RoutingPacket::parsimUnpack(b);
    doUnpacking(b,this->packetId_var);
    doUnpacking(b,this->path_var);
    doUnpacking(b,this->apIndex_var);
    doUnpacking(b,this->destLocation_var);
}

int VHRDataPacket::getPacketId() const
{
    return packetId_var;
}

void VHRDataPacket::setPacketId(int packetId)
{
    this->packetId_var = packetId;
}

PointVector& VHRDataPacket::getPath()
{
    return path_var;
}

void VHRDataPacket::setPath(const PointVector& path)
{
    this->path_var = path;
}

int VHRDataPacket::getApIndex() const
{
    return apIndex_var;
}

void VHRDataPacket::setApIndex(int apIndex)
{
    this->apIndex_var = apIndex;
}

Point& VHRDataPacket::getDestLocation()
{
    return destLocation_var;
}

void VHRDataPacket::setDestLocation(const Point& destLocation)
{
    this->destLocation_var = destLocation;
}

class VHRDataPacketDescriptor : public cClassDescriptor
{
  public:
    VHRDataPacketDescriptor();
    virtual ~VHRDataPacketDescriptor();

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

Register_ClassDescriptor(VHRDataPacketDescriptor);

VHRDataPacketDescriptor::VHRDataPacketDescriptor() : cClassDescriptor("VHRDataPacket", "RoutingPacket")
{
}

VHRDataPacketDescriptor::~VHRDataPacketDescriptor()
{
}

bool VHRDataPacketDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<VHRDataPacket *>(obj)!=NULL;
}

const char *VHRDataPacketDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int VHRDataPacketDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 4+basedesc->getFieldCount(object) : 4;
}

unsigned int VHRDataPacketDescriptor::getFieldTypeFlags(void *object, int field) const
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
        FD_ISCOMPOUND,
    };
    return (field>=0 && field<4) ? fieldTypeFlags[field] : 0;
}

const char *VHRDataPacketDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "packetId",
        "path",
        "apIndex",
        "destLocation",
    };
    return (field>=0 && field<4) ? fieldNames[field] : NULL;
}

int VHRDataPacketDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='p' && strcmp(fieldName, "packetId")==0) return base+0;
    if (fieldName[0]=='p' && strcmp(fieldName, "path")==0) return base+1;
    if (fieldName[0]=='a' && strcmp(fieldName, "apIndex")==0) return base+2;
    if (fieldName[0]=='d' && strcmp(fieldName, "destLocation")==0) return base+3;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *VHRDataPacketDescriptor::getFieldTypeString(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeString(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldTypeStrings[] = {
        "int",
        "PointVector",
        "int",
        "Point",
    };
    return (field>=0 && field<4) ? fieldTypeStrings[field] : NULL;
}

const char *VHRDataPacketDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
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

int VHRDataPacketDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    VHRDataPacket *pp = (VHRDataPacket *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string VHRDataPacketDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    VHRDataPacket *pp = (VHRDataPacket *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getPacketId());
        case 1: {std::stringstream out; out << pp->getPath(); return out.str();}
        case 2: return long2string(pp->getApIndex());
        case 3: {std::stringstream out; out << pp->getDestLocation(); return out.str();}
        default: return "";
    }
}

bool VHRDataPacketDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    VHRDataPacket *pp = (VHRDataPacket *)object; (void)pp;
    switch (field) {
        case 0: pp->setPacketId(string2long(value)); return true;
        case 2: pp->setApIndex(string2long(value)); return true;
        default: return false;
    }
}

const char *VHRDataPacketDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 1: return opp_typename(typeid(PointVector));
        case 3: return opp_typename(typeid(Point));
        default: return NULL;
    };
}

void *VHRDataPacketDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    VHRDataPacket *pp = (VHRDataPacket *)object; (void)pp;
    switch (field) {
        case 1: return (void *)(&pp->getPath()); break;
        case 3: return (void *)(&pp->getDestLocation()); break;
        default: return NULL;
    }
}


