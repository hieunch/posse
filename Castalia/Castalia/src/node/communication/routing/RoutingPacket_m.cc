//
// Generated file, do not edit! Created by nedtool 4.6 from src/node/communication/routing/RoutingPacket.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "RoutingPacket_m.h"

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

NetMacInfoExchange_type::NetMacInfoExchange_type()
{
    RSSI = 0;
    LQI = 0;
    nextHop = 0;
    lastHop = 0;
}

void doPacking(cCommBuffer *b, NetMacInfoExchange_type& a)
{
    doPacking(b,a.RSSI);
    doPacking(b,a.LQI);
    doPacking(b,a.nextHop);
    doPacking(b,a.lastHop);
}

void doUnpacking(cCommBuffer *b, NetMacInfoExchange_type& a)
{
    doUnpacking(b,a.RSSI);
    doUnpacking(b,a.LQI);
    doUnpacking(b,a.nextHop);
    doUnpacking(b,a.lastHop);
}

class NetMacInfoExchange_typeDescriptor : public cClassDescriptor
{
  public:
    NetMacInfoExchange_typeDescriptor();
    virtual ~NetMacInfoExchange_typeDescriptor();

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

Register_ClassDescriptor(NetMacInfoExchange_typeDescriptor);

NetMacInfoExchange_typeDescriptor::NetMacInfoExchange_typeDescriptor() : cClassDescriptor("NetMacInfoExchange_type", "")
{
}

NetMacInfoExchange_typeDescriptor::~NetMacInfoExchange_typeDescriptor()
{
}

bool NetMacInfoExchange_typeDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<NetMacInfoExchange_type *>(obj)!=NULL;
}

const char *NetMacInfoExchange_typeDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int NetMacInfoExchange_typeDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 4+basedesc->getFieldCount(object) : 4;
}

unsigned int NetMacInfoExchange_typeDescriptor::getFieldTypeFlags(void *object, int field) const
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
        FD_ISEDITABLE,
    };
    return (field>=0 && field<4) ? fieldTypeFlags[field] : 0;
}

const char *NetMacInfoExchange_typeDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "RSSI",
        "LQI",
        "nextHop",
        "lastHop",
    };
    return (field>=0 && field<4) ? fieldNames[field] : NULL;
}

int NetMacInfoExchange_typeDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='R' && strcmp(fieldName, "RSSI")==0) return base+0;
    if (fieldName[0]=='L' && strcmp(fieldName, "LQI")==0) return base+1;
    if (fieldName[0]=='n' && strcmp(fieldName, "nextHop")==0) return base+2;
    if (fieldName[0]=='l' && strcmp(fieldName, "lastHop")==0) return base+3;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *NetMacInfoExchange_typeDescriptor::getFieldTypeString(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeString(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldTypeStrings[] = {
        "double",
        "double",
        "int",
        "int",
    };
    return (field>=0 && field<4) ? fieldTypeStrings[field] : NULL;
}

const char *NetMacInfoExchange_typeDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
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

int NetMacInfoExchange_typeDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    NetMacInfoExchange_type *pp = (NetMacInfoExchange_type *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string NetMacInfoExchange_typeDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    NetMacInfoExchange_type *pp = (NetMacInfoExchange_type *)object; (void)pp;
    switch (field) {
        case 0: return double2string(pp->RSSI);
        case 1: return double2string(pp->LQI);
        case 2: return long2string(pp->nextHop);
        case 3: return long2string(pp->lastHop);
        default: return "";
    }
}

bool NetMacInfoExchange_typeDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    NetMacInfoExchange_type *pp = (NetMacInfoExchange_type *)object; (void)pp;
    switch (field) {
        case 0: pp->RSSI = string2double(value); return true;
        case 1: pp->LQI = string2double(value); return true;
        case 2: pp->nextHop = string2long(value); return true;
        case 3: pp->lastHop = string2long(value); return true;
        default: return false;
    }
}

const char *NetMacInfoExchange_typeDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        default: return NULL;
    };
}

void *NetMacInfoExchange_typeDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    NetMacInfoExchange_type *pp = (NetMacInfoExchange_type *)object; (void)pp;
    switch (field) {
        default: return NULL;
    }
}

Register_Class(RoutingPacket);

RoutingPacket::RoutingPacket(const char *name, int kind) : ::cPacket(name,kind)
{
    this->TTL_var = 10000;
    this->hopCount_var = 0;
    this->distanceCount_var = 0;
    this->isDataPacket_var = false;
    this->source_var = 0;
    this->destination_var = 0;
    this->sequenceNumber_var = 0;
}

RoutingPacket::RoutingPacket(const RoutingPacket& other) : ::cPacket(other)
{
    copy(other);
}

RoutingPacket::~RoutingPacket()
{
}

RoutingPacket& RoutingPacket::operator=(const RoutingPacket& other)
{
    if (this==&other) return *this;
    ::cPacket::operator=(other);
    copy(other);
    return *this;
}

void RoutingPacket::copy(const RoutingPacket& other)
{
    this->netMacInfoExchange_var = other.netMacInfoExchange_var;
    this->TTL_var = other.TTL_var;
    this->hopCount_var = other.hopCount_var;
    this->distanceCount_var = other.distanceCount_var;
    this->isDataPacket_var = other.isDataPacket_var;
    this->source_var = other.source_var;
    this->destination_var = other.destination_var;
    this->sequenceNumber_var = other.sequenceNumber_var;
}

void RoutingPacket::parsimPack(cCommBuffer *b)
{
    ::cPacket::parsimPack(b);
    doPacking(b,this->netMacInfoExchange_var);
    doPacking(b,this->TTL_var);
    doPacking(b,this->hopCount_var);
    doPacking(b,this->distanceCount_var);
    doPacking(b,this->isDataPacket_var);
    doPacking(b,this->source_var);
    doPacking(b,this->destination_var);
    doPacking(b,this->sequenceNumber_var);
}

void RoutingPacket::parsimUnpack(cCommBuffer *b)
{
    ::cPacket::parsimUnpack(b);
    doUnpacking(b,this->netMacInfoExchange_var);
    doUnpacking(b,this->TTL_var);
    doUnpacking(b,this->hopCount_var);
    doUnpacking(b,this->distanceCount_var);
    doUnpacking(b,this->isDataPacket_var);
    doUnpacking(b,this->source_var);
    doUnpacking(b,this->destination_var);
    doUnpacking(b,this->sequenceNumber_var);
}

NetMacInfoExchange_type& RoutingPacket::getNetMacInfoExchange()
{
    return netMacInfoExchange_var;
}

void RoutingPacket::setNetMacInfoExchange(const NetMacInfoExchange_type& netMacInfoExchange)
{
    this->netMacInfoExchange_var = netMacInfoExchange;
}

int RoutingPacket::getTTL() const
{
    return TTL_var;
}

void RoutingPacket::setTTL(int TTL)
{
    this->TTL_var = TTL;
}

int RoutingPacket::getHopCount() const
{
    return hopCount_var;
}

void RoutingPacket::setHopCount(int hopCount)
{
    this->hopCount_var = hopCount;
}

double RoutingPacket::getDistanceCount() const
{
    return distanceCount_var;
}

void RoutingPacket::setDistanceCount(double distanceCount)
{
    this->distanceCount_var = distanceCount;
}

bool RoutingPacket::getIsDataPacket() const
{
    return isDataPacket_var;
}

void RoutingPacket::setIsDataPacket(bool isDataPacket)
{
    this->isDataPacket_var = isDataPacket;
}

const char * RoutingPacket::getSource() const
{
    return source_var.c_str();
}

void RoutingPacket::setSource(const char * source)
{
    this->source_var = source;
}

const char * RoutingPacket::getDestination() const
{
    return destination_var.c_str();
}

void RoutingPacket::setDestination(const char * destination)
{
    this->destination_var = destination;
}

unsigned int RoutingPacket::getSequenceNumber() const
{
    return sequenceNumber_var;
}

void RoutingPacket::setSequenceNumber(unsigned int sequenceNumber)
{
    this->sequenceNumber_var = sequenceNumber;
}

class RoutingPacketDescriptor : public cClassDescriptor
{
  public:
    RoutingPacketDescriptor();
    virtual ~RoutingPacketDescriptor();

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

Register_ClassDescriptor(RoutingPacketDescriptor);

RoutingPacketDescriptor::RoutingPacketDescriptor() : cClassDescriptor("RoutingPacket", "cPacket")
{
}

RoutingPacketDescriptor::~RoutingPacketDescriptor()
{
}

bool RoutingPacketDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<RoutingPacket *>(obj)!=NULL;
}

const char *RoutingPacketDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int RoutingPacketDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 8+basedesc->getFieldCount(object) : 8;
}

unsigned int RoutingPacketDescriptor::getFieldTypeFlags(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeFlags(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISCOMPOUND,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<8) ? fieldTypeFlags[field] : 0;
}

const char *RoutingPacketDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "netMacInfoExchange",
        "TTL",
        "hopCount",
        "distanceCount",
        "isDataPacket",
        "source",
        "destination",
        "sequenceNumber",
    };
    return (field>=0 && field<8) ? fieldNames[field] : NULL;
}

int RoutingPacketDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='n' && strcmp(fieldName, "netMacInfoExchange")==0) return base+0;
    if (fieldName[0]=='T' && strcmp(fieldName, "TTL")==0) return base+1;
    if (fieldName[0]=='h' && strcmp(fieldName, "hopCount")==0) return base+2;
    if (fieldName[0]=='d' && strcmp(fieldName, "distanceCount")==0) return base+3;
    if (fieldName[0]=='i' && strcmp(fieldName, "isDataPacket")==0) return base+4;
    if (fieldName[0]=='s' && strcmp(fieldName, "source")==0) return base+5;
    if (fieldName[0]=='d' && strcmp(fieldName, "destination")==0) return base+6;
    if (fieldName[0]=='s' && strcmp(fieldName, "sequenceNumber")==0) return base+7;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *RoutingPacketDescriptor::getFieldTypeString(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeString(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldTypeStrings[] = {
        "NetMacInfoExchange_type",
        "int",
        "int",
        "double",
        "bool",
        "string",
        "string",
        "unsigned int",
    };
    return (field>=0 && field<8) ? fieldTypeStrings[field] : NULL;
}

const char *RoutingPacketDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
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

int RoutingPacketDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    RoutingPacket *pp = (RoutingPacket *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string RoutingPacketDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    RoutingPacket *pp = (RoutingPacket *)object; (void)pp;
    switch (field) {
        case 0: {std::stringstream out; out << pp->getNetMacInfoExchange(); return out.str();}
        case 1: return long2string(pp->getTTL());
        case 2: return long2string(pp->getHopCount());
        case 3: return double2string(pp->getDistanceCount());
        case 4: return bool2string(pp->getIsDataPacket());
        case 5: return oppstring2string(pp->getSource());
        case 6: return oppstring2string(pp->getDestination());
        case 7: return ulong2string(pp->getSequenceNumber());
        default: return "";
    }
}

bool RoutingPacketDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    RoutingPacket *pp = (RoutingPacket *)object; (void)pp;
    switch (field) {
        case 1: pp->setTTL(string2long(value)); return true;
        case 2: pp->setHopCount(string2long(value)); return true;
        case 3: pp->setDistanceCount(string2double(value)); return true;
        case 4: pp->setIsDataPacket(string2bool(value)); return true;
        case 5: pp->setSource((value)); return true;
        case 6: pp->setDestination((value)); return true;
        case 7: pp->setSequenceNumber(string2ulong(value)); return true;
        default: return false;
    }
}

const char *RoutingPacketDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 0: return opp_typename(typeid(NetMacInfoExchange_type));
        default: return NULL;
    };
}

void *RoutingPacketDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    RoutingPacket *pp = (RoutingPacket *)object; (void)pp;
    switch (field) {
        case 0: return (void *)(&pp->getNetMacInfoExchange()); break;
        default: return NULL;
    }
}


