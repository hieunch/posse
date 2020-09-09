//
// Generated file, do not edit! Created by nedtool 4.6 from src/node/communication/routing/boundhole/BoundHoleMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "BoundHoleMessage_m.h"

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
    cEnum *e = cEnum::find("BoundHoleType");
    if (!e) enums.getInstance()->add(e = new cEnum("BoundHoleType"));
    e->insert(BOUNDHOLE_BOUNDHOLE, "BOUNDHOLE_BOUNDHOLE");
    e->insert(BOUNDHOLE_REFRESH, "BOUNDHOLE_REFRESH");
);

Register_Class(BoundHolePacket);

BoundHolePacket::BoundHolePacket(const char *name, int kind) : ::RoutingPacket(name,kind)
{
    this->sourceId_var = 0;
    this->destinationId_var = 0;
    this->packetId_var = 0;
    this->type_var = 0;
    this->index_var = 0;
}

BoundHolePacket::BoundHolePacket(const BoundHolePacket& other) : ::RoutingPacket(other)
{
    copy(other);
}

BoundHolePacket::~BoundHolePacket()
{
}

BoundHolePacket& BoundHolePacket::operator=(const BoundHolePacket& other)
{
    if (this==&other) return *this;
    ::RoutingPacket::operator=(other);
    copy(other);
    return *this;
}

void BoundHolePacket::copy(const BoundHolePacket& other)
{
    this->sourceId_var = other.sourceId_var;
    this->destinationId_var = other.destinationId_var;
    this->packetId_var = other.packetId_var;
    this->type_var = other.type_var;
    this->previousNode_var = other.previousNode_var;
    this->index_var = other.index_var;
    this->boundholeNodes_var = other.boundholeNodes_var;
}

void BoundHolePacket::parsimPack(cCommBuffer *b)
{
    ::RoutingPacket::parsimPack(b);
    doPacking(b,this->sourceId_var);
    doPacking(b,this->destinationId_var);
    doPacking(b,this->packetId_var);
    doPacking(b,this->type_var);
    doPacking(b,this->previousNode_var);
    doPacking(b,this->index_var);
    doPacking(b,this->boundholeNodes_var);
}

void BoundHolePacket::parsimUnpack(cCommBuffer *b)
{
    ::RoutingPacket::parsimUnpack(b);
    doUnpacking(b,this->sourceId_var);
    doUnpacking(b,this->destinationId_var);
    doUnpacking(b,this->packetId_var);
    doUnpacking(b,this->type_var);
    doUnpacking(b,this->previousNode_var);
    doUnpacking(b,this->index_var);
    doUnpacking(b,this->boundholeNodes_var);
}

int BoundHolePacket::getSourceId() const
{
    return sourceId_var;
}

void BoundHolePacket::setSourceId(int sourceId)
{
    this->sourceId_var = sourceId;
}

int BoundHolePacket::getDestinationId() const
{
    return destinationId_var;
}

void BoundHolePacket::setDestinationId(int destinationId)
{
    this->destinationId_var = destinationId;
}

int BoundHolePacket::getPacketId() const
{
    return packetId_var;
}

void BoundHolePacket::setPacketId(int packetId)
{
    this->packetId_var = packetId;
}

int BoundHolePacket::getType() const
{
    return type_var;
}

void BoundHolePacket::setType(int type)
{
    this->type_var = type;
}

Point& BoundHolePacket::getPreviousNode()
{
    return previousNode_var;
}

void BoundHolePacket::setPreviousNode(const Point& previousNode)
{
    this->previousNode_var = previousNode;
}

int BoundHolePacket::getIndex() const
{
    return index_var;
}

void BoundHolePacket::setIndex(int index)
{
    this->index_var = index;
}

NodeVector& BoundHolePacket::getBoundholeNodes()
{
    return boundholeNodes_var;
}

void BoundHolePacket::setBoundholeNodes(const NodeVector& boundholeNodes)
{
    this->boundholeNodes_var = boundholeNodes;
}

class BoundHolePacketDescriptor : public cClassDescriptor
{
  public:
    BoundHolePacketDescriptor();
    virtual ~BoundHolePacketDescriptor();

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

Register_ClassDescriptor(BoundHolePacketDescriptor);

BoundHolePacketDescriptor::BoundHolePacketDescriptor() : cClassDescriptor("BoundHolePacket", "RoutingPacket")
{
}

BoundHolePacketDescriptor::~BoundHolePacketDescriptor()
{
}

bool BoundHolePacketDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<BoundHolePacket *>(obj)!=NULL;
}

const char *BoundHolePacketDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int BoundHolePacketDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 7+basedesc->getFieldCount(object) : 7;
}

unsigned int BoundHolePacketDescriptor::getFieldTypeFlags(void *object, int field) const
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
        FD_ISCOMPOUND,
        FD_ISEDITABLE,
        FD_ISCOMPOUND,
    };
    return (field>=0 && field<7) ? fieldTypeFlags[field] : 0;
}

const char *BoundHolePacketDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "sourceId",
        "destinationId",
        "packetId",
        "type",
        "previousNode",
        "index",
        "boundholeNodes",
    };
    return (field>=0 && field<7) ? fieldNames[field] : NULL;
}

int BoundHolePacketDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='s' && strcmp(fieldName, "sourceId")==0) return base+0;
    if (fieldName[0]=='d' && strcmp(fieldName, "destinationId")==0) return base+1;
    if (fieldName[0]=='p' && strcmp(fieldName, "packetId")==0) return base+2;
    if (fieldName[0]=='t' && strcmp(fieldName, "type")==0) return base+3;
    if (fieldName[0]=='p' && strcmp(fieldName, "previousNode")==0) return base+4;
    if (fieldName[0]=='i' && strcmp(fieldName, "index")==0) return base+5;
    if (fieldName[0]=='b' && strcmp(fieldName, "boundholeNodes")==0) return base+6;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *BoundHolePacketDescriptor::getFieldTypeString(void *object, int field) const
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
        "int",
        "Point",
        "int",
        "NodeVector",
    };
    return (field>=0 && field<7) ? fieldTypeStrings[field] : NULL;
}

const char *BoundHolePacketDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
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

int BoundHolePacketDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    BoundHolePacket *pp = (BoundHolePacket *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string BoundHolePacketDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    BoundHolePacket *pp = (BoundHolePacket *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getSourceId());
        case 1: return long2string(pp->getDestinationId());
        case 2: return long2string(pp->getPacketId());
        case 3: return long2string(pp->getType());
        case 4: {std::stringstream out; out << pp->getPreviousNode(); return out.str();}
        case 5: return long2string(pp->getIndex());
        case 6: {std::stringstream out; out << pp->getBoundholeNodes(); return out.str();}
        default: return "";
    }
}

bool BoundHolePacketDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    BoundHolePacket *pp = (BoundHolePacket *)object; (void)pp;
    switch (field) {
        case 0: pp->setSourceId(string2long(value)); return true;
        case 1: pp->setDestinationId(string2long(value)); return true;
        case 2: pp->setPacketId(string2long(value)); return true;
        case 3: pp->setType(string2long(value)); return true;
        case 5: pp->setIndex(string2long(value)); return true;
        default: return false;
    }
}

const char *BoundHolePacketDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 4: return opp_typename(typeid(Point));
        case 6: return opp_typename(typeid(NodeVector));
        default: return NULL;
    };
}

void *BoundHolePacketDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    BoundHolePacket *pp = (BoundHolePacket *)object; (void)pp;
    switch (field) {
        case 4: return (void *)(&pp->getPreviousNode()); break;
        case 6: return (void *)(&pp->getBoundholeNodes()); break;
        default: return NULL;
    }
}


