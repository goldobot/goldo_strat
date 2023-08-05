from  google.protobuf.descriptor import FieldDescriptor as _fd
import goldo.pb2_options_pb2 as _opts
import struct

import google.protobuf as _pb
_sym_db = _pb.symbol_database.Default()

_types = {
    _fd.TYPE_BOOL: '?',
    _fd.TYPE_SINT32: 'i',
    _fd.TYPE_INT32: 'I',
    _fd.TYPE_FIXED32: 'I',
    _fd.TYPE_SINT64: 'q',
    _fd.TYPE_INT64: 'Q',
    _fd.TYPE_FLOAT: 'f',
    _fd.TYPE_DOUBLE: 'd'
    }
    
_cpp_types = {
    _opts.CppType.INT8: 'b',
    _opts.CppType.UINT8: 'B',
    _opts.CppType.INT16: 'h',
    _opts.CppType.UINT16: 'H',
    _opts.CppType.INT32: 'i',
    _opts.CppType.UINT32: 'I',
    _opts.CppType.INT64: 'q',
    _opts.CppType.UINT64: 'Q',
    _opts.CppType.FLOAT: 'f',
    _opts.CppType.DOUBLE: 'd'
    }
    
_message_codecs = {}

def get_message_codec(full_name):
    if full_name in _message_codecs:
        return _message_codecs[full_name]
    msg_type = _sym_db.GetSymbol(full_name)
    codec = _MessageCodec(msg_type)
    _message_codecs[full_name] = codec
    return codec
    
def _get_cpp_type(field):
    options = field.GetOptions()
    if options.HasExtension(_opts.cpp_type):
        return options.Extensions[_opts.cpp_type]

class _AttrFieldCodec:
    def __init__(self, name):
        self._name = name
        
    def get(self, msg):
        return getattr(msg, self._name)
        
    def set(self, msg, val):
        return setattr(msg, self._name, val)
        
class _MessageFieldCodec:
    def __init__(self, name, serializer):
        self._name = name
        self._codec = serializer
        
    def get(self, msg):
        return self._codec.serialize(getattr(msg, self._name))
        
    def set(self, msg, val):
        getattr(msg, self._name).CopyFrom(self._codec.deserialize(val))
        
class _ArrayFieldCodec:
    def __init__(self, name, count, serializer):
        self._name = name
        self._count = count
        self._codec = serializer
        
    def get(self, msg):
        array = getattr(msg, self._name)
        buff = b''
        for i in range(self._count):
            if i < len(array):
                buff += self._codec.serialize(array[i])
            else:
                buff += self._codec.serialize(self._codec._msg_type())
        return buff
        
    def set(self, msg, val):
        size = self._codec._size
        getattr(msg, self._name).extend([self._codec.deserialize(val[i*size:(i+1)*size]) for i in range(self._count)])        
       
class _MessageCodec:
    def __init__(self, msg_type):
        self._msg_type = msg_type
        self._descriptor = msg_type.DESCRIPTOR
        self._struct_fmt = '<'
        self._field_codecs = []
        self._is_fixed_size = True

        for field in self._descriptor.fields:
            if _get_cpp_type(field) == _opts.CppType.VOID:
                continue
            if field.label == _fd.LABEL_REPEATED:
                self._add_repeated(field)
            elif field.type == _fd.TYPE_MESSAGE:
                self._add_message(field)
            else:
                self._add_field(field)
        self._size = struct.calcsize(self._struct_fmt)
        self._unpack = struct.Struct(self._struct_fmt).unpack
        self._pack = struct.Struct(self._struct_fmt).pack
        
    def serialize(self, msg):
        return self._pack(*(c.get(msg) for c in self._field_codecs))
        
    def deserialize(self, payload):
        msg = self._msg_type()
        vals = self._unpack(payload)
        for i in range(len(self._field_codecs)):            
            self._field_codecs[i].set(msg, vals[i])
        return msg
        
    def _add_repeated(self, field):
        options = field.GetOptions()
        if options.HasExtension(_opts.fixed_count):
            # Fixed size array
            array_count = options.Extensions[_opts.max_count]
            codec = get_message_codec(field.message_type.full_name)
            self._struct_fmt += '{}s'.format(codec._size * array_count)
            self._field_codecs.append(_ArrayFieldCodec(field.name, array_count, codec))
        else:
            print('ERROR, protobuf repeated field misconfigured')
        
    def _add_message(self, field):
        codec = get_message_codec(field.message_type.full_name)
        self._struct_fmt += '{}s'.format(codec._size)
        self._field_codecs.append(_MessageFieldCodec(field.name, codec))
        
    def _add_field(self, field):
        cpp_type = _get_cpp_type(field)
        if cpp_type is not None:
            self._struct_fmt += _cpp_types[cpp_type]
        elif field.type in _types:
            self._struct_fmt += _types[field.type]
        else:
            print('ERROR', field.type)
        self._field_codecs.append(_AttrFieldCodec(field.name))
    
def serialize(msg):
    codec = get_message_codec(msg.DESCRIPTOR.full_name)    
    return codec.serialize(msg)
   
def deserialize(full_name, payload):
    codec = get_message_codec(full_name)    
    return codec.deserialize(payload)
