# Add directory to python import path
import sys as _sys
from pathlib import Path as _Path
_pb_dir = str(_Path(__file__).parent)
if _pb_dir not in _sys.path:
    _sys.path.append(_pb_dir)
    
# Set protobuf implementation to cpp
import os as _os
#_os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'cpp'

import goldo
import google.protobuf as _pb
import google.protobuf.wrappers_pb2 as _pb_wrappers
import google.protobuf.empty_pb2 as _pb_empty
from google.protobuf.json_format import ParseDict as _ParseDict

from .codecs import serialize, deserialize

_sym_db = _pb.symbol_database.Default()

def get_symbol(name):
    return _sym_db.GetSymbol(name)
    
def from_dict(full_name, _dict):
    msg = get_symbol(full_name)()
    return _ParseDict(_dict, msg)
    

