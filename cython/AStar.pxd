# distutils: language = c++
# cython: language_level=3
from libcpp.list cimport list
from libcpp.pair cimport pair
from libcpp cimport bool


cdef extern from "lib/astar.cpp":
    pass

    
cdef extern from "lib/astar.hpp":
    ctypedef enum AStarPathType:
        raw,
        smooth
    ctypedef enum NodeType:
        none,
        WAYNODE,
        WALLNODE
    cdef cppclass AStar:
        AStar() except +
        AStar(unsigned, unsigned) except +
        void setMatrix(unsigned,unsigned) except +
        
        void setStart(unsigned,unsigned) except +
        void setEnd(unsigned,unsigned) except +
        
        void setWall(unsigned,unsigned) except +
        void setWay(unsigned,unsigned,unsigned) except +
        
        void fillDisk(float,float,float,NodeType,unsigned);
        void fillRect(int,int,int,int,NodeType,unsigned);
        void fillPoly(float*,float*,unsigned,NodeType,unsigned);
        
        bool getDebugArr(char*,unsigned);
        
        list[pair[unsigned,unsigned]] getPathOnlyIfNeed(bool,bool*,AStarPathType) except +