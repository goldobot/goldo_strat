# distutils: language = c++
# cython: language_level=3

from libc.math cimport floor, ceil

from libcpp.list cimport list
from libcpp.pair cimport pair
from libcpp cimport bool

from cpython.bytes cimport PyBytes_FromStringAndSize
from cpython cimport array

from AStar cimport AStar, NodeType, AStarPathType

cdef (float, float) Point

cdef class AStarWrapper:
    cdef AStar* c_astar
    # FIXME : TODO : generic code for coordinate system setting
    #cdef char c_arr[300 * 200] # 2023
    cdef char c_arr[200 * 300]

    def __cinit__(self):
        self.c_astar = new AStar()
        # FIXME : TODO : generic code for coordinate system setting
        #self.c_astar[0].setMatrix(300,200) # 2023
        self.c_astar[0].setMatrix(200,300)

    def fillDisk(self, p, r, c):
        cdef float xf = 100.0*p[0]
        # FIXME : TODO : generic code for coordinate system setting
        #cdef float yf = 100.0*p[1] + 99 # 2023
        cdef float yf = 100.0*p[1] + 149
        cdef NodeType node_type = NodeType.WALLNODE if c==0 else NodeType.WAYNODE
        self.c_astar[0].fillDisk(xf, yf,r*100.0,node_type,c)

    def fillRect(self, p1, p2, c):
        cdef NodeType node_type = NodeType.WALLNODE if c==0 else NodeType.WAYNODE
        cdef int ix1 = int(floor(100.0*p1[0]))
        # FIXME : TODO : generic code for coordinate system setting
        #cdef int iy1 = int(floor(100.0*p1[1] + 99)) # 2023
        cdef int iy1 = int(floor(100.0*p1[1] + 149))
        cdef int ix2 = int(ceil(100.0*p2[0]))
        # FIXME : TODO : generic code for coordinate system setting
        #cdef int iy2 = int(ceil(100.0*p2[1] + 99)) # 2023
        cdef int iy2 = int(ceil(100.0*p2[1] + 149))
        self.c_astar[0].fillRect(ix1,iy1,ix2,iy2,node_type,c)

    def fillPoly(self, pts, c):
        cdef NodeType node_type = NodeType.WALLNODE if c==0 else NodeType.WAYNODE
        cdef array.array x = array.array('f', [100.0*p[0] for p in pts])
        # FIXME : TODO : generic code for coordinate system setting
        #cdef array.array y = array.array('f', [100.0*p[1] + 99 for p in pts])
        cdef array.array y = array.array('f', [100.0*p[1] + 149 for p in pts])
        self.c_astar[0].fillPoly(x.data.as_floats,y.data.as_floats,len(pts),node_type,c)

    def getArr(self):
        # FIXME : TODO : generic code for coordinate system setting
        #self.c_astar[0].getDebugArr(self.c_arr, 300 * 200) # 2023
        #return PyBytes_FromStringAndSize(self.c_arr, 300 * 200)
        self.c_astar[0].getDebugArr(self.c_arr, 200 * 300)
        return PyBytes_FromStringAndSize(self.c_arr, 200 * 300)

    cdef setWall(self, unsigned x, unsigned y):
        self.c_astar[0].setWall(x, y)

    cdef setWay(self, unsigned x, unsigned y):
        self.c_astar[0].setWay(x, y, 1)

    def resetCosts(self):
        # background
        # FIXME : TODO : generic code for coordinate system setting
        #self.fillRect((0, -1.0), (3.0, 1.0), 1) # 2023
        self.fillRect((0, -1.5), (2.0, 1.5), 1)

    def computePath(self, p0, p1):
        cdef unsigned x0 = p0[0] * 100
        # FIXME : TODO : generic code for coordinate system setting
        #cdef unsigned y0 = p0[1] * 100 + 99 # 2023
        cdef unsigned y0 = p0[1] * 100 + 149
        cdef unsigned x1 = p1[0] * 100
        # FIXME : TODO : generic code for coordinate system setting
        #cdef unsigned y1 = p1[1] * 100 + 99 # 2023
        cdef unsigned y1 = p1[1] * 100 + 149

        self.c_astar[0].setStart(x0,y0)
        self.c_astar[0].setEnd(x1,y1)

        cdef list[pair[unsigned,unsigned]] path
        cdef bool is_new_path
        cdef AStarPathType path_type = AStarPathType.smooth 


        path = self.c_astar[0].getPathOnlyIfNeed(True, &is_new_path, path_type)
        ret = []
        for p in path:
            # FIXME : TODO : generic code for coordinate system setting
            #ret.append((p.first * 0.01, (<int>(p.second) - 99) * 0.01)) # 2023
            ret.append((p.first * 0.01, (<int>(p.second) - 149) * 0.01))
        return ret

