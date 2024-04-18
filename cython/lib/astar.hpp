#ifndef ASTAR_H_
#define ASTAR_H_

#include <list>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "fixinclude.hpp"

using namespace std;

class AStar
{
    SearchNode **matrix;

    UINT width;
    UINT height;
    pair<UINT, UINT> start;
    pair<UINT, UINT> end;

    list<pair<UINT, UINT> > path;
    list<pair<UINT, UINT> > m_smoothPath;
    list<pair<UINT, UINT> > m_smoothMarginPath;
    list<pair<UINT, UINT> > openedNodes;
    list<list<NodeState> > matrixsChanges;

    AStarHeuristics heuristic;

    bool ready;

    void calculateCost(pair<UINT, UINT> node);
    void openNode(UINT x, UINT y, UINT originX, UINT originY, list<NodeState> *changes = NULL);
    void freeNodes();

    void setOriginNode(UINT x, UINT y, UINT originX, UINT originY);

    double pathCostToNode(UINT x, UINT y, UINT destX, UINT destY);

    bool lineOfSight(int x0, int y0, int x1, int y1);

    void AddMargin();
    bool NeedAddMargin(int x0, int y0, int x1, int y1, int *newX, int *newY);
    bool HasNeighbour(int x, int y, int *avoidNeighbourX, int *avoidNeighbourY);   
    

public:

    AStar();
    AStar(UINT width, UINT height);
    ~AStar();

    void setMatrix(UINT width, UINT height);
    void setStart(UINT x, UINT y);
    void setEnd(UINT x, UINT y);
    void setWall(UINT x, UINT y);
    void setWay(UINT x, UINT y, UINT expandCost = 1);
    void setHeuristics(AStarHeuristics heuristic);
    
    void clipPoint(int& x, int& y) const;
    
    void setNode(UINT x, UINT y, NodeType type, UINT expandCost= 1);
    
    void fillDisk(float x, float y, float r, NodeType type, UINT expandCost= 1);
    void fillRect(int x1, int y1, int x2, int y2, NodeType type, UINT expandCost= 1);
    void fillPoly(float* pts_x,float* pts_y, UINT num_pts, NodeType type, UINT expandCost= 1);
    
    

    void search(bool saveChanges = false);

    NodeType getNodeType(UINT x, UINT y) const;
    pair<UINT, UINT> getStart() const;
    pair<UINT, UINT> getEnd() const;
    list<list<NodeState> > getChanges() const;
    bool getDebugArr(char* arr, UINT size) const;

    double estimateCost(UINT x1, UINT y1, UINT x2, UINT y2);

    void destroyMatrix();

    list<pair<UINT, UINT> > getPath(AStarPathType getPathType)
    {
        switch(getPathType)
        {
        case AStarPathType::raw:
            return path;
        case AStarPathType::smooth:
            return m_smoothPath;
        case AStarPathType::smoothMargin:
            return m_smoothMarginPath;
        default:
            break;
        }

        return path;
    }

    list<pair<UINT, UINT> > getPathOnlyIfNeed(bool autoPathComputeIfNeed = true, bool * isNewPath = NULL, AStarPathType getPathType = smooth);

    void ClearPath(void);
    bool IsPathStillValid(void);

};

#endif
