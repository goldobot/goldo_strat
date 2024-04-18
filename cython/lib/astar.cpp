#include "astar.hpp"

#include <vector>
#include <algorithm>

SearchNode::SearchNode()
{
    type = WAYNODE;
    originX = 0;
    originY = 0;
    h = numeric_limits<double>::max();
    g = numeric_limits<UINT>::max();
    expandCost = 1;
    cost = numeric_limits<double>::max();
    pathCost = numeric_limits<double>::max();
}

SearchNode::~SearchNode()
{

}

PathNode::PathNode()
{
    x = 0;
    y = 0;
}

PathNode::~PathNode()
{

}

NodeState::NodeState()
{
    x = 0;
    y = 0;

    state = NONE;
}

NodeState::NodeState(UINT x, UINT y, NodeType state)
{
    this->x = x;
    this->y = y;
    this->state = state;
    this->ancestor = SELF;
}

NodeState::~NodeState()
{

}

AStar::AStar()
{
    matrix = NULL;
    width = 0;
    height = 0;
    heuristic = euclidean;

    start.first = 0;
    start.second = 0;
    end.first = 0;
    end.second = 0;
    path.clear();
    m_smoothPath.clear();
    m_smoothMarginPath.clear();
    openedNodes.clear();
    matrixsChanges.clear();
    ready = false;
}

AStar::AStar(UINT width, UINT height)
{
    matrix = new SearchNode*[height];

    for (UINT i = 0; i < height; i++)
        matrix[i] = new SearchNode[width];

    this->width = width;
    this->height = height;
    start.first = 0;
    start.second = 0;
    end.first = 0;
    end.second = 0;
    path.clear();
    m_smoothPath.clear();
    m_smoothMarginPath.clear();
    openedNodes.clear();
    matrixsChanges.clear();
    ready = true;
}

AStar::~AStar()
{
    destroyMatrix();

    path.clear();
    m_smoothPath.clear();
    m_smoothMarginPath.clear();
    openedNodes.clear();
}

void AStar::calculateCost(pair<UINT, UINT> node)
{
    double x, y, destX, destY, h, cost, euc, pathCost;
    UINT g, originX, originY;
    h = 0;

    x = (double)node.first;
    y = (double)node.second;
    destX = (double)end.first;
    destY = (double)end.second;

    originX = matrix[node.first][node.second].originX;
    originY = matrix[node.first][node.second].originY;

    g = matrix[originX][originY].g + matrix[node.first][node.second].expandCost;

    euc = sqrt(pow(x - destX, 2.0) + pow(y - destY, 2.0));

    if (heuristic == none)					h = 0;
    else if (heuristic == euclidean)		h = euc;
    else if (heuristic == manhattan)		h = abs(x - destX) + abs(y - destY);
    else if (heuristic == diagonal)			h = max(abs(x - destX), abs(y - destY));
    else if (heuristic == newH)				h = ceil(euc) - euc;

    cost = h + (double)g;

    pathCost = matrix[originX][originY].pathCost + cost;

    matrix[node.first][node.second].h = h;
    matrix[node.first][node.second].g = g;
    matrix[node.first][node.second].cost = cost;
    matrix[node.first][node.second].pathCost = pathCost;
}

void AStar::openNode(UINT x, UINT y, UINT originX, UINT originY, list<NodeState> *changes)
{
    NodeState nodeState;
    double orignalPathCost, newPathCost;
    //UINT predX, predY;

    if (x >= width       || y >= height)		return;
    if (originX >= width || originY >= height)	return;

    if (matrix[x][y].type == WALLNODE || matrix[x][y].type == VISITEDNODE) return;
    else if (matrix[x][y].type == WAYNODE)
    {
        matrix[x][y].type = OPENEDNODE;
        openedNodes.push_back(pair<UINT, UINT>(x, y));
    }
    else if (matrix[x][y].cost < estimateCost(originX, originY, x, y))
    {
        return;
    }
    else if (matrix[x][y].expandCost != 1)
    {
        orignalPathCost = pathCostToNode(matrix[x][y].originX, matrix[x][y].originY, x, y);
        newPathCost = pathCostToNode(originX, originY, x, y);

        //predX = matrix[x][y].originX;
        //predY = matrix[x][y].originY;

        //orignalPathCost = matrix[predX][predY].pathCost;
        //newPathCost = matrix[originX][originY].pathCost;

        if (orignalPathCost < newPathCost) return;
    }


    setOriginNode(x, y, originX, originY);
    calculateCost(pair<UINT, UINT>(x, y));

    if (changes != NULL)
    {
        nodeState.x = x;
        nodeState.y = y;
        nodeState.state = OPENEDNODE;

        originX -= x;
        originY -= y;

        if (originX == 0xffffffff)
        {
            if (originY == 0xffffffff) nodeState.ancestor = UPLEFT;
            else if (originY == 0) nodeState.ancestor = LEFT;
            else nodeState.ancestor = DOWNLEFT;
        }
        else if (originX == 1)
        {
            if (originY == 0xffffffff) nodeState.ancestor = UPRIGHT;
            else if (originY == 0) nodeState.ancestor = RIGHT;
            else nodeState.ancestor = DOWNRIGHT;
        }
        else
        {
            if (originY == 0xffffffff) nodeState.ancestor = UP;
            else nodeState.ancestor = DOWN;
        }

        changes->push_back(nodeState);
    }
}

void AStar::freeNodes()
{
    if (ready)			return;
    if (matrix == NULL)	return;

    for (UINT i = 0; i < width;  i++)
        for (UINT j = 0; j < height; j++)
            if (matrix[i][j].type != WALLNODE)
                matrix[i][j].type = WAYNODE;

    ready = true;
}

void AStar::setOriginNode(UINT x, UINT y, UINT originX, UINT originY)
{
    if (x >= width || y >= height)				return;
    if (originX >= width || originY >= height)	return;

    matrix[x][y].originX = originX;
    matrix[x][y].originY = originY;
}

double AStar::pathCostToNode(UINT x, UINT y, UINT destX, UINT destY)
{
    double euc, h;
    double cost = 0;
    UINT originX, originY;

    h = 0;

    originX = matrix[x][y].originX;
    originY = matrix[x][y].originY;

    while (!(originX == x && originY == y))
    {
        euc = sqrt(pow((double)x - (double)destX, 2.0) + pow((double)y - (double)destY, 2.0));

        if (heuristic == none)					h = 0;
        else if (heuristic == euclidean)		h = euc;
        else if (heuristic == manhattan)		h = abs((double)x - (double)destX) + abs((double)y - (double)destY);
        else if (heuristic == diagonal)			h = max(abs((double)x - (double)destX), abs((double)y - (double)destY));
        else if (heuristic == newH)				h = ceil(euc) - euc;

        cost += h + matrix[x][y].expandCost;

        x = originX;
        y = originY;

        originX = matrix[x][y].originX;
        originY = matrix[x][y].originY;
    }

    return cost;
}

void AStar::search(bool saveChanges)
{
    UINT x, y;
    double minCost;
    pair<UINT, UINT> node;
    pair<UINT, UINT> lastNode;
    pair<UINT, UINT> lastSmoothNode;
    UINT lastSmoothNodeExpandCost = 0; /* FIXME : TODO : is init 0 OK? */


    list<pair<UINT, UINT> >::iterator elem, it;
    list<NodeState> *newMatrixState;
    NodeState nodeState;

    freeNodes();
    path.clear();
    m_smoothPath.clear();
    m_smoothMarginPath.clear();
    matrixsChanges.clear();

    // Protections

    // No matrix for computation
    if (matrix == NULL)									return;

    // Start or End out of map
    if (start.first > width || start.second > height)	return;
    if (end.first > width || end.second > height)		return;

    // Start or End in in a wall ... :-(
    if(matrix[start.first][start.second].type == WALLNODE) return;
    if(matrix[end.first][end.second].type == WALLNODE) return;

    if (start == end)
    {
        path.push_front(start);
        m_smoothPath.push_front(start);
        m_smoothMarginPath.push_front(start);
        return;
    }

    ready = false;

    if (saveChanges) newMatrixState = new list<NodeState>();
    else			 newMatrixState = NULL;

    openedNodes.clear();
    openedNodes.push_front(start);

    x = start.first;
    y = start.second;
    setOriginNode(x, y, x, y);
    matrix[x][y].cost = 0;
    matrix[x][y].pathCost = 0;
    matrix[x][y].type = OPENEDNODE;
    matrix[x][y].g = 0;
    //calculateCost(start);

    while (matrix[end.first][end.second].type != OPENEDNODE)
    {
        minCost = numeric_limits<double>::max();

        if (newMatrixState != NULL) newMatrixState->clear();

        if (openedNodes.size() == 0) return;

        for (it = openedNodes.begin(); it != openedNodes.end(); it++)
        {
            x = it->first;
            y = it->second;

            if (matrix[x][y].cost < minCost)
            {
                minCost = matrix[x][y].cost;
                elem = it;
            }
        }

        x = elem->first;
        y = elem->second;

        matrix[x][y].type = VISITEDNODE;

        if (newMatrixState != NULL)
        {
            nodeState.x = x;
            nodeState.y = y;
            nodeState.state = VISITEDNODE;

            if (matrix[x][y].originX == 0xffffffff)
            {
                if (matrix[x][y].originY == 0xffffffff) nodeState.ancestor = UPLEFT;
                else if (matrix[x][y].originY == 0) nodeState.ancestor = LEFT;
                else nodeState.ancestor = DOWNLEFT;
            }
            else if (matrix[x][y].originX == 1)
            {
                if (matrix[x][y].originY == 0xffffffff) nodeState.ancestor = UPRIGHT;
                else if (matrix[x][y].originY == 0) nodeState.ancestor = RIGHT;
                else nodeState.ancestor = DOWNRIGHT;
            }
            else if (matrix[x][y].originX == 0)
            {
                if (matrix[x][y].originY == 0xffffffff) nodeState.ancestor = UP;
                else if (matrix[x][y].originY == 1) nodeState.ancestor = DOWN;
                else nodeState.ancestor = SELF;
            }

            newMatrixState->push_back(nodeState);
        }

        openedNodes.remove(*elem);

        if (x > 0)
        {
            if (y > 0)			openNode(x - 1, y - 1, x, y, newMatrixState);
            openNode(x - 1, y    , x, y, newMatrixState);
            if (y + 1 < height)	openNode(x - 1, y + 1, x, y, newMatrixState);
        }

        if (y > 0)				openNode(x    , y - 1, x, y, newMatrixState);
        if (y + 1 < height)		openNode(x    , y + 1, x, y, newMatrixState);

        if (x + 1 < width)
        {
            if (y > 0)			openNode(x + 1, y - 1, x, y, newMatrixState);
            openNode(x + 1, y    , x, y, newMatrixState);
            if (y + 1 < height)	openNode(x + 1, y + 1, x, y, newMatrixState);
        }

        if (newMatrixState != NULL)
            matrixsChanges.push_back(*newMatrixState);
    }

    x = end.first;
    y = end.second;

    node.first = matrix[x][y].originX;
    node.second = matrix[x][y].originY;

    while (!(matrix[x][y].originX == x && matrix[x][y].originY == y))
    {
        node.first = x;
        node.second = y;

        path.push_front(node);

        if(m_smoothPath.size() == 0)
        {
            m_smoothPath.push_front(node);
            lastSmoothNode = node;
            lastSmoothNodeExpandCost = matrix[x][y].expandCost;
        }
        else
        {
            // If the cost between the 2 points are differents
            // Check if the new point has not direct sight
            if(((lastSmoothNodeExpandCost + 40) < matrix[x][y].expandCost) || (!lineOfSight(lastSmoothNode.first, lastSmoothNode.second, node.first, node.second)))
            {
                //The new node is not in line sight, so save the last node as last smooth node
                m_smoothPath.push_front(lastNode);
                lastSmoothNode = lastNode;
                lastSmoothNodeExpandCost = matrix[x][y].expandCost;
            }
        }

        lastNode = node;

        x = matrix[node.first][node.second].originX;
        y = matrix[node.first][node.second].originY;
    }
    


    // Last check when smoothing the path
    // Maybe the line of sight to the beginning is not OK
    if(m_smoothPath.size() > 0)
    {
        // Check if the new point has direct sight
        if(!lineOfSight(lastSmoothNode.first, lastSmoothNode.second, start.first, start.second))
        {
            //The start node is not in line sight, so save the last node as last smooth node
            m_smoothPath.push_front(lastNode);
        }
        m_smoothPath.push_front(start);
    }

    // Add intermediate point to avoid risk of collision
    AddMargin();

    return;
}

void AStar::setMatrix(UINT width, UINT height)
{
    destroyMatrix();

    this->width = width;
    this->height = height;

    if (width == 0 || height == 0) return;

    matrix = new SearchNode*[width];

    for (UINT i = 0; i < width; i++)
        matrix[i] = new SearchNode[height];

    path.clear();
    m_smoothPath.clear();
    m_smoothMarginPath.clear();
}

void AStar::setStart(UINT x, UINT y)
{
    if (x <= width && y <= height)
    {
        start.first = x;
        start.second = y;

        path.clear();
        m_smoothPath.clear();
        m_smoothMarginPath.clear();
    }
}

void AStar::setEnd(UINT x, UINT y)
{
    if (x <= width && y <= height)
    {
        end.first = x;
        end.second = y;

        path.clear();
        m_smoothPath.clear();
        m_smoothMarginPath.clear();
    }
}

void AStar::setWall(UINT x, UINT y)
{
    if		(matrix == NULL)			return;
    else if	(x > width || y > height)	return;

    matrix[x][y].type = WALLNODE;
}

void AStar::setWay(UINT x, UINT y, UINT expandCost)
{
    if		(matrix == NULL)			return;
    else if	(x > width || y > height)	return;

    matrix[x][y].type = WAYNODE;
    matrix[x][y].expandCost = expandCost;
}

void AStar::clipPoint(int& x, int& y) const
{
    if(x < 0) 
      x = 0;
    if(!((unsigned int)x < width)) 
      x = width - 1;
    if(y < 0) 
      y = 0;
    if((unsigned int)y >= height) 
      y = height - 1;  
}

void AStar::setNode(UINT x, UINT y, NodeType type, UINT expandCost) 
{
    if		(matrix == NULL)			return;
    else if	(x > width || y > height)	return;

    matrix[x][y].type = type;
    matrix[x][y].expandCost = expandCost;    
};

void AStar::fillDisk(float x, float y, float r, NodeType type, UINT expandCost)
{  
    if(r < 0)
        return;
    int x0 = static_cast<int>(std::floor(x - r));
    int y0 = static_cast<int>(std::floor(y - r));
    int x1 = static_cast<int>(std::ceil(x + r) + 1);
    int y1 = static_cast<int>(std::ceil(y + r) + 1);
    
    clipPoint(x0,y0);
    clipPoint(x1,y1);  
    
    for(unsigned ix=x0; ix < (unsigned int)x1; ix++) 
    {
        for(unsigned iy=y0; iy < (unsigned int)y1; iy++) 
        {
            float dx = ix - x;
            float dy = iy - y;
            if((dx * dx + dy * dy) < r * r)
                setNode(ix, iy, type, expandCost);            
        }
    }
}

void AStar::fillRect(int x1, int y1, int x2, int y2, NodeType type, UINT expandCost)
{
    clipPoint(x1,y1);
    clipPoint(x2,y2);
    
    if(x1 > x2) {
        std::swap(x1, x2);
    };
    if(y1 > y2) {
        std::swap(y1, y2);
    };
    
    for(unsigned ix=x1; ix <= (unsigned int)x2; ix++) 
    {
        for(unsigned iy=y1; iy <= (unsigned int)y2; iy++) 
        {
            setNode(ix, iy, type, expandCost);            
        }
    }
}

#include <iostream>

bool scanlineIntersection(float x1, float y1, float x2, float y2, float x, float& y_int)
{          
    if(fabsf(x2 - x1) >= std::numeric_limits<float>::epsilon())
    {
        float y = y1 + (x-x1)*(y2-y1)/(x2-x1);

        if(y > y1 && y > y2)
            return false;
        if(y < y1 && y < y2)
            return false;
        y_int = y;
        return true;
    } else 
    {
      return false;
    }; 
}



void AStar::fillPoly(float* pts_x,float* pts_y, UINT num_pts, NodeType type, UINT expandCost)
{
    if(num_pts == 0)
        return;
    float x_min = pts_x[0];
    float x_max = pts_x[0];
    
    for(unsigned i=0; i < num_pts; i++)
    {
        if(pts_x[i] > x_max)
            x_max = pts_x[i];
        if(pts_x[i] < x_min)
            x_min = pts_x[i];
    }
    int x1 = static_cast<int>(std::floor(x_min));
    int x2 = static_cast<int>(std::ceil(x_max) + 1);
    
    int x_dummy = 0;
    int y_dummy = 0;
    
    clipPoint(x1,y_dummy);
    clipPoint(x2,y_dummy);
    
    for(int x=x1; x<x2; x++)
    {
        float y_min= std::numeric_limits<float>::infinity();
        float y_max = -std::numeric_limits<float>::infinity();
        
  
        for(unsigned i=0; i<num_pts; i++)
        {
            float y = 0;
            unsigned j = i+1;
            if(j == num_pts)
                j = 0;           
            
            if(scanlineIntersection(pts_x[i], pts_y[i], pts_x[j], pts_y[j], x, y))
            {                
                if(y < y_min)
                    y_min = y;
                if(y > y_max)
                  y_max = y; 
            };            
        };
       if(y_min > y_max)
          continue;
        
      int y1 =  static_cast<int>(std::floor(y_min));     
      int y2 =  static_cast<int>(std::ceil(y_max));
      
      clipPoint(x_dummy,y1);
      clipPoint(x_dummy,y2);
    
      for(int y=y1;y<=y2;y++)
      {
        setNode(x, y, type, expandCost);        
      };
    };
    
    
}    
  

void AStar::setHeuristics(AStarHeuristics heuristic)
{
    if (heuristic >= none && heuristic <= newH)
    {
        this->heuristic = heuristic;
    }
    else
    {
        this->heuristic = euclidean;
    }
}

void AStar::destroyMatrix()
{
    if (matrix != NULL)
    {
        for (UINT i = 0; i < width; i++)
            delete [] matrix[i];
        delete [] matrix;

        matrix = NULL;

        path.clear();
        m_smoothPath.clear();
        m_smoothMarginPath.clear();
        matrixsChanges.clear();
    }
}

NodeType AStar::getNodeType(UINT x, UINT y) const
{
    if		(matrix == NULL)			return NONE;
    else if	(x > width || y > height)	return NONE;

    return matrix[x][y].type;
}

pair<UINT, UINT> AStar::getStart() const
{
    return start;
}

pair<UINT, UINT> AStar::getEnd() const
{
    return end;
}

bool AStar::getDebugArr(char* arr, UINT size) const
{
    if(size < width * height)
        return false;
    for(unsigned ix = 0; ix < width; ix++)
    {
        for(unsigned iy = 0; iy < height; iy++)
        {
            UINT expandCost = matrix[ix][iy].expandCost;
            expandCost = expandCost>25?25:expandCost;
            UINT debugGray = 255-expandCost*10;
            arr[iy + ix * height] = matrix[ix][iy].type == WALLNODE ? 0 : debugGray;
        }
    }
    return true;  
}


// Compute a new path only if necessary
list<pair<UINT, UINT> > AStar::getPathOnlyIfNeed(bool autoPathComputeIfNeed, bool * isNewPath, AStarPathType getPathType)
{
    if(isNewPath != NULL)
    {
        *isNewPath = false;
    }

    if((!IsPathStillValid()) && autoPathComputeIfNeed)
    {
        search(false);

        if(isNewPath != NULL)
        {
            *isNewPath = true;
        }
    }

    return getPath(getPathType);
}




list<list<NodeState> > AStar::getChanges() const
{
    return matrixsChanges;
}

double AStar::estimateCost(UINT x1, UINT y1, UINT x2, UINT y2)
{
    double x, y, destX, destY, cost, h, euc;
    UINT g;
    h = 0;

    x = (float)x2;
    y = (float)y2;
    destX = (float)end.first;
    destY = (float)end.second;

    euc = sqrt(pow(x - destX, 2.0) + pow(y - destY, 2.0));

    if (heuristic == none)					h = 0;
    else if (heuristic == euclidean)		h = euc;
    else if (heuristic == manhattan)		h = abs(x - destX) + abs(y - destY);
    else if (heuristic == diagonal)			h = max(abs(x - destX), abs(y - destY));
    else if (heuristic == newH)				h = ceil(euc) - euc;

    g = matrix[x1][y1].g + matrix[x2][y2].expandCost;

    cost = h + (double)g;

    return cost;
}

void AStar::ClearPath(void)
{
    path.clear();
    m_smoothPath.clear();
    m_smoothMarginPath.clear();
}

bool AStar::IsPathStillValid(void)
{
    bool pathValid = false;

    // No path, the path is invalid
    if(path.size() > 0)
    {
        // By default, the path is valid
        pathValid = true;

        list<pair<UINT, UINT> >::iterator pathIt;

        // Look at each waypoint and check if it is still valid
        // Not on a wall
        for (pathIt = path.begin(); pathIt != path.end(); pathIt++)
        {
            // Oups, wall !!
            if(matrix[pathIt->first][pathIt->second].type == WALLNODE)
            {
                // The path is not valid, and stop seeking
                pathValid = false;
                break;
            }
        }
    }

    return pathValid;
}

bool AStar::lineOfSight(int x0, int y0, int x1, int y1)
{
    int diffX = x1 - x0;
    int diffY = y1 - y0;

    int f = 0;

    int dirX;
    int dirY;

    if (diffY < 0)
    {
        diffY = -diffY;
        dirY = -1;
    }
    else
    {
        dirY = 1;
    }

    if (diffX < 0)
    {
        diffX = -diffX;
        dirX = -1;
    }
    else
    {
        dirX = 1;
    }

    if (diffX >= diffY)
    {
        // Move along the x axis and increment/decrement y when f >= diff.x.
        while(true)
        {
            // Oups, wall !!
            if(matrix[x0][y0].type == WALLNODE)
            {
                return false;
            }

            if(x0 == x1) break;

            // Compute next point to check
            f += diffY;
            if (f >= diffX)
            {
                y0 += dirY;
                f -= diffX;
            }

            x0 += dirX;
        }
    }
    else
    {
        //if (diff.x < diff.y). Move along the y axis and increment/decrement x when f >= diff.y.
        while(true)
        {
            // Oups, wall !!
            if(matrix[x0][y0].type == WALLNODE)
            {
                return false;
            }

            if(y0 == y1) break;

            // Compute next point to check
            f += diffX;
            if (f >= diffY)
            {
                x0 += dirX;
                f -= diffY;
            }

            y0 += dirY;
        }
    }

    return true;
}


void AStar::AddMargin()
{
    // No path, the path is invalid
    if(m_smoothPath.size() > 0)
    {
        //int x0 = start.first;
        //int y0 = start.second;

        // For each waypoint
        for (list<pair<UINT, UINT> >::iterator pathIt = m_smoothPath.begin(); pathIt != m_smoothPath.end(); pathIt++)
        {
            /*
            // Dest waypoint
            int x1 = pathIt->first;
            int y1 = pathIt->second;

            // New waypoint (if needed)
            int newX;
            int newY;

            // Need the dest waypoint to be moved
            if(HasNeighbour(x1, y1, &newX, &newY))
            {
                x1 = newX;
                y1 = newY;
            }

            pair<UINT, UINT> newNode;

            // Avoid no solution infinite loop
            int loopCounter = 0;
            list<pair<UINT, UINT> > chunkOfMarginPath;

            while((loopCounter < 100) && NeedAddMargin(x0, y0, x1, y1, &newX, &newY))
            {
                newNode.first = x0 = newX;
                newNode.second = y0 = newY;
                chunkOfMarginPath.push_back(newNode);
                loopCounter++;
            }

            // Recopy the chunkOfMarginPath
            if((loopCounter < 100) && (chunkOfMarginPath.size() > 0))
            {
                for (list<pair<UINT, UINT> >::iterator chunkOfMarginPathIt = chunkOfMarginPath.begin(); chunkOfMarginPathIt != chunkOfMarginPath.end(); chunkOfMarginPathIt++)
                {
                    m_smoothMarginPath.push_back(*chunkOfMarginPathIt);
                }
            }


            newNode.first = x0 = x1;
            newNode.second = y0 = y1;
            m_smoothMarginPath.push_back(newNode);

            */
            m_smoothMarginPath.push_back(*pathIt);

        }
    }
}


// Goes along the line from 2 points
// Return True if need add a intermediate point
bool AStar::NeedAddMargin(int x0, int y0, int x1, int y1, int *newX, int *newY)
{
    int diffX = x1 - x0;
    int diffY = y1 - y0;

    int f = 0;

    int dirX;
    int dirY;

    if (diffY < 0)
    {
        diffY = -diffY;
        dirY = -1;
    }
    else
    {
        dirY = 1;
    }

    if (diffX < 0)
    {
        diffX = -diffX;
        dirX = -1;
    }
    else
    {
        dirX = 1;
    }

    if (diffX >= diffY)
    {
        // Move along the x axis and increment/decrement y when f >= diff.x.
        while(true)
        {
            // Check neighbourhood
            int avoidNeighPosX;
            int avoidNeighPosY;

            // Has neighbour
            if(HasNeighbour(x0, y0, &avoidNeighPosX, &avoidNeighPosY))
            {
                *newX = avoidNeighPosX;
                *newY = avoidNeighPosY;
                return true;
            }

            if(x0 == x1) break;

            // Compute next point to check
            f += diffY;
            if (f >= diffX)
            {
                y0 += dirY;
                f -= diffX;
            }

            x0 += dirX;
        }
    }
    else
    {
        //if (diff.x < diff.y). Move along the y axis and increment/decrement x when f >= diff.y.
        while(true)
        {
            // Check neighbourhood
            int avoidNeighPosX;
            int avoidNeighPosY;

            // Has neighbour
            if(HasNeighbour(x0, y0, &avoidNeighPosX, &avoidNeighPosY))
            {
                *newX = avoidNeighPosX;
                *newY = avoidNeighPosY;
                return true;
            }

            if(y0 == y1) break;

            // Compute next point to check
            f += diffX;
            if (f >= diffY)
            {
                x0 += dirX;
                f -= diffY;
            }

            y0 += dirY;
        }
    }

    return false;
}


// Return True if has neighbour
// Also return the opposite point to go away from the neighbour
bool AStar::HasNeighbour(int x, int y, int *avoidNeighbourX, int *avoidNeighbourY)
{
    int xp1 = x + 1;
    int xm1 = x - 1;
    int yp1 = y + 1;
    int ym1 = y - 1;

    // Protection
    if((xm1 < 0) || (xp1 >= (int)width) || (ym1 < 0) || (yp1 >= (int)height)) return false;

    bool hasNeighbour = false;

    if(matrix[xm1][ym1].type == WALLNODE)
    {
        *avoidNeighbourX = xp1;
        *avoidNeighbourY = yp1;
        hasNeighbour = true;
    }
    else if(matrix[xm1][y].type == WALLNODE)
    {
        *avoidNeighbourX = xp1;
        *avoidNeighbourY = y;
        hasNeighbour = true;
    }
    else if(matrix[xm1][yp1].type == WALLNODE)
    {
        *avoidNeighbourX = xp1;
        *avoidNeighbourY = ym1;
        hasNeighbour = true;
    }
    else if(matrix[x][ym1].type == WALLNODE)
    {
        *avoidNeighbourX = x;
        *avoidNeighbourY = yp1;
        hasNeighbour = true;
    }
    else if(matrix[x][yp1].type == WALLNODE)
    {
        *avoidNeighbourX = x;
        *avoidNeighbourY = ym1;
        hasNeighbour = true;
    }
    else if(matrix[xp1][ym1].type == WALLNODE)
    {
        *avoidNeighbourX = xm1;
        *avoidNeighbourY = yp1;
        hasNeighbour = true;
    }
    else if(matrix[xp1][y].type == WALLNODE)
    {
        *avoidNeighbourX = xm1;
        *avoidNeighbourY = y;
        hasNeighbour = true;
    }
    else if(matrix[xp1][yp1].type == WALLNODE)
    {
        *avoidNeighbourX = xm1;
        *avoidNeighbourY = ym1;
        hasNeighbour = true;
    }

    return hasNeighbour;
}















