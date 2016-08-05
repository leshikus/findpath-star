#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "findpath.h"

class Map {
protected:
    const unsigned char* pMap;
    const int nWidth, nHeight, nSize;
    
public:
    Map(const unsigned char* _pMap, const int _nWidth, const int _nHeight) :
        pMap(_pMap), nWidth(_nWidth), nHeight(_nHeight), nSize(_nWidth * _nHeight) {
    }
    
    inline int toPos(int x, int y) {
        return x + y * nWidth;
    }
    
    inline int dist(int from, int to) {
        int deltaX = from % nWidth - to % nWidth;
        int deltaY = from / nWidth - to / nWidth;
        
        return abs(deltaX) + abs(deltaY);        
    }
    
    bool canPass(int pos) {
        return pMap[pos] != 1;
    }
};

class MarkedMap: public Map {
    // nodes with known minimal path
    int* pQueue;

    int* pOutBuffer;
    const int nOutBufferSize;
    
    int findNeighbor(const int pos, const int m) {
        if ((pos % nWidth >= 1) && (pDist[pos - 1] == m)) return pos - 1;
        if ((pos % nWidth < nWidth - 1) && (pDist[pos + 1] == m)) return pos + 1;

        if ((pos >= nWidth) && (pDist[pos - nWidth] == m)) return pos - nWidth;
        if ((pos < nSize - nWidth) && (pDist[pos + nWidth] == m)) return pos + nWidth;
        return -1;
    }

public:
    bool isPathFound;
    int pathLength; // -1 if path not found
    int bestDist;
    
    // a node -> (best known distance + 1) * sign
    // sign > 0 distance to the start node
    // sign < 0 distance to the target node
    int* pDist;
    
    MarkedMap(const unsigned char* _pMap, const int _nWidth, const int _nHeight,
        int* _pOutBuffer, const int _nOutBufferSize):
        Map(_pMap, _nWidth, _nHeight), isPathFound(false), pathLength(-1),
        pOutBuffer(_pOutBuffer), nOutBufferSize(_nOutBufferSize) {
            
        pDist = (int*) calloc(nSize, sizeof(int));
        
        // a number of nodes in a queue does not exceed a total number
        // of grid nodes
        pQueue = new int[nSize];
    }

    ~MarkedMap() {
        delete[] pQueue;
        free(pDist);
    }

    void debugDist() {
        if (!DEBUG) return;

        int i, j, k;
        for (j = 0, k = 0; j < nHeight; j++) {
            for(i = 0; i < nWidth; i++) printf("%3d", pDist[k++]);
            printf("\n");
        }
    }

    // joinPos start wave collides into target wave here
    // joinDist distance from joinPos to the start
    void fillPath(int joinPos, int joinDist, int bestDist) {
        debugDist();

        isPathFound = true;
        
        // do nothing if the buffer is not big enough to contain the path
        if (pathLength > nOutBufferSize) return;

        int pos = joinPos;
        for (int d = joinDist; d > 0; d--) {
            pOutBuffer[d - 1] = pos;
            pos = findNeighbor(pos, d);
        }
        pOutBuffer[0] = pos;

        pos = joinPos;
        int td = joinDist - bestDist;
        for (int d = joinDist; d < bestDist; d++, td++) {
            pos = findNeighbor(pos, td);
            pOutBuffer[d] = pos;
        }
    }
};

class NodeQueue {
    MarkedMap map;
    int* queue;
    int* pEnd;
    int* pProcessed;
    int sign;
    int from;
    int to;
    int heur; // heuristics for pProcessed
    
    NodeQueue(MarkedMap _map, int _from, int _to, int* _queue, int _start, int _sign):
        map(_map), queue(_queue), pEnd(_queue + _start), pProcessed(_queue + _start), sign(_sign), from(_from), to(_to) {       
        setDist(from, 1);
        addPos(from);
    }
    
    int getDist(int pos) {
        return map.pDist[pos] * sign;
    }
    
    void setDist(int pos, int d) {
        map.pDist[pos] = d * sign;
    }

    void mark(const int pos, const int m) {
        if (map.isPathFound) return;
        map.debugDist();
        
        if (DEBUG) {
            printf("mark: pos = %i, m = %i\n", pos, m);
        }
        
        if (map.canPass(pos)) return;
        
        int d = getDist(pos);
        if (d < 0) {
            map.fillPath(pos, (sign > 0) ? m - 1 : 1 - d, m - d - 2);
            return;
        }
        
        assert(d <= m); // if d > m have to add a point again?
        if (d > 0) return; // 1 .. m - this is no worse then m, skipping
        // d == 0 here and below
        setDist(pos, m);
        
        // add to the queue (here goes a*)
        addPos(pos, m);
    }

    void addPos(int pos) {
        *pEnd = pos;
        pEnd += sign;
    }    
    
    void addPos(int pos, int d) {
        if (DEBUG) printf("marked %i with dist %i\n", pos, d);
        
        int heurNew = getHeuristics(pos, d);
        if (heurNew < heur) {
            *pEnd = *pProcessed;
            *pProcessed = pos;
            heur = heurNew;
        } else {
            *pEnd = pos;
        }
        pEnd += sign;
    }
    
    int getNextPos() {
        int next = *pProcessed; 
        pProcessed += sign;
        return next;
    }
    
    void markNeighbors(const int pos, const int m) {
        if (pos % nWidth >= 1) mark(pos - 1, m);
        if (pos % nWidth < nWidth - 1) mark(pos + 1, m);

        if (pos >= nWidth) mark(pos - nWidth, m);
        if (pos < nSize - nWidth) mark(pos + nWidth, m);
    }
    
    int getHeuristics(int pos, int d) {
        return dist(pos, to) + d * sign;
    }

public:    
    // returns true if search is completed
    bool search() {
        int pos = getNextNode();
        int d = pDist[pos];
        assert(d);
        
        heur = getHeuristics(pos, d);
        markNeighbors(pos, d + sign);
        
        if (!map.isPathFound) return true;
        
        // stop searching if no more entries to process
        return pProcessed == pCurrent;
    }
};

int FindPath(const int nStartX, const int nStartY,
    const int nTargetX, const int nTargetY,
    const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
    int* pOutBuffer, const int nOutBufferSize) {
        
    if ((nStartX == nTargetX) && (nStartY == nTargetY)) return 0;
    
    MarkedMap map(pMap, nMapWidth, nMapHeight, pOutBuffer, nOutBufferSize);  
    
    int posStart = map.toPos(nStartX, nStartY);
    int posTarget = map.toPos(nTargetX, nTargetY);

    NodeQueue sourceQueue(map, posStart, posTarget, map.queue, 0, 1);
    NodeQueue targetQueue(map, posTarget, posStart, map.queue, map.nSize - 1, -1);
    
    while (true) {
        if (sourceQueue.search()) return map.pathLength;
        if (targetQueue.search()) return map.pathLength;
    }
}

