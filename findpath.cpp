#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "findpath.h"
#include "markedmap.h"


class NodeQueue {
    MarkedMap* map;
    const int* queue;
    const int* pStart;
    
    int* pEnd;
    int* pProcessed;
    const int sign;
    const int from;
    const int to;
    
    // for pProcessed node
    int dist; // actual distance to "from" node + 1
    int heur; // heuristics (dist + predicted distance to "to" node)
    
    int getDist(int pos) const {
        return map->pDist[pos] * sign;
    }
    
    void setDist(int pos, int d) {
        map->pDist[pos] = d * sign;
    }
   
    int getNext() {
        int next = *pProcessed; 
        pProcessed += sign;
        return next;
    }
    
    int getHeuristics(int pos, int d) {
        return map->getDist(pos, to) + d;
    }

public:    
    NodeQueue(MarkedMap* _map, int _from, int _to, int* _queue, int _start, int _sign):
        map(_map), queue(_queue), pStart(_queue + _start), pEnd(_queue + _start), pProcessed(_queue + _start),
        sign(_sign), from(_from), to(_to) {
        setDist(from, 1);

        *pEnd = from;
        pEnd += sign;
    }
    
    // returns true if search is completed
    bool search() {
        int pos = getNext();
        dist = getDist(pos);
        assert(dist);
        
        heur = getHeuristics(pos, dist);
        
        dist++;
        map->markNeighbors(pos, this);

        if (map->pathLen != -1) return true;
        
        if (DEBUG) printf("pProcessed == %p, pEnd == %p\n", pProcessed, pEnd);
        
        // stop searching if no more entries to process
        return pProcessed == pEnd;
    }

    bool isGood(const int pos) {
        int d = getDist(pos);
        if (DEBUG) printf("isGood: d = %i, dist = %i\n", d, dist);
        
        if (d < 0) {
            map->fillPath(pos, (sign > 0) ? dist - 1 : - d - 1, dist - d - 2);
            return false;
        }
        
        assert(d <= dist); // if d > dist have to add a point again?
        return d == 0; // 1 .. dist - d is no worse then dist, skipping
    }

    void add(int pos) {
        // d == 0, never been here
        setDist(pos, dist);
        
        if (DEBUG) printf("marked %i with dist %i\n", pos, dist);
        
        int heurNew = getHeuristics(pos, dist);
        if (heurNew < heur) {
            *pEnd = *pProcessed;
            *pProcessed = pos;
            heur = heurNew;
        } else {
            *pEnd = pos;
        }
        pEnd += sign;
    }

    int size() const {
        return (pEnd - pStart) * sign;
    }
};

void MarkedMap::mark(const int pos, NodeQueue* queue) {
    if (pathLen != -1) return;
    debugDist();
    
    if (DEBUG) {
        printf("mark: pos = %i\n", pos);
    }
    
    if (!canPass(pos)) return;        
    if (queue->isGood(pos)) queue->add(pos);
}

int FindPath(const int nStartX, const int nStartY,
    const int nTargetX, const int nTargetY,
    const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
    int* pOutBuffer, const int nOutBufferSize) {
        
    if ((nStartX == nTargetX) && (nStartY == nTargetY)) {
        if (nOutBufferSize >= 1) pOutBuffer[0] = nTargetY * nMapWidth + nTargetX;
        return 0;
    }
    
    MarkedMap map(pMap, nMapWidth, nMapHeight, pOutBuffer, nOutBufferSize);  
    
    int posStart = map.toPos(nStartX, nStartY);
    int posTarget = map.toPos(nTargetX, nTargetY);

    NodeQueue sourceQueue(&map, posStart, posTarget, map.pQueue, 0, 1);
    NodeQueue targetQueue(&map, posTarget, posStart, map.pQueue, map.nSize - 1, -1);

    for (int i = 0; i < map.nSize; i += 2) {
        if (sourceQueue.search()) break;
        if (targetQueue.search()) break;
    }
    
    if (STATS) printf("Nodes: source = %i, target = %i, total = %i\n", sourceQueue.size(), targetQueue.size(), sourceQueue.size() + targetQueue.size());
    return map.pathLen;
}

