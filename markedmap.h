#ifndef MARKEDMAP_H
#define MARKEDMAP_H

#include <assert.h>

class NodeQueue;

class Map {
protected:
    const unsigned char* pMap;

    bool canPass(int pos) const {
        return pMap[pos] == 1;
    }
    
public:
    const int nWidth, nHeight, nSize;
    
    Map(const unsigned char* _pMap, const int _nWidth, const int _nHeight) :
        pMap(_pMap), nWidth(_nWidth), nHeight(_nHeight), nSize(_nWidth * _nHeight) {
    }
    
    inline int toPos(int x, int y) const {
        return x + y * nWidth;
    }
    
    inline int getDist(int from, int to) const {
        int deltaX = from % nWidth - to % nWidth;
        int deltaY = from / nWidth - to / nWidth;
        
        return abs(deltaX) + abs(deltaY);        
    }
    
};

class MarkedMap: public Map {
    int* pOutBuffer;
    const int nOutBufferSize;
    
    int findNeighbor(const int pos, const int d) const {
        if (DEBUG) printf("findNeighbor: pos = %i, d = %i\n", pos, d);
        
        if ((pos % nWidth >= 1) && (pDist[pos - 1] == d)) return pos - 1;
        if ((pos % nWidth < nWidth - 1) && (pDist[pos + 1] == d)) return pos + 1;
        if ((pos >= nWidth) && (pDist[pos - nWidth] == d)) return pos - nWidth;
        if ((pos < nSize - nWidth) && (pDist[pos + nWidth] == d)) return pos + nWidth;
        return -1;
    }

public:
    int pathLen; // -1 if path not found
    int bestDist;

    // a node -> (best known distance + 1) * sign
    // sign > 0 distance to the start node
    // sign < 0 distance to the target node
    int* pDist;
    
    // nodes with known minimal path
    int* pQueue;
    
    MarkedMap(const unsigned char* _pMap, const int _nWidth, const int _nHeight,
        int* _pOutBuffer, const int _nOutBufferSize):
        Map(_pMap, _nWidth, _nHeight), pathLen(-1),
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

    void debugDist() const {
        if (!DEBUG) return;
        if (nWidth > MAX_DEBUG_WIDTH) return;

        int i, j, k;
        for (j = 0, k = 0; j < nHeight; j++) {
            for(i = 0; i < nWidth; i++) printf("%3d", pDist[k++]);
            printf("\n");
        }
    }

    // joinPos start wave collides into target wave here
    // joinDist distance from joinPos to the start
    void fillPath(int joinPos, int joinDist, int bestDist) {
        if (DEBUG) printf("fillPath: joinPos = %i, joinDist = %i, bestDist = %i\n", joinPos, joinDist, bestDist);
        debugDist();
        pathLen = bestDist;
        
        // do nothing if the buffer is not big enough to contain the path
        if (pathLen > nOutBufferSize) return;

        int pos = joinPos;
        for (int d = joinDist; d > 1; d--) {
            pOutBuffer[d - 1] = pos;
            pos = findNeighbor(pos, d);
            if (DEBUG) printf("pOutBuffer[%i] found: %i\n", d - 2, pos);
        }
        pOutBuffer[0] = pos;

        pos = joinPos;
        int td = joinDist - bestDist;
        for (int d = joinDist; d < bestDist; d++, td++) {
            pos = findNeighbor(pos, td);
            if (DEBUG) printf("pOutBuffer[%i] found: %i\n", d, pos);
            pOutBuffer[d] = pos;
        }
    }

    void mark(const int pos, NodeQueue* queue);
    
    void markNeighbors(const int pos, NodeQueue* queue) {
        if (pos % nWidth >= 1) mark(pos - 1, queue);
        if (pos % nWidth < nWidth - 1) mark(pos + 1, queue);
        if (pos >= nWidth) mark(pos - nWidth, queue);
        if (pos < nSize - nWidth) mark(pos + nWidth, queue);
    }
 
};

#endif // MARKEDMAP_H
