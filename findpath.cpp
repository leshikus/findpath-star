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
};

class MarkedMap: public Map {
    int posStart, posTarget;
    
    // point -> best known distance
    // >0 distance to the start point
    // <0 distance to the target point
    int* pDist;
    
    // points with known minimal path
    int* pKnown;
    int* pEnd;
    int* pProcessed;

    // start wave collides into target wave here
    int joinPos;
    // distance from joinPos to the start
    int joinDist;
    
    // heuristics for *pProcessed
    int hCur;
    
public:
    bool isPathFound;
    int bestDist;

    MarkedMap(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY, const unsigned char* _pMap, const int _nWidth, const int _nHeight):
        Map(_pMap, _nWidth, _nHeight), isPathFound(false) {
            
        pDist = (int*) calloc(nSize, sizeof(int));
        pKnown = new int [nSize];
        
        pProcessed = pEnd = pKnown;
        
        posStart = toPos(nStartX, nStartY);
        posTarget = toPos(nTargetX, nTargetY);
        
        pDist[posStart] = 1;
        pDist[posTarget] = -1;
        
        hCur = 0;
        markSourceNeighbors(posStart, 1);
        markTargetNeighbors(posTarget, -1);
    }

    ~MarkedMap() {
        delete[] pKnown;
        free(pDist);
    }

    void markSource(const int pos, const int m) {
        debugDist();
        
        if (DEBUG) {
            printf("markSource: pos = %i, m = %i\n", pos, m);
        }
        
        int d = pDist[pos];        
        if (d < 0) {
            isPathFound = true;
            joinPos = pos;
            joinDist = m;
            bestDist = m - d;
            return;
        }
        if (d >= m) return;
        
        assert(d == 0); // if d != 0 have to add a point again?
        pDist[pos] = m;
        
        // add to the queue (here goes a*)
        addPos(pos, dist(pos, posTarget) + m);
    }

    void markTarget(const int pos, const int m) {
        debugDist();

        if (DEBUG) {
            printf("markSource: pos = %i, m = %i\n", pos, m);
        }
        
        int d = pDist[pos];        
        if (d > 0) {
            isPathFound = true;
            joinPos = pos;
            joinDist = - m;
            bestDist = d - m;
            return;
        }
        if (d <= m) return;
        
        assert(d == 0); // if d != 0 have to add a point again?
        pDist[pos] = m;
        
        // add to the queue (here goes a*)
        addPos(pos, dist(pos, posStart) - m);
    }

    void addPos(int pos, int hNew) {
        if (DEBUG) printf("marked %i with %i\n", pos, hNew);
        if (hNew < hCur) {
            *pEnd = *pProcessed;
            *pProcessed = pos;
            hCur = hNew;
        } else {
            *pEnd = pos;
        }
        pEnd++;        
    }
    
    void markSourceNeighbors(const int pos, const int m) {
        if (pos % nWidth >= 1) markSource(pos - 1, m);
        if (pos % nWidth < nWidth - 1) markSource(pos + 1, m);

        if (pos >= nWidth) markSource(pos - nWidth, m);
        if (pos < nSize - nWidth) markSource(pos + nWidth, m);
    }

    void markTargetNeighbors(const int pos, const int m) {
        if (pos % nWidth >= 1) markTarget(pos - 1, m);
        if (pos % nWidth < nWidth - 1) markTarget(pos + 1, m);

        if (pos >= nWidth) markTarget(pos - nWidth, m);
        if (pos < nSize - nWidth) markTarget(pos + nWidth, m);
    }

    int findNeighbor(const int pos, const int m) {
        if ((pos % nWidth >= 1) && (pDist[pos - 1] == m)) return pos - 1;
        if ((pos % nWidth < nWidth - 1) && (pDist[pos + 1] == m)) return pos + 1;

        if ((pos >= nWidth) && (pDist[pos - nWidth] == m)) return pos - nWidth;
        if ((pos < nSize - nWidth) && (pDist[pos + nWidth] == m)) return pos + nWidth;
        return -1;
    }

    void debugDist() {
        if (!DEBUG) return;

        int i, j, k;
        for (j = 0, k = 0; j < nHeight; j++) {
            for(i = 0; i < nWidth; i++) printf("%3d", pDist[k++]);
            printf("\n");
        }
    }
    
    void processNext() {
        int pos = *pProcessed;
        int d = pDist[pos];
        assert(d);
        
        pProcessed++;
        
        if (d > 0) {
            hCur = dist(pos, posTarget) + d;
            markSourceNeighbors(pos, d + 1);
        } else {
            hCur = dist(pos, posStart) - d;
            markTargetNeighbors(pos, d - 1);
        }
    }
    
    // pOutBuffer[sourceDist] == pos
    void fillPath(int* pOutBuffer) {
        if (DEBUG) {
            int i, j, k;
            for (j = 0, k = 0; j < nHeight; j++) {
                for(i = 0; i < nWidth; i++) printf("%2d", pDist[k++]);
                printf("\n");
            }
        }
        
        int pos = joinPos;     
        for (int d = joinDist - 1; d > 0; d--) {
            pOutBuffer[d] = pos;
            pos = findNeighbor(pos, d);
        }
        pOutBuffer[0] = pos;

        pos = joinPos;
        int td = 1 + joinDist - bestDist;
        for (int d = joinDist; d < bestDist - 1; d++, td++) {
            pos = findNeighbor(pos, td);
            pOutBuffer[d] = pos;
        }
        pOutBuffer[bestDist - 1] = posTarget;
    }
    
    bool hasNext() {
        return pProcessed < pEnd;
    }
};


int FindPath(const int nStartX, const int nStartY,
    const int nTargetX, const int nTargetY,
    const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
    int* pOutBuffer, const int nOutBufferSize) {
        
    if ((nStartX == nTargetX) && (nStartY == nTargetY)) return 0;
        
    MarkedMap map(nStartX, nStartY, nTargetX, nTargetY, pMap, nMapWidth, nMapHeight);
    
    bool isAdded;
    do {
        map.processNext();
        if (map.isPathFound) {
            if (map.bestDist >= nOutBufferSize) return map.bestDist;
            if (nOutBufferSize <= 0) return 0;
            map.fillPath(pOutBuffer);
            return map.bestDist;
        }
    } while (map.hasNext());
    return -1;
}
