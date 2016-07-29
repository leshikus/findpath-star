#include <stdio.h>
#include <stdlib.h>
#include "findpath.h"

class Map {
protected:
    const unsigned char* pMap;
    const int nWidth, nHeight, nSize;
    
public:
    Map(const unsigned char* _pMap, const int _nWidth, const int _nHeight) :
        pMap(_pMap), nWidth(_nWidth), nHeight(_nHeight), nSize(_nWidth * _nHeight) {
    }
    
    inline int pos(int x, int y) {
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
    
public:
    bool isPathFound;
    int bestDist;

    MarkedMap(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY, const unsigned char* _pMap, const int _nWidth, const int _nHeight):
        Map(_pMap, _nWidth, _nHeight), isPathFound(false) {
            
        pDist = new int[nSize];
        pKnown = new int [nSize];
        
        pProcessed = pEnd = pKnown;
        
        posStart = pos(nStartX, nStartY);
        posTarget = pos(nTargetX, nTargetY);
        
        pDist[posStart] = 1;
        pDist[posTarget] = -1;
        
        markSourceNeighbors(posStart, 1);
        markTargetNeighbors(posTarget, -1);
    }

    ~MarkedMap() {
        delete[] pKnown;
        delete[] pDist;
    }

    void markSource(const int pos, const int m) {
        if (DEBUG) {
            printf("markSource: pos = %i, d = %i\n", pos, m);
        }
        
        int d = pDist[pos];        
        if (d < 0) {
            isPathFound = true;
            joinPos = pos;
            joinDist = m;
            return;
        }
        if (d >= m) return;
        d != 0 add a point again?

        pDist[pos] = m;
        if (DEBUG) printf("marked at %i\n", pEnd - pEquidistantSet);

        pMarkedMap[pos] = d;
        
        add
        // add to the queue (here goes a*)
        if (better(pos, *pProcessed)) {
            *pEnd = *pProcessed;
            *pProcessed = pos;
        } else {
            *pEnd = pos;
        }
        pEnd++;
    }

    void markTarget(const int pos, const int m) {
        if (DEBUG) {
            printf("markTarget: pos = %i, d = %i\n", pos, m);
        }

        int d = pDist[pos];        
        if (d > 0) {
            isPathFound = true;
            bestDist = d - m;
            pDist[] = d;
            return;
        }

        // is completed?
        if (pos == nTarget) {
            isPathFound = true;
            fillPath();
            return;
        }
        
        if (pMap[pos] == 0) return;
        
        int d = pDist[pos];
        
        if (d == 0) {
        } else 
         oldDist == 0)< 0) && (d < 0)) {
            if (d <= oldDist) return;
            
        } else if (){
            
        }
        
        // mark
        pDist[pos] = d;
        
        if (pMarkedMap[pos] != 0) return;
        
        if (debug) printf("marked at %i\n", pEnd - pEquidistantSet);

        pMarkedMap[pos] = d;
        
        // add to the queue (here goes a*)
        if (better(pos, *pProcessed)) {
            *pEnd = *pProcessed;
            *pProcessed = pos;
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
        if ((pos % nWidth >= 1) && (pMarkedMap[pos - 1] == m)) return pos - 1;
        if ((pos % nWidth < nWidth - 1) && (pMarkedMap[pos + 1] == m)) return pos + 1;

        if ((pos >= nWidth) && (pMarkedMap[pos - nWidth] == m)) return pos - nWidth;
        if ((pos < nSize - nWidth) && (pMarkedMap[pos + nWidth] == m)) return pos + nWidth;
        return -1;
    }
    
    void processNext() {
        int pos = *pProcessed;
        int d = pDist[pos];
        assert(d);
        pProcessed++;
        
        if (d > 0) markSourceNeighbors(pos, d + 1)
            else markTargetNeighbors(pos, d - 1);

        if (DEBUG) {
            int i, j, k;
            for (j = 0, k = 0; j < nHeight; j++) {
                for(i = 0; i < nWidth; i++) printf("%3d", pDist[k++]);
                printf("\n");
            }
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
        for (int d = sourceDist - 1; d > 0; d--) {
            pOutBuffer[d] = pos;
            pos = findNeighbor(pos, d);
        }
        pOutBuffer[0] = p;

        pos = joinPos;
        int td = 1 + sourceDist - bestDist;
        for (int d = sourceDist; d < bestDist - 1; d++, td++) {
            p = findNeighbor(p, td);
            pOutBuffer[d] = p;
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
