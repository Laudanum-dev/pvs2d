#include "pvs2d.h"

#include <stdlib.h>

/*
segments have pointer to line they are based on
lines contains segments that are based on this line and tStart and tEnd. tStart < tEnd
bsp nodes should contain the splitting line, left and right childs or leaves indices
portals are segs and have index of leaves to the left and right
building BSP:
    firstly group all given segments onto lines:
        have a stack of lines
        for each new seg iter through all lines and if match then assign this seg to its line and viseversa
        if not then just add new line
    have a list of all segments in the area
    choose any line it has, see the amount of segments it cuts
    choose the line that has minimum cuts
    separate all segment to three lists: to the left of the line, to the right and adjacent
    put all adjacent into the current node
    create nodes for left and right if needed and recursively repeat the process

building portals:
    initially create bunch of portals
    for each node: 
        have a list of adjacent portals to subspace
        if a node is a leaf, then check for errors in lists and fill in portals in list with leaf id
        otherwise
        split the line of node into portals? each node should have boundaries of splitline
        add those portals to left list and right list
        split the portals in the given list to left and right, potentially splitting some of them?
building PVS:
    there are two options of how to do this: 
    first is to use the O(num_of_transparent^2 * num_of_opaque) 
    second is to build a graph of leaf adjacency, where leafs are nodes and visible portals are edges.
    for each leaf launch DFS: firstly add to PVS all the adjacent leafs,
        then on each leaf during DFS, have previous edge, for each edge of the current leaf:
            calculate whether the current edge is visible in intersections of all frustums in stack
            if yes, build 2d view frustum between prev edge and current edge,
                and put it into stack and enter the node, and remove after exiting it
            otherwise skip the edge
    this solution should work for around 
    O(num_of_leaves * (num_of_leaves + num_of_transparent) * num_of_transparent * O(is_segment_in_frustum))
    in the worst case. However consider the fact that most of the times DFS won't go into the branch 
    either because its already calculated or frustum check failed, 
    so the amount of times is_segment_in_frustum is called is minimized significantly
    also this method is cooler
    the Drawback is that we never considering the opaque walls, which, if we use the first method, 
    trim PVS a little bit, so we have slightly more. the Drawdrawback of this Drawback is that it is
    insignificant if we use portal-based frustum culling
*/

#ifndef DBG_ASSERT
#ifdef _DEBUG
#include <stdio.h>
#include <math.h>
#define DBG_ASSERT(x, ...) { if (!(x)) { fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); return -1; }}
#else
#define DBG_ASSERT(x, ...)
#endif
#endif

#define MATCH_TOLERANCE (0.0005f)
// the so called EPS. used to fix some errors that inevitably happen with float arithmetics

static inline char _collinear(int ax, int ay, int bx, int by, int cx, int cy) {
	return (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by) == 0);
};

static inline void _intersect(int ax, int ay, int bx, int by, int cx, int cy, int dx, int dy, int* numerDest, int* denomDest) {
    int nx = bx - ax, ny = by - ay;
    *numerDest = (dy - ay) * nx - (dx - ax) * ny;
    *denomDest = (cx - dx) * ny - (cy - dy) * nx;
};

typedef struct PVS2D_Line {
    int ax, ay, bx, by;
    PVS2D_SegStack* mems;
} PVS2D_Line;

typedef struct PVS2D_Seg {
    PVS2D_Line* line;
    float tStart, tEnd;
    int opq;
} PVS2D_Seg;

typedef struct PVS2D_SegStack {
    PVS2D_Seg* seg;
    PVS2D_SegStack* next;
} PVS2D_SegStack;

typedef struct PVS2D_BSPTreeNode {
    PVS2D_BSPTreeNode* left;
    PVS2D_BSPTreeNode* right;
    unsigned int leftLeaf;
    unsigned int rightLeaf;
    PVS2D_Line* line;
    PVS2D_SegStack* segs;
} PVS2D_BSPTreeNode;

int _buildBSP(PVS2D_BSPTreeNode* cur_node, PVS2D_SegStack* cur_segs) {
    cur_node->left = 0;
    cur_node->right = 0;
    cur_node->leftLeaf = 0;
    cur_node->rightLeaf = 0;
    cur_node->line = 0;
    cur_node->segs = 0;
    
    PVS2D_Seg* rootSeg = 0;
    unsigned int minsplits = -1;        // UINT_MAX
    static int leafIndex = 1;           
    // the index of leaves, tells the amount of leaves compiled.
    // the static is used instead of global variable to reduce junk.
    // also note that leaf indices count from 1 instead of 0.
    // this was done in order to not use uint max for the same thing, 
    // so the leaf 0 is 'invalid' leaf
    
    for (PVS2D_SegStack* rootHead = cur_segs; rootHead != 0; rootHead = rootHead->next) {
        // choose any segment and see how much it splits
        unsigned int splitC = 0;
        int numer, denom;
        for (PVS2D_SegStack* curHead = cur_segs; curHead != 0; curHead = curHead->next) {
            _intersect(
                rootHead->seg->line->ax, rootHead->seg->line->ay, 
                rootHead->seg->line->bx, rootHead->seg->line->by,
                curHead->seg->line->ax, curHead->seg->line->ay, 
                curHead->seg->line->bx, curHead->seg->line->by, &numer, &denom
            );
            if (denom == 0) {
                // parallel. do nothing
            }
            else {
                float t = (float)numer / denom;
                if (t > curHead->seg->tStart + MATCH_TOLERANCE && t < curHead->seg->tEnd - MATCH_TOLERANCE) {
                    splitC++;
                }
                else {
                    // intersect outside of segments. do nothing
                }
            }
        }
        if (splitC < minsplits) {
            minsplits = splitC;
            rootSeg = rootHead->seg;
        }
    }

    // use the min segment and split all segments into right ones, left ones and etc.
    PVS2D_SegStack* segsLeft = 0;
    PVS2D_SegStack* segsRight = 0;
    cur_node->line = rootSeg->line;
    PVS2D_SegStack *curHead, *nextHead = cur_segs;
    while (1) {
        if (nextHead == 0) {
            break;
        }
        curHead = nextHead;
        nextHead = curHead->next;
        // this juncture above was used instead of for loop to allow us modifying curHead->next ptr 
        
        if (curHead->seg->line == rootSeg->line) {
            // they are collinear
            curHead->next = cur_node->segs;
            cur_node->segs = curHead;
            continue;
        }

        int numer, denom;
        _intersect(
            rootSeg->line->ax, rootSeg->line->ay, 
            rootSeg->line->bx, rootSeg->line->by,
            curHead->seg->line->ax, curHead->seg->line->ay, 
            curHead->seg->line->bx, curHead->seg->line->by, &numer, &denom
        );

        if (denom == 0) {
            // they are parallel
            if (numer > 0) {
                // put it to the left
                curHead->next = segsLeft;
                segsLeft = curHead;
            }
            else {
                // put it to the right
                curHead->next = segsRight;
                segsRight = curHead;
            }
        }
        else {
            float t = (float)numer / denom;
            if (t > curHead->seg->tStart + MATCH_TOLERANCE && t < curHead->seg->tEnd - MATCH_TOLERANCE) {
                // it splits the segment
                PVS2D_SegStack* newElem = (PVS2D_SegStack*)malloc(sizeof(PVS2D_SegStack));
                newElem->seg = (PVS2D_Seg*)malloc(sizeof(PVS2D_Seg));
                newElem->seg->tStart = t;
                newElem->seg->tEnd = curHead->seg->tEnd;
                newElem->seg->line = curHead->seg->line;
                newElem->seg->opq = curHead->seg->opq;
                curHead->seg->tEnd = t;
                if (denom > 0) {
                    // tEnd point of curHead is to the left
                    // therefore A point is to the right
                    // since point B is to the left, then newElem should go to the left
                    newElem->next = segsLeft;
                    segsLeft = newElem;
                    curHead->next = segsRight;
                    segsRight = curHead;
                }
                else {
                    newElem->next = segsRight;
                    segsRight = newElem;
                    curHead->next = segsLeft;
                    segsLeft = curHead;
                }
            }
            else {
                // so the line doesn't split the segment
                // therefore we just add it to the appropriate list
                // the problem is that one of its vertices can be on the line 
                // so instead of checking one of its vertices we just use the middle point of seg
                // since both of its vertices can't be on the line, the middle is 
                // guaranteed to be far enough from it
                float middle = (curHead->seg->tStart + curHead->seg->tEnd) / 2;
                // if denom > 0 then order is tEnd -> tStart -> line
                // else order is tStart -> tEnd -> line
                if (denom > 0 && middle > t || denom < 0 && middle < t) {
                    // then segment is to the left
                    curHead->next = segsLeft;
                    segsLeft = curHead;
                }
                else {
                    curHead->next = segsRight;
                    segsRight = curHead;
                }
            }
        }
    }
    // now all segments are sorted to their lists. 
    // time for recursion
    if (segsLeft == 0) {
        // we don't have any segs to the left, therefore its a leaf.
        cur_node->left = 0;
        cur_node->leftLeaf = leafIndex++;
    }
    else {
        PVS2D_BSPTreeNode* newNode = (PVS2D_BSPTreeNode*)malloc(sizeof(PVS2D_BSPTreeNode));
        int rez = _buildBSP(newNode, segsLeft);
        if (rez != 0) return rez;   // error encountered
        cur_node->left = newNode;
        cur_node->leftLeaf = 0;
    }

    if (segsRight == 0) {
        // we don't have any segs to the right, therefore its a leaf
        cur_node->right = 0;
        cur_node->rightLeaf = leafIndex++;
    }
    else {
        PVS2D_BSPTreeNode* newNode = (PVS2D_BSPTreeNode*)malloc(sizeof(PVS2D_BSPTreeNode));
        int rez = _buildBSP(newNode, segsRight);
        if (rez != 0) return rez;   // error encountered
        cur_node->right = newNode;
        cur_node->rightLeaf = 0;
    }
    // nothing seems needs freeing.
    return 0;

};

int PVS2D_BuildBSP(int* segs, unsigned int segsC, PVS2D_BSPTreeNode* rootDest) {
    typedef struct _lstack {
        PVS2D_Line* line;
        struct _lstack* next;
    } _lstack;

    PVS2D_SegStack* prSegs = 0;
    _lstack* prLines = 0;
    for (unsigned int i = 0; i < segsC; i++) {
        int ax = segs[5 * i],
            ay = segs[5 * i + 1],
            bx = segs[5 * i + 2],
            by = segs[5 * i + 3],
            opq = segs[5 * i + 4];
        _lstack* match = 0;
        for (_lstack* top = prLines; top != 0; top = top->next) {
            if (_collinear(ax, ay, bx, by, top->line->ax, top->line->ay) &&
                _collinear(ax, ay, bx, by, top->line->bx, top->line->by)
            ) {
                match = top->line;
                break;
            }
        }
        PVS2D_SegStack* newSeg = (PVS2D_SegStack*)malloc(sizeof(PVS2D_SegStack));
        if (match == 0) {
            _lstack* newLine = (_lstack*)malloc(sizeof(_lstack));
            newLine->line->ax = ax;
            newLine->line->ay = ay;
            newLine->line->bx = bx;
            newLine->line->by = by;
            newLine->line->mems = (PVS2D_SegStack*)malloc(sizeof(PVS2D_SegStack));
            newLine->line->mems->seg = newSeg->seg;
            newLine->line->mems->next = 0;
            newLine->next = prLines;

            newSeg->seg->line = newLine->line;
            newSeg->seg->tStart = 0.0f;
            newSeg->seg->tEnd = 1.0f;
            newSeg->seg->opq = opq;
            newSeg->next = prSegs;
            prSegs = newSeg;
        }
        else {
            float tStart = 0.0f, tEnd = 1.0f;
            if (ax == bx) {
                tStart = (float)(ay - match->line->ay) / (match->line->by - match->line->ay);
                tEnd = (float)(by - match->line->ay) / (match->line->by - match->line->ay);
            }
            else {
                tStart = (float)(ax - match->line->ax) / (match->line->bx - match->line->ax);
                tEnd = (float)(bx - match->line->ax) / (match->line->bx - match->line->ax);
            }
            if (tStart > tEnd) {
                float _t = tStart;
                tStart = tEnd;
                tEnd = _t;
            }
            newSeg->seg->line = match->line;
            PVS2D_SegStack* newEntry = (PVS2D_SegStack*)malloc(sizeof(PVS2D_SegStack));
            newEntry->seg = newSeg->seg;
            newEntry->next = newSeg->seg->line->mems;
            newSeg->seg->line->mems = newEntry;
            newSeg->seg->opq = opq;
            newSeg->seg->tStart = tStart;
            newSeg->seg->tEnd = tEnd;
            newSeg->next = prSegs;
            prSegs = newSeg;
        }
    }
    return _buildBSP(rootDest, prSegs);

};
