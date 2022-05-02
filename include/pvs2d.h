#ifndef PVS2D_H
#define PVS2D_H


// --------------------------------------------------------
//                       STRUCTURES
// --------------------------------------------------------

typedef struct PVS2D_Line {
    int ax, ay, bx, by;
    struct PVS2D_SegStack* mems;
} PVS2D_Line;

typedef struct PVS2D_Seg {
    struct PVS2D_Line* line;
    float tStart, tEnd;
    int opq;
} PVS2D_Seg;

typedef struct PVS2D_SegStack {
    struct PVS2D_Seg* seg;
    struct PVS2D_SegStack* next;
} PVS2D_SegStack;

typedef struct PVS2D_BSPTreeNode {
    struct PVS2D_BSPTreeNode* left;
    struct PVS2D_BSPTreeNode* right;
    unsigned int leftLeaf;
    unsigned int rightLeaf;
    struct PVS2D_Line* line;
    struct PVS2D_SegStack* segs;
} PVS2D_BSPTreeNode;

// --------------------------------------------------------
//                  INTERFACE FUNCTIONS
// --------------------------------------------------------

int PVS2D_BuildBSPTree(
    int* segs, unsigned int segsC,
    PVS2D_BSPTreeNode* rootDest
);

unsigned int PVS2D_FindLeafOfPoint(
    PVS2D_BSPTreeNode* root, 
    float x, float y
);


#endif