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
	
	float tSplitStart;
	float tSplitEnd;
	struct PVS2D_PortalStack* portals;
} PVS2D_BSPTreeNode;

typedef struct PVS2D_PortalStack {
	struct PVS2D_Portal* portal;
	struct PVS2D_PortalStack* next;
	int left;
} PVS2D_PortalStack;

typedef struct PVS2D_Portal {
	PVS2D_Seg seg;
	unsigned int leftLeaf;
	unsigned int rightLeaf;
} PVS2D_Portal;

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

int PVS2D_BuildPortals(
	PVS2D_BSPTreeNode* root
);

#endif