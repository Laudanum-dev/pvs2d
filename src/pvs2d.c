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
#define DBG_ASSERT(x, ret, ...) { if (!(x)) { fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); return ret; }}
#else
#define DBG_ASSERT(x, ...)
#endif
#endif

const double MATCH_TOLERANCE = 0.0625f;
// the so called EPS. used to fix some errors that inevitably happen with float arithmetics

static inline char _collinear(int ax, int ay, int bx, int by, int cx, int cy) {
	return (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by) == 0);
};

static inline void _intersect(int ax, int ay, int bx, int by, int cx, int cy, int dx, int dy, int* numerDest, int* denomDest) {
	int nx = bx - ax, ny = by - ay;
	*numerDest = nx * (cy - ay) - ny * (cx - ax);
	*denomDest = nx * (cy - dy) - ny * (cx - dx);
};

static inline void _intersectF(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy, double* numerDest, double* denomDest) {
	double nx = bx - ax, ny = by - ay;
	*numerDest = nx * (cy - ay) - ny * (cx - ax);
	*denomDest = nx * (cy - dy) - ny * (cx - dx);
};

// returns 0 if vector (ax, ay) is on the right side of (bx, by)
// returns anything else if not
static inline unsigned int _side(double ax, double ay, double bx, double by) {
	return (ax * by > ay * bx);
}

int _cropSplitSegs(PVS2D_BSPTreeNode* node, PVS2D_Line* line, int left) {
	if (node->line == line) {
		DBG_ASSERT(0, -1, "This should not have happened...");
	}
	int numer, denom;
	_intersect(
		line->ax, line->ay, line->bx, line->by,
		node->line->ax, node->line->ay, node->line->bx, node->line->by,
		&numer, &denom
	);
	if (denom != 0) {
		// crop only if not parallel.
		double t = (double)numer / denom;
		if (left) {
			// crop the splitseg so it is to the left
			if (denom > 0) {
				node->tSplitEnd = min(t, node->tSplitEnd);
			}
			else {
				node->tSplitStart = max(t, node->tSplitStart);
			}
		}
		else {
			// crop the splitseg so it is to the right
			if (denom > 0) {
				node->tSplitStart = max(t, node->tSplitStart);
			}
			else {
				node->tSplitEnd = min(t, node->tSplitEnd);
			}
		}
	}
	if (node->left) {
		int rez = _cropSplitSegs(node->left, line, left);
		if (rez) return rez;
	}
	if (node->right) {
		int rez = _cropSplitSegs(node->right, line, left);
		if (rez) return rez;
	}
	return 0;
}

enum _side {
	SIDE_COL,			// collinear
	SIDE_L_PARAL,		// parallel, is on the left
	SIDE_R_PARAL,		// parallel, is on the right
	SIDE_S_FL,			// faces left, splitted
	SIDE_S_FR,			// faces right, splitted
	SIDE_L_FL,			// faces left, is on the left
	SIDE_L_FR,			// faces right, is on the left
	SIDE_R_FL,			// faces left, is on the right
	SIDE_R_FR			// faces right, is on the right
};

// tells which side of line the segment is on, as well as what orientation it has.
// the orientation is being treated as if we would stand in line's point A looked 
// on the point B also, if the those bits aren't 00, then the function can output the 
// parameter of point on the segment's line of where the collision happens.
char _split(PVS2D_Line* line, PVS2D_Seg* seg, double* tDest) {
	if (seg->line == line) {
		// collinear.
		return SIDE_COL;
	}
	int numer, denom;
	_intersect(
		line->ax, line->ay, line->bx, line->by,
		seg->line->ax, seg->line->ay, seg->line->bx, seg->line->by,
		&numer, &denom
	);
	if (denom == 0) {
		// parallel.
		if (numer == 0) {
			// they are still collinear
			// this should probably be a warning
			return SIDE_COL;
		}
		else if (numer > 0) {
			// to the left
			return SIDE_L_PARAL;
		}
		else {
			// to the right
			return SIDE_R_PARAL;
		}
	}
	else {
		// not parallel. calculate the split point
		double t = (double)numer / denom;
		if (tDest) *tDest = t;
		
		if (t > seg->tStart + MATCH_TOLERANCE && t < seg->tEnd - MATCH_TOLERANCE) {
			// splits the segment
			if (denom < 0) {
				// faces left
				return SIDE_S_FL;
			}
			else {
				// faces right
				return SIDE_S_FR;
			}
		}
		else {
			// doesn't split the segment.
			// in order to get more accurate result, we will use middle point of the segment
			// to determine the side it is on
			double middle = (seg->tStart + seg->tEnd) / 2;
			// we need to take in account the orientation of the segment in order for our magic to work
			// the rest here are mostly self-explainatory
			if (denom < 0) {
				if (middle > t) {
					return SIDE_L_FL;
				}
				else {
					return SIDE_R_FL;
				}
			}
			else {
				if (middle > t) {
					return SIDE_R_FR;
				}
				else {
					return SIDE_L_FR;
				}
			}
		}
	}

};

int _buildBSP(PVS2D_BSPTreeNode* cur_node, PVS2D_SegStack* cur_segs, unsigned int* leafIndex) {
	DBG_ASSERT(cur_node, -1, "cur_node can't be NULL (node must be allocated before calling the function)")
	DBG_ASSERT(cur_segs, -1, "cur_segs can't be NULL (segment array can't have 0 segments)");
	cur_node->left = 0;
	cur_node->right = 0;
	cur_node->leftLeaf = 0;
	cur_node->rightLeaf = 0;
	cur_node->line = 0;
	cur_node->segs = 0;
	cur_node->tSplitStart = -INFINITY;
	cur_node->tSplitEnd = INFINITY;
	cur_node->portals = 0;


	PVS2D_Seg* rootSeg = 0;
	unsigned int minsplits = -1;        // UINT_MAX
	// the index of leaves, tells the amount of leaves compiled.
	// the static is used instead of global variable to reduce junk.
	// this was done in order to not use uint max for the same thing, 
	// so the leaf 0 is 'invalid' leaf

	for (PVS2D_SegStack* rootHead = cur_segs; rootHead != 0; rootHead = rootHead->next) {
		// choose any segment and see how much it splits
		unsigned int splitC = 0;
		for (PVS2D_SegStack* curHead = cur_segs; curHead != 0; curHead = curHead->next) {
			char side = _split(rootHead->seg->line, curHead->seg, 0);
			if (side == SIDE_S_FL || side == SIDE_S_FR) {
				// it splits it
				splitC++;
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
	PVS2D_SegStack* curHead, * nextHead = cur_segs;
	while (1) {
		if (nextHead == 0) {
			break;
		}
		curHead = nextHead;
		nextHead = curHead->next;
		// this juncture above was used instead of for loop to allow us modifying curHead->next ptr 

		double t;
		char side = _split(rootSeg->line, curHead->seg, &t);

		switch (side) {
		case SIDE_COL:;
			curHead->next = cur_node->segs;
			cur_node->segs = curHead;
			break;
		case SIDE_L_PARAL:;
		case SIDE_L_FL:;
		case SIDE_L_FR:;
			curHead->next = segsLeft;
			segsLeft = curHead;
			break;
		case SIDE_R_PARAL:;
		case SIDE_R_FL:;
		case SIDE_R_FR:;
			curHead->next = segsRight;
			segsRight = curHead;
			break;
		case SIDE_S_FL:;
		case SIDE_S_FR:;
			PVS2D_SegStack* newElem = (PVS2D_SegStack*)malloc(sizeof(PVS2D_SegStack));
			DBG_ASSERT(newElem, -1, "Failed to allocate new seg stack node");
			*newElem = *curHead;
			newElem->seg = (PVS2D_Seg*)malloc(sizeof(PVS2D_Seg));
			DBG_ASSERT(newElem->seg, -1, "Failed to allocate new segment");
			*newElem->seg = *curHead->seg;
			newElem->seg->tStart = t;
			curHead->seg->tEnd = t;
			if (side == SIDE_S_FL) {
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
			break;
		default:
			// something is wrong here
			break;
		}

	}
	// now all segments are sorted to their lists. 
	// now we also have to calculate tSplitStart and tSplitEnd
	// this is done by each node descending its children and "cutting" their split segment by itself
	// time for recursion
	if (segsLeft == 0) {
		// we don't have any segs to the left, therefore its a leaf.
		cur_node->left = 0;
		cur_node->leftLeaf = (*leafIndex)++;
	}
	else {
		PVS2D_BSPTreeNode* newNode = (PVS2D_BSPTreeNode*)malloc(sizeof(PVS2D_BSPTreeNode));
		DBG_ASSERT(newNode, -1, "Failed to allocate new BSP tree node");
		int rez = _buildBSP(newNode, segsLeft, leafIndex);
		if (rez) return rez;   // error encountered
		cur_node->left = newNode;
		cur_node->leftLeaf = 0;
		// crop left children
		rez = _cropSplitSegs(newNode, cur_node->line, 1);
		if (rez) return rez;	// error encountered
	}

	if (segsRight == 0) {
		// we don't have any segs to the right, therefore its a leaf
		cur_node->right = 0;
		cur_node->rightLeaf = (*leafIndex)++;
	}
	else {
		PVS2D_BSPTreeNode* newNode = (PVS2D_BSPTreeNode*)malloc(sizeof(PVS2D_BSPTreeNode));
		DBG_ASSERT(newNode, -1, "Failed to allocate new BSP tree node");
		int rez = _buildBSP(newNode, segsRight, leafIndex);
		if (rez) return rez;   // error encountered
		cur_node->right = newNode;
		cur_node->rightLeaf = 0;
		// crop right children
		rez = _cropSplitSegs(newNode, cur_node->line, 0);
		if (rez) return rez;	// error encountered
	}
	// nothing seems needs freeing.

	return 0;

};

int PVS2D_BuildBSPTree(int* segs, unsigned int segsC, PVS2D_BSPTreeNode* rootDest) {
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
				match = top;
				break;
			}
		}
		PVS2D_SegStack* newSeg = (PVS2D_SegStack*)malloc(sizeof(PVS2D_SegStack));
		DBG_ASSERT(newSeg, -1, "Failed to allocate new segment stack node");
		newSeg->seg = (PVS2D_Seg*)malloc(sizeof(PVS2D_Seg));
		DBG_ASSERT(newSeg->seg, -1, "Failed to allocate new segment");
		double tStart = 0.0, tEnd = 1.0;
		if (match == 0) {
			_lstack* newLine = (_lstack*)malloc(sizeof(_lstack));
			DBG_ASSERT(newLine, -1, "Failed to allocate new line into stack");
			newLine->line = (PVS2D_Line*)malloc(sizeof(PVS2D_Line));
			DBG_ASSERT(newLine->line, -1, "Failed to allocate new line in stack node");
			newLine->line->ax = ax;
			newLine->line->ay = ay;
			newLine->line->bx = bx;
			newLine->line->by = by;
			newLine->line->mems = (PVS2D_SegStack*)malloc(sizeof(PVS2D_SegStack));
			DBG_ASSERT(newLine->line->mems, -1, "Failed to allocate new segment into lines's stack");
			newLine->line->mems->seg = newSeg->seg;
			newLine->line->mems->next = 0;
			newLine->next = prLines;
			prLines = newLine;

			newSeg->seg->line = newLine->line;
		}
		else {
			if (ax == bx) {
				tStart = (double)(ay - match->line->ay) / (match->line->by - match->line->ay);
				tEnd = (double)(by - match->line->ay) / (match->line->by - match->line->ay);
			}
			else {
				tStart = (double)(ax - match->line->ax) / (match->line->bx - match->line->ax);
				tEnd = (double)(bx - match->line->ax) / (match->line->bx - match->line->ax);
			}
			if (tStart > tEnd) {
				double _t = tStart;
				tStart = tEnd;
				tEnd = _t;
			}
			newSeg->seg->line = match->line;
			PVS2D_SegStack* newEntry = (PVS2D_SegStack*)malloc(sizeof(PVS2D_SegStack));
			DBG_ASSERT(newEntry, -1, "Failed to allocate new entry to global seg stack");
			newEntry->seg = newSeg->seg;
			newEntry->next = match->line->mems;
			match->line->mems = newEntry;
		}
		newSeg->seg->tStart = tStart;
		newSeg->seg->tEnd = tEnd;
		newSeg->seg->opq = opq;
		newSeg->next = prSegs;
		prSegs = newSeg;
	}
	// free line stack (but not the lines themselves)
	for (_lstack* prLine = prLines; prLine != 0;) {
		_lstack* next = prLine->next;
		free(prLine);
		prLine = next;
	}
	unsigned int leafIndex = 0;
	return _buildBSP(rootDest, prSegs, &leafIndex);

};

unsigned int PVS2D_FindLeafOfPoint(PVS2D_BSPTreeNode* root, double x, double y) {
	if (_side(
		root->line->bx - root->line->ax,
		root->line->by - root->line->ay,
		x - root->line->ax,
		y - root->line->ay)
	) {
		// its on the left
		return (root->left) ? PVS2D_FindLeafOfPoint(root->left, x, y) : root->leftLeaf;
	}
	else {
		// its on the right
		return (root->right) ? PVS2D_FindLeafOfPoint(root->right, x, y) : root->rightLeaf;
	}
}

typedef struct _pairfc {
	double p;
	signed char d;
} _pairfc;

int _pairfc_cmp(const void* a, const void* b) {
	double ap = ((_pairfc*)a)->p;
	double bp = ((_pairfc*)b)->p;
	if (fabs(ap - bp) < MATCH_TOLERANCE) {
		signed char ad = ((_pairfc*)a)->d;
		signed char bd = ((_pairfc*)b)->d;
		if (ad < bd) return 1;
		if (ad > bd) return -1;
		return 0;
	}
	else {
		if (ap < bp) return -1;
		if (ap > bp) return 1;
		return 0;
	}
}

// converts node's segments into portals that node contains.
// it will return the pointer to the stack of created portals
// or 0 if errors happened
PVS2D_PortalStack* _portalsOfNode(PVS2D_BSPTreeNode* node) {
	int segsC = 0;
	for (PVS2D_SegStack* curSeg = node->segs; curSeg != 0; curSeg = curSeg->next) {
		if (curSeg->seg->opq)
			segsC++;
	}
	_pairfc* th = calloc(segsC * 2 + 2, sizeof(_pairfc));
	DBG_ASSERT(th, 0, "Failed to initialize scanline array");
	segsC = 0;
	for (PVS2D_SegStack* curSeg = node->segs; curSeg != 0; curSeg = curSeg->next) {
		if (curSeg->seg->opq) {
			th[segsC].p = curSeg->seg->tStart;
			th[segsC++].d = 1;
			th[segsC].p = curSeg->seg->tEnd;
			th[segsC++].d = -1;
		}
	}
	th[segsC].p = node->tSplitStart;
	th[segsC++].d = -1;
	th[segsC].p = node->tSplitEnd;
	th[segsC++].d = 1;
	qsort(th, segsC, sizeof(_pairfc), _pairfc_cmp);
	int l = 1;
	double prevSeg = NAN;
	PVS2D_PortalStack* portals = 0;
	for (int i = 0; i < segsC; i++) {
		if (i == 0 && th[i].d == 1) {
			// we are starting with opaque segment
			prevSeg = th[i].p;
		}
		if (i == segsC - 1 && th[i].d == -1) {
			// we are ending with opaque segment
			if (isnan(prevSeg)) {
				// "outside" segment. happens when there is transparent portal at tSplitStart
				prevSeg = th[0].p;
			}
			PVS2D_PortalStack* newElem = (PVS2D_PortalStack*)malloc(sizeof(PVS2D_PortalStack));
			DBG_ASSERT(newElem, 0, "Failed to create new portal stack element");
			newElem->portal = (PVS2D_Portal*)malloc(sizeof(PVS2D_Portal));
			DBG_ASSERT(newElem->portal, 0, "Failed to create new portal");
			newElem->portal->seg.line = node->line;
			newElem->portal->seg.opq = 1;
			newElem->portal->seg.tStart = prevSeg;
			newElem->portal->seg.tEnd = th[i].p;
			newElem->next = portals;
			portals = newElem;
			prevSeg = th[i].p;
			break;
		}
		if (th[i].d == 1) {
			if (l == 0) {
				// stop previous transparent portal and put it into stack
				PVS2D_PortalStack* newElem = (PVS2D_PortalStack*)malloc(sizeof(PVS2D_PortalStack));
				DBG_ASSERT(newElem, 0, "Failed to create new portal stack element");
				newElem->portal = (PVS2D_Portal*)malloc(sizeof(PVS2D_Portal));
				DBG_ASSERT(newElem->portal, 0, "Failed to create new portal");
				newElem->portal->seg.line = node->line;
				newElem->portal->seg.opq = 0;
				newElem->portal->seg.tStart = prevSeg;
				newElem->portal->seg.tEnd = th[i].p;
				newElem->next = portals;
				portals = newElem;
				prevSeg = th[i].p;
			}
			l++;
		}
		if (th[i].d == -1) {
			if (l == 1) {
				// stop previous opaque portal and put it into stack, if it was not portal from "outside"
				if (isnan(prevSeg)) {
					// "outside" segment. happens when there is transparent portal at tSplitStart
					prevSeg = th[i].p;
					l--;
					continue;
				}
				PVS2D_PortalStack* newElem = (PVS2D_PortalStack*)malloc(sizeof(PVS2D_PortalStack));
				DBG_ASSERT(newElem, 0, "Failed to create new portal stack element");
				newElem->portal = (PVS2D_Portal*)malloc(sizeof(PVS2D_Portal));
				DBG_ASSERT(newElem->portal, 0, "Failed to create new portal");
				newElem->portal->seg.line = node->line;
				newElem->portal->seg.opq = 1;
				newElem->portal->seg.tStart = prevSeg;
				newElem->portal->seg.tEnd = th[i].p;
				newElem->next = portals;
				portals = newElem;
				prevSeg = th[i].p;
			}
			l--;
		}
		// we could put here some checks like correct brackets sequence but nah
	}
	// i hope there is no need to check for closing the sequence.
	free(th);
	return portals;
}

// the function should not modify the order of elements in adjacents, 
// but can insert elements
int _buildPortals(PVS2D_BSPTreeNode* node, PVS2D_PortalStack* adjacent) {
	// adjacent, if provided, must be circular linked list, meaning that it's "last" element
	// should just point to the "first", as well as, since it forms an array of 
	// segment that enclose a certain area, they must be present in counter-clockwise order.

	// first step - split adjacents into left and right ones. since this thing is
	// present in continous order, there are two nodes, from which start left ones and right ones
	PVS2D_PortalStack* firstL = 0;
	PVS2D_PortalStack* firstR = 0;
	
	PVS2D_PortalStack* adjCur = 0;
	PVS2D_PortalStack* adjNxt = adjacent;
	PVS2D_PortalStack* adjNew = 0;
	int prside = -1;
	int loop = 0;
	while (1) {
		if (!adjNxt) break;		// adjacents is empty (0)
		adjCur = adjNxt;
		adjNxt = adjNxt->next;

		// process the portal
		double t;
		char side = _split(node->line, &adjCur->portal->seg, &t);
		switch (side) {
		case SIDE_COL:
			DBG_ASSERT(0, -1, "Incorrect adjacents info");
			// this should not be possible, since the subspace of a node is a convex shape
			break;
		case SIDE_L_PARAL:
		case SIDE_L_FL:
		case SIDE_L_FR:
			// found left
			if (prside == 0) {
				// was right
				firstL = adjCur;
			}
			prside = 1;
			break;
		case SIDE_R_PARAL:
		case SIDE_R_FL:
		case SIDE_R_FR:
			// found right
			if (prside == 1) {
				// was left
				firstR = adjCur;
			}
			prside = 0;
			break;
		case SIDE_S_FL:
		case SIDE_S_FR:
			// found split
			adjNew = (PVS2D_PortalStack*)malloc(sizeof(PVS2D_PortalStack));
			DBG_ASSERT(adjNew, -1, "Failed to create new portal stack node");
			*adjNew = *adjCur;
			adjNew->portal = (PVS2D_Portal*)malloc(sizeof(PVS2D_Portal));
			DBG_ASSERT(adjNew->portal, -1, "Failed to create new portal");
			*adjNew->portal = *adjCur->portal;
			// depending on the side the portal we at, put new one at the "end" of the segment
			// or not, since we can only put new one after ourselves
			if (adjCur->left) {
				adjNew->portal->seg.tStart = t;
				adjCur->portal->seg.tEnd = t;
			}
			else {
				adjNew->portal->seg.tEnd = t;
				adjCur->portal->seg.tStart = t;
			}
			adjNew->next = adjCur->next;
			adjCur->next = adjNew;

			if (prside != -1) {
				if (prside == 1)	// was left
					firstR = adjNew;
				else				// was right
					firstL = adjNew;
				prside = 1 - prside;
			}
		}

		// loop two times just for sure
		if (adjNxt == adjacent) {
			if (loop == 1) break;
			loop++;
		}
	}
	// edge_case:
	//		0 for normal
	//		1 for no adjacents at all
	//		2 for no left adjacents
	//		3 for no right adjacents
	int edge_case = 0;
	if (!firstL && !firstR) {
		edge_case = prside + 2;
		switch (edge_case)
		{
		case 2:
			firstR = adjacent;
			break;
		case 3:
			firstL = adjacent;
			break;
		default:
			break;
		}
	}
	else {
		DBG_ASSERT(firstL && firstR, -1, "Failed to split left and right");
	}


	// step 2 - now that we splitted adjacents (sorta), we need to add portals from our node
	PVS2D_PortalStack* portals = _portalsOfNode(node);
	DBG_ASSERT(portals, -1, "Failed to create node's portals");
	// the portals have the reverse order of node->line, so for the left subspace they will be counterclockwise

	// step 3 - form adjacents for left and right subspace and recurse

	// create adjacents of right subspace 
	PVS2D_PortalStack* plast = 0;
	for (PVS2D_PortalStack* prt = portals; ; prt = prt->next) {
		prt->left = 0;
		if (!prt->next) {
			plast = prt;
			break;
		}
	}
	switch (edge_case) {
	case 0: 
		// everything is normal
		plast->next = firstR;
		for (PVS2D_PortalStack* prt = firstR; ; prt = prt->next) {
			if (prt->next == firstL) {
				prt->next = portals;
				break;
			}
		}
		break;
	
	case 1:
	case 3: 
		// no right adjacents (or no adjacents at all)
		// in each case we don't care about anything other than node's portals
		plast->next = portals;
		break;
	
	case 2: 
		// no left adjacents
		plast->next = firstR;
		for (PVS2D_PortalStack* prt = firstR; ; prt = prt->next) {
			if (prt->next == firstR) {
				prt->next = portals;
				break;
			}
		}
		break;
	}
	// now just feed 'portals' to the recursion
	if (node->right) {
		_buildPortals(node->right, portals);
		// we should get kinda the same thing
	}
	else {
		// right is a leaf, thats good
		for (PVS2D_PortalStack* prt = portals; ; prt = prt->next) {
			if (prt->left) {
				prt->portal->leftLeaf = node->rightLeaf;
			}
			else {
				prt->portal->rightLeaf = node->rightLeaf;
			}
			if (prt->next == portals)
				break;
		}
	}

	// now we need to cleanup 'portals' in order to pass it to the left child
	// the thing is that we will have to reverse the whole stack.
	// luckily nodes above don't rely on portals of ours, so we can change them as we like
	PVS2D_PortalStack* tportals = 0;
	adjCur = 0;
	adjNxt = portals;
	while (1) {
		adjCur = adjNxt;
		adjNxt = adjNxt->next;
		if (adjCur->portal->seg.line != node->line) {
			// since the order of portals is preserved throughout calls
			// of _buildPortals, our own portals will always be at the beginning
			// of the stack. so when we find one that isn't we can just exit the loop
			break;
		}
		adjCur->next = tportals;
		tportals = adjCur;
		if (adjNxt == portals) {
			// we looped
			break;
		}
	}

	// next step of cleanup is to connect right to the left
	// in edge cases 1 and 3 we don't have right adjacents at all
	// in edge case 2 we just need to connect right to themselves
	switch (edge_case) {
	case 2:
		for (PVS2D_PortalStack* prt = firstR; ; prt = prt->next) {
			if (prt->next == portals) {
				prt->next = firstR;
				break;
			}
		}
		break;
	case 0:
		for (PVS2D_PortalStack* prt = firstR; ; prt = prt->next) {
			if (prt->next == portals) {
				prt->next = firstL;
				break;
			}
		}
		break;
	}

	// now go to left
	plast = 0;
	for (PVS2D_PortalStack* prt = tportals; ; prt = prt->next) {
		prt->left = 1;
		if (!prt->next) {
			plast = prt;
			break;
		}
	}

	switch (edge_case) {
	case 0:
		// everything is normal
		plast->next = firstL;
		for (PVS2D_PortalStack* prt = firstL; ; prt = prt->next) {
			if (prt->next == firstR) {
				prt->next = tportals;
				break;
			}
		}
		break;
	case 1:
	case 2:
		// no left portals (or no portals at all)
		plast->next = tportals;
		break;
	case 3:
		// no right portals
		plast->next = firstL;
		for (PVS2D_PortalStack* prt = firstL; ; prt = prt->next) {
			if (prt->next == firstL) {
				prt->next = tportals;
				break;
			}
		}
		break;
	}
	// now we have tportals ready to be fed into left child

	if (node->left) {
		_buildPortals(node->left, tportals);
	}
	else {
		for (PVS2D_PortalStack* prt = tportals; ; prt = prt->next) {
			if (prt->left) {
				prt->portal->leftLeaf = node->leftLeaf;
			}
			else {
				prt->portal->rightLeaf = node->leftLeaf;
			}
			if (prt->next == tportals)
				break;
		}
	}

	// step 4 - now have to cleanup:

	// put portals into node
	adjCur = 0;
	adjNxt = tportals;
	while (1) {
		adjCur = adjNxt;
		adjNxt = adjNxt->next;
		if (adjCur->portal->seg.line != node->line) {
			// since the order of portals is preserved throughout calls
			// of _buildPortals, our own portals will always be at the beginning
			// of the stack. so when we find one that isn't we can just exit the loop
			break;
		}
		adjCur->next = node->portals;
		node->portals = adjCur;
		if (adjNxt == tportals) {
			// we looped
			break;
		}
	}

	// next step of cleanup is to connect left to the right.
	// in edge cases 1 and 2 we don't have left adjacents at all
	// in edge case 3 we just need to connect left to themselves
	switch (edge_case) {
	case 3:
		for (PVS2D_PortalStack* prt = firstL; ; prt = prt->next) {
			if (prt->next == tportals) {
				prt->next = firstL;
				break;
			}
		}
		break;
	case 0:
		for (PVS2D_PortalStack* prt = firstL; ; prt = prt->next) {
			if (prt->next == tportals) {
				prt->next = firstR;
				break;
			}
		}
		break;
	}
	//// cleanup tportals stack
	//for (PVS2D_PortalStack* prt = tportals; prt;) {
	//	PVS2D_PortalStack* nxt = prt->next;
	//	free(prt);
	//	prt = nxt;
	//}
	return 0;
}

int PVS2D_BuildPortals(PVS2D_BSPTreeNode* root) {
	return _buildPortals(root, 0);
}

unsigned int _findLeafCount(PVS2D_BSPTreeNode* node) {
	unsigned int ret = 0;
	if (node->left) {
		ret = _findLeafCount(node->left);
	}
	else {
		ret = node->leftLeaf;
	}
	if (node->right) {
		ret = max(ret, _findLeafCount(node->right));
	}
	else {
		ret = max(ret, node->rightLeaf);
	}
	return ret;
}

void _buildLeafGraphFromPortals(PVS2D_BSPTreeNode* node, PVS2D_LeafGraphNode* nodes) {
	for (PVS2D_PortalStack* prt = node->portals; prt; prt = prt->next) {
		if (prt->portal->seg.opq)
			continue;		// we are interested only in transparent portals connecting leaves
		unsigned int lleaf = prt->portal->leftLeaf;
		unsigned int rleaf = prt->portal->rightLeaf;
		if (isinf(prt->portal->seg.tStart) || isinf(prt->portal->seg.tEnd)) {
			// nodes that contain infinite portals must be oob ones
			nodes[lleaf].oob = 1;
			nodes[rleaf].oob = 1;
		}
		PVS2D_LGEdgeStack* lElem = (PVS2D_LGEdgeStack*)malloc(sizeof(PVS2D_LGEdgeStack));
		DBG_ASSERT(lElem, , "Failed to create leaf edge stack node");
		lElem->prt = prt->portal;
		lElem->node = nodes + rleaf;
		lElem->next = nodes[lleaf].adjs;
		nodes[lleaf].adjs = lElem;
		PVS2D_LGEdgeStack* rElem = (PVS2D_LGEdgeStack*)malloc(sizeof(PVS2D_LGEdgeStack));
		DBG_ASSERT(rElem, , "Failed to create leaf edge stack node");
		rElem->prt = prt->portal;
		rElem->node = nodes + lleaf;
		rElem->next = nodes[rleaf].adjs;
		nodes[rleaf].adjs = rElem;
	}
	if (node->left) {
		_buildLeafGraphFromPortals(node->left, nodes);
	}
	if (node->right) {
		_buildLeafGraphFromPortals(node->right, nodes);
	}
}

void _dfsTag(PVS2D_LeafGraphNode* node, char* tagged) {
	if (tagged[node->leaf])
		return;
	tagged[node->leaf] = 1;
	node->oob = 1;
	for (PVS2D_LGEdgeStack* edge = node->adjs; edge; edge = edge->next) {
		if (!edge->prt->seg.opq)
			_dfsTag(edge->node, tagged);
	}
}

PVS2D_LeafGraphNode* PVS2D_BuildLeafGraph(PVS2D_BSPTreeNode* root, unsigned int* nodesCDest) {
	DBG_ASSERT(root, 0, "'root' can't be nullptr");
	DBG_ASSERT(nodesCDest, 0, "'nodesCDest' can't be nullptr");
	int leafC = _findLeafCount(root) + 1;
	// be aware of the fact that leaves count from 0
	DBG_ASSERT(leafC, 0, "Incorrect BSP Tree data");
	PVS2D_LeafGraphNode* nodes = (PVS2D_LeafGraphNode*)calloc(leafC, sizeof(PVS2D_LeafGraphNode));
	DBG_ASSERT(nodes, 0, "Failed to create leaf graph nodes array");
	*nodesCDest = leafC;
	for (int i = 0; i < leafC; i++) {
		nodes[i].leaf = i;
		nodes[i].adjs = 0;
		nodes[i].oob = 0;
	}
	_buildLeafGraphFromPortals(root, nodes);

	// now we must spread oob tag
	char* tagged = (char*)calloc(leafC, sizeof(char));
	// kind of array that keeps track of what nodes we visited
	for (int i = 0; i < leafC; i++) {
		if (nodes[i].oob) {
			_dfsTag(nodes + i, tagged);
		}
	}
	free(tagged);
	return nodes;
}

// a1x, a1y, a2x, a2y represent a lineA, 
// b1x, b1y, b2x, b2y represent a lineB
// area "within a frustum" is area that lineA will traverse through
// when is rotated clockwise around the point of intersection of two lines
// before becoming lineB
typedef struct _frustum {
	double a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y;
} _frustum;

void _makeFrustumBetweenSegs(PVS2D_Seg* seg1, PVS2D_Seg* seg2, _frustum* frustum) {
	DBG_ASSERT(!isinf(seg1->tStart), , "Input segments can't be infinite");
	DBG_ASSERT(!isinf(seg2->tStart), , "Input segments can't be infinite");
	DBG_ASSERT(!isinf(seg1->tEnd), , "Input segments can't be infinite");
	DBG_ASSERT(!isinf(seg2->tEnd), , "Input segments can't be infinite");
	frustum->a1x = seg1->line->ax + seg1->tStart * (seg1->line->bx - seg1->line->ax);
	frustum->a1y = seg1->line->ay + seg1->tStart * (seg1->line->by - seg1->line->ay);
	frustum->a2x = seg2->line->ax + seg2->tStart * (seg2->line->bx - seg2->line->ax);
	frustum->a2y = seg2->line->ay + seg2->tStart * (seg2->line->by - seg2->line->ay);

	frustum->b1x = seg1->line->ax + seg1->tEnd * (seg1->line->bx - seg1->line->ax);
	frustum->b1y = seg1->line->ay + seg1->tEnd * (seg1->line->by - seg1->line->ay);
	frustum->b2x = seg2->line->ax + seg2->tEnd * (seg2->line->bx - seg2->line->ax);
	frustum->b2y = seg2->line->ay + seg2->tEnd * (seg2->line->by - seg2->line->ay);

	// the dot product here
	if (
		(frustum->b1x - frustum->a1x) * (frustum->b2x - frustum->a2x) +
		(frustum->b1y - frustum->a1y) * (frustum->b2y - frustum->a2y) > 0
	) {
		// the segments were oriented in the same way, have to change the direction,
		// so the lines cross inside the quadrangle
		double t;
		t = frustum->a2x;
		frustum->a2x = frustum->b2x;
		frustum->b2x = t;
		t = frustum->a2y;
		frustum->a2y = frustum->b2y;
		frustum->b2y = t;
	}
	// now it is guaranteed that lines cross eachother
	
	// but they can be wrongly oriented (the "inside" area might not contain the segments)
	// gotta fix that. the point b1 must be to the left of lineA
	
	// the cross product here
	if (
		(frustum->a2x - frustum->a1x) * (frustum->b1y - frustum->a1y) <
		(frustum->a2y - frustum->a1y) * (frustum->b1x - frustum->a1x)
	) {
		// the b1 is to the right of lineA, have to swap lineA and lineB
		double t;
		t = frustum->a1x;
		frustum->a1x = frustum->b1x;
		frustum->b1x = t;
		t = frustum->a2x;
		frustum->a2x = frustum->b2x;
		frustum->b2x = t;
		t = frustum->a1y;
		frustum->a1y = frustum->b1y;
		frustum->b1y = t;
		t = frustum->a2y;
		frustum->a2y = frustum->b2y;
		frustum->b2y = t;
	}
}

char _isInFrustum(_frustum* frustum, double x, double y) {
	// two cross products
	return
		((frustum->a2x - frustum->a1x) * (y - frustum->a1y) < (frustum->a2y - frustum->a1y) * (x - frustum->a1x)) &&
		((frustum->b2x - frustum->b1x) * (y - frustum->b1y) > (frustum->b2y - frustum->b1y) * (x - frustum->b1x));
}

// here you might end up with the fact that tStart > tEnd, which is kinda mathematically correct
// you might also end up with one or two of them being inf. that is also mathematically correct
// lastly, if line is ON the line of frustrum, you'll get nans
// god bless float-point arithmetics
void _cropLineByFrustum(PVS2D_Line* line, _frustum* frustum, double* tStartDest, double* tEndDest) {
	*tStartDest = -INFINITY;
	*tEndDest = INFINITY;
	
	double tAn, tAd, tBn, tBd;
	_intersectF(
		frustum->a1x, frustum->a1y, frustum->a2x, frustum->a2y, 
		line->ax, line->ay, line->bx, line->by, &tAn, &tAd
	);
	_intersectF(
		frustum->b1x, frustum->b1y, frustum->b2x, frustum->b2y,
		line->ax, line->ay, line->bx, line->by, &tBn, &tBd
	);
	// now we must determine which part of line is inside the frustum
	if (fabs(tAd) > MATCH_TOLERANCE) {
		if (tAd < 0) {
			// line goes to the right relatively to frustrum's lineA
			*tEndDest = min(*tEndDest, tAn / tAd);
		}
		else {
			*tStartDest = max(*tStartDest, tAn / tAd);
		}
	}
	if (fabs(tBd) > MATCH_TOLERANCE) {
		if (tBd < 0) {
			*tStartDest = max(*tStartDest, tBn / tBd);
		}
		else {
			*tEndDest = min(*tEndDest, tBn / tBd);
		}
	}

}

typedef struct _frustumStack {
	struct _frustum* frustum;
	struct _frustumStack* next;
} _frustumStack;

void _dfsPVSCalc(PVS2D_LeafGraphNode* node, PVS2D_Seg* prevSeg, _frustumStack* frs, char* visited, char* pvs) {
	pvs[node->leaf] = 1;
	for (PVS2D_LGEdgeStack* edge = node->adjs; edge; edge = edge->next) {
		if (!prevSeg) {
			// we are at root node
			// all neighboor nodes are visible from root
			visited[edge->node->leaf] = 1;
			_dfsPVSCalc(edge->node, &edge->prt->seg, frs, visited, pvs);
			visited[edge->node->leaf] = 0;
		}
		else {
			if (!visited[edge->node->leaf]) {
				// go through all of the frustums in frs and crop the segment accordingly
				double tStart = edge->prt->seg.tStart, tEnd = edge->prt->seg.tEnd;
				char ok = 1;
				for (_frustumStack* fr = frs; fr; fr = fr->next) {
					double ttStart = 0, ttEnd = 0;
					_cropLineByFrustum(edge->prt->seg.line, fr->frustum, &ttStart, &ttEnd);
					tStart = max(tStart, ttStart);
					tEnd = min(tEnd, ttEnd);
					if (tStart > tEnd + MATCH_TOLERANCE) {
						// it's not intersecting it anymore
						ok = 0;
						break;
					}
				}
				if (ok) {
					// we can go here

					// create new frustum and put it into stack
					// since we are using recursion, we can just 
					// allocate it on stack, and be fine with it
					_frustum newFrustum = { 0 };
					_makeFrustumBetweenSegs(prevSeg, &edge->prt->seg, &newFrustum);
					_frustumStack newNode = { 0 };
					newNode.frustum = &newFrustum;
					newNode.next = frs;
					visited[edge->node->leaf] = 1;
					_dfsPVSCalc(edge->node, &edge->prt->seg, &newNode, visited, pvs);
					visited[edge->node->leaf] = 0;
					// no need to delete anything since we allocated on stack :)
				}
			}
		}
	}
}

char* PVS2D_GetLeafPVS(PVS2D_LeafGraphNode* node, unsigned int leafC) {
	DBG_ASSERT(!node->oob, 0, "Can't build PVS of Out-Of-Bounds node");
	char* visited = (char*)calloc(leafC, sizeof(char));
	DBG_ASSERT(visited, 0, "Failed to create array of visited nodes");
	char* pvs = (char*)calloc(leafC, sizeof(char));
	DBG_ASSERT(pvs, 0, "Failed to create PVS array");
	_dfsPVSCalc(node, 0, 0, visited, pvs);
	free(visited);
	return pvs;
}
