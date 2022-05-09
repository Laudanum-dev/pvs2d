/**
 * @file pvs2d.h
 * @author Latypov Ilya (id763281073@gmail.com)
 * @brief Potentially Visible Sets calculating library. 
 * @version 0.0.1
 * @date 2022-05-09
 * 
 * Potentially Visible Sets calculating library.
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef PVS2D_H
#define PVS2D_H


// --------------------------------------------------------
//                       STRUCTURES
// --------------------------------------------------------

/**
 * @brief 2D Line between two points. 
 * 
 * Represents a line that goes through two points. Points are required to be integer.
 * The architecture implies that there will be many segments lying on this line,
 * all of which are stored in `mems` stack.
 * Also note that line is "oriented" from Point A(ax, ay) to point B(bx, by).
 * The orientation doesn't have any impact on performance, but might 
 * slightly modify the resulting structures.
 * 
 */
typedef struct PVS2D_Line {
	/**
	 * @brief Points the line bases on. 
	 * 
	 * Four coordinates of two points the line bases on.
	 * 
	 */
	int ax, ay, bx, by;

	/**
	 * @brief Stack of all segments that are on this line. 
	 * 
	 * Represents a stack of all segments (instances of `PVS2D_Seg` structure) 
	 * that are based on this line.
	 * 
	 */
	struct PVS2D_SegStack* mems;
} PVS2D_Line;

/**
 * @brief 2D Segment lying on a specific line. 
 * 
 * Represents a segment that lies on a specific line. The endpoints of segment
 * are represented with tStart and tEnd variables, each of them represeting
 * a point on parametric line, tStart for the start of the segment and tEnd 
 * for the end. The point lying on the parametric line with parameter `t` can be 
 * represented as 
 * `(line->bx * t + line->ax * (1 - t), line->by * t + line->ay * (1 - t))`
 * Note that the segment is oriented the same way as the line 
 * it lies on.
 * 
 */
typedef struct PVS2D_Seg {
	/**
	 * @brief Base line. 
	 * 
	 * The line segment lies on. 
	 * 
	 */
	struct PVS2D_Line* line;

	/**
	 * @brief Start and End parameters of the segment. 
	 * 
	 * The start and end parameters of the endpoints of the segment. It must be guaranteed 
	 * that tStart < tEnd
	 * 
	 */
	double tStart, tEnd;

	/**
	 * @brief Opaqueness flag. 
	 * 
	 * The flag that tells whether the segment is opaque or not. This 
	 * is used during visibility calculations to allow for "hinting" 
	 * BSP Tree building algorithm where it should put a split. 
	 * In reality, it doesn't affect almost anything. 
	 * 0 for transparent value of flag, anything else for opaque. 
	 * 
	 */
	int opq;
} PVS2D_Seg;

/**
 * @brief Stack of segments. 
 * 
 * Represents a linked list of segments (instances of `PVS2D_Seg` structure). 
 * 
 */
typedef struct PVS2D_SegStack {
	/**
	 * @brief Segment itself. 
	 * 
	 * A segment itself. 
	 * 
	 */
	struct PVS2D_Seg* seg;

	/**
	 * @brief Pointer to next node. 
	 * 
	 * A pointer to the next node of linked list. 
	 * 
	 */
	struct PVS2D_SegStack* next;
} PVS2D_SegStack;

/**
 * @brief Binary Space Partioning tree node. 
 * 
 * Represents a Binary Space Partitioning tree node. It contains a line that splits the space, 
 * pointers to the left and right subtrees, and indexes of leaves that are used instead of nodes 
 * when either left or right subspace is empty. Since the splitting line also is bounded by 
 * the subspace of a node, it has its tSplitStart and tSplitEnd parameters (like if it was 
 * a segment). The structure also has a stack of all nodes that lie on the splitting line, 
 * as well as all of the portals.
 * 
 */
typedef struct PVS2D_BSPTreeNode {
	/**
	 * @brief Left and right subtree pointers. 
	 * 
	 * Pointers to the left and right subtrees. Their subspace is located directly 
	 * to the left or to the right of splitting line accordingly. Any of the pointers can be 
	 * `NULL`, indicating, that it's subspace doesn't have any segments in it, 
	 * therefore being a leaf. In this case leftLeaf or rightLeaf accordingly
	 * should contain valid index of a leaf. 
	 * 
	 */
	struct PVS2D_BSPTreeNode *left, *right;

	/**
	 * @brief Left and right leafs indexes. 
	 * 
	 * For example, if this node doesn't have a left child node, 
	 * the `leftLeaf` should contain the index of a leaf that is the left subspace.
	 * 
	 */
	unsigned int leftLeaf, rightLeaf;

	/**
	 * @brief Splitting line. 
	 * 
	 * The line that splits this node's subspace into two parts. Therefore left child's subspace 
	 * is directly to the left of the splitting line, and right child's is to the right. Their 
	 * subspaces are two parts of current node's subspace. 
	 * 
	 */
	struct PVS2D_Line* line;

	/**
	 * @brief Stack of segments on the splitting line. 
	 * 
	 * The stack of all segments that lie on the splitting line. 
	 * 
	 */
	struct PVS2D_SegStack* segs;
	
	/**
	 * @brief Endpoints of the splitting line. 
	 * 
	 * Parameters of the endpoints of the part of the splitting line, that lies inside the node's 
	 * subspace. As with segments (`PVS2D_Seg`), it must be that tSplitStart < tSplitEnd, 
	 * but unlike those, tSplitStart can be -infinity, and tSplitEnd can be +infinity.
	 * 
	 */
	double tSplitStart, tSplitEnd;

	/**
	 * @brief Stack of portals on the splitting line. 
	 * 
	 * Stack of portals lying on the splitting line. They must have no gaps between eachother 
	 * and never intersect with eachother. 
	 * 
	 */
	struct PVS2D_PortalStack* portals;
} PVS2D_BSPTreeNode;

/**
 * @brief Stack of portals. 
 * 
 * Represents a linked list of portals (instances of `PVS2D_Portal`). 
 * Also each linked list node provide information, whether the portal 
 * it contains is, in context, to the left or to the right. 
 * 
 */
typedef struct PVS2D_PortalStack {
	/**
	 * @brief Portal itself. 
	 * 
	 * A Portal itself. 
	 * 
	 */
	struct PVS2D_Portal* portal;

	/**
	 * @brief Pointer to the next node. 
	 * 
	 * A pointer to the next node of linked list. 
	 * 
	 */
	struct PVS2D_PortalStack* next;

	/**
	 * @brief Flag of relative position of the portal. 
	 * 
	 * A flag that is used during the portal building, used to indicate whether the current 
	 * subspace we are in is to the left of the portal (1 if so) or to the right (0 if so)
	 * 
	 */
	int left;
} PVS2D_PortalStack;

/**
 * @brief Portal between two leafs. 
 * 
 * Represents a portal lying between two leafs. Essentially, the portal is just a wrapper of 
 * `PVS2D_Seg` that contains indexes of left and right leafs. It is also essential that portal 
 * touches only two leafs (if its not so, the portal must be split during portal building step). 
 * 
 */
typedef struct PVS2D_Portal {
	/**
	 * @brief The segment the portal is. 
	 * 
	 * A segment that represents the portal's position. 
	 * 
	 */
	struct PVS2D_Seg seg;

	/**
	 * @brief Leafs of portals. 
	 * 
	 * Leafs that touch the portal from the left and from the right accordingly. 
	 * 
	 */
	unsigned int leftLeaf, rightLeaf;
} PVS2D_Portal;

/**
 * @brief Stack of leaf's adjacent leafs. 
 * 
 * Represents a linked list of the leaf's adjacent leafs. Those adjacency relations are 
 * represented as the graph, and this structure represents the edge of this graph, 
 * as if the graph was represented as an adjacency list. Simply put that this is like a 
 * list of all nodes adjacent to the `node`. Also note, that since all of connections 
 * between portals go through portals, each edge is represented by some portal. 
 * 
 */
typedef struct PVS2D_LeafGraphEdgeStack {
	/**
	 * @brief The node of the graph. 
	 * 
	 * The node of the graph, from where this edge goes. On the other side of the portal there is 
	 * the edge that goes back into the `node`. 
	 * 
	 */
	struct PVS2D_LeafGraphNode* node;

	/**
	 * @brief Pointer to the next edge. 
	 * 
	 * A pointer to the next node in the linked list. 
	 * 
	 */
	struct PVS2D_LeafGraphEdgeStack* next;

	/**
	 * @brief Portal that represents the edge. 
	 * 
	 * Since each edge between graph is going between two adjacent leafs, each edge can be represented 
	 * as the portal that connects those two leafs, which `prt` actually is. 
	 * 
	 */
	struct PVS2D_Portal* prt;
} PVS2D_LeafGraphEdgeStack, PVS2D_LGEdgeStack;

/**
 * @brief Node in graph of leaf adjacency. 
 * 
 * Represents a node in the graph of leaf adjacency. As well as represents the leaf itself by 
 * having some more related data, like whether this leaf is to be concidered Out-Of-Bounds or not. 
 * 
 */
typedef struct PVS2D_LeafGraphNode {
	/**
	 * @brief Leaf index. 
	 * 
	 * Index of the leaf this struct represents. 
	 * 
	 */
	unsigned int leaf;

	/**
	 * @brief Out-Of-Bounds flag. 
	 * 
	 * The flag that indicates whether this leaf is located Out-Of-Bounds 
	 * (shortly - OOB) or not. this flag is set automatically, based on the 
	 * leaf's adjacent portals. If the leaf contains the portal of infinite 
	 * length, then this portal is OOB. If the leaf has transparent portal 
	 * between itself and OOB portal, then this leaf is also OOB. The reason 
	 * for this flag to exist is that it is impossible (at least currently) 
	 * to calculate PVS with infinite portals present. 
	 * 
	 */
	char oob;

	/**
	 * @brief Stack of adjacent nodes. 
	 * 
	 * Stack of the graph edges that this node has. 
	 * 
	 */
	struct PVS2D_LeafGraphEdgeStack* adjs;
} PVS2D_LeafGraphNode;

/**
 * @brief Stack of leaf graph nodes. 
 * 
 * Represents a linked list of leaf graph nodes. 
 * 
 */
typedef struct PVS2D_LeafGraphNodeStack {
	/**
	 * @brief The node itself. 
	 * 
	 */
	PVS2D_LeafGraphNode* node;

	/**
	 * @brief Pointer to the next node. 
	 * 
	 * Pointer to the next linked list node. 
	 * 
	 */
	struct PVS2D_LeafGraphNodeStack* next;
} PVS2D_LeafGraphNodeStack, PVS2D_LGNodeStack;
 
// --------------------------------------------------------
//                  INTERFACE FUNCTIONS
// --------------------------------------------------------

/**
 * @brief Builds Binary Space Partitioning Tree from given array of segments. 
 * 
 * Builds Binary Space Partitioning Tree from given array of segments. the array must have 
 * the following structure: `segsC` blocks of 5 integers: `ax, ay, bx, by, opq`, 
 * `ax, ay` - coordinates of start point of segment, 
 * `bx, by` - coordinates of end point of segment, 
 * `opq` - flag indicating whether the segment is opaque (1 if so) or not (0 if so). 
 * Outputs 0 if succeed, something else otherwise. 
 * 
 * @param segs Input array of segments, `5 * segsC` ints. 
 * @param segsC The number of blocks. 
 * @param rootDest The destination of built BSP Tree. Must be allocated before calling. 
 * @return int - 0 if ok, something else otherwise. 
 */
int PVS2D_BuildBSPTree(
	int* segs, unsigned int segsC,
	PVS2D_BSPTreeNode* rootDest
);

/**
 * @brief Finds the leaf index the given point is inside of. 
 * 
 * @param root Pointer to BSP Tree root. 
 * @param x X coordinate of given point. 
 * @param y Y coordinate of given point. 
 * @return unsigned int - The index of the leaf the point is inside of. 
 */
unsigned int PVS2D_FindLeafOfPoint(
	PVS2D_BSPTreeNode* root,
	double x, double y
);

/**
 * @brief Finds the leafs the given segment goes through. 
 * 
 * Finds the leafs the given segment goes through. Outputs to "bitset" 
 * (array if chars, i-th char is 1 if segment passes through i-th leaf). 
 * 
 * @param root Pointer to BSP Tree root. 
 * @param ax X coordinate of first point of array. 
 * @param ay Y coordinate of first point of array. 
 * @param bx X coordinate of second point of array. 
 * @param by Y coordinate of second point of array. 
 * @param leafbitset The bitset the function will write into. Must be allocated 
 * with the number of leaves in tree beforehand. 
 */
void PVS2D_FindLeafsOfSegment(
	PVS2D_BSPTreeNode* root,
	double ax, double ay, double bx, double by,
	char* leafbitset
);

/**
 * @brief Builds portals data inside the Binary Space Partitioning tree. 
 * 
 * Builds portals data inside the Binary Space Partitioning tree. Essentially it fills in 
 * the node's `portals` field. Returns 0 on success, something else otherwise. 
 * 
 * @param root The pointer to BSP Tree root. 
 * @return int - 0 if succeed, something else otherwise. 
 */
int PVS2D_BuildPortals(
	PVS2D_BSPTreeNode* root
);

/**
 * @brief Builds leaf graph using tree's portal data. 
 * 
 * Builds leaf graph using tree's portal data. Therefore the tree must have been processed with 
 * `PVS2D_BuildPortals` beforehand. Outputs the pointer to the list of all graph nodes 
 * (the number of nodes is the number of leaves in the tree), as well 
 * as the number of leaves in the tree. 
 * 
 * @param root The pointer to BSP Tree root. 
 * @param nodesCDest The pointer to the unsigned int, where the number of leaves will be placed. 
 * @return PVS2D_LeafGraphNode* - The pointer to the list of graph nodes. 
 */
PVS2D_LeafGraphNode* PVS2D_BuildLeafGraph(
	PVS2D_BSPTreeNode* root,
	unsigned int* nodesCDest
);

/**
 * @brief Build the Potentially Visible Set of the given leaf. 
 * 
 * Technically, this function could have been improved by reusing calculations info 
 * from previous calculations. Returns the "bitmask" (array if chars, i-th char 
 * is 1 if i-th leaf is visible from given). 
 * 
 * @param node The node, PVS of which we need to calculate. 
 * @param leafC The number of leaves in the tree. 
 * @return char* - The bitmask. 
 */
char* PVS2D_GetLeafPVS(
	PVS2D_LeafGraphNode* node, unsigned int leafC
);

#endif