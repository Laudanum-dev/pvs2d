////////////////////////////////////////////////////////////
/// \mainpage
///
/// \section welcome Welcome
/// Welcome to the official PVS2D documentation. Here you will find a detailed view of all
/// the PVS2D structures and functions. <br/>
///	
/// \section example Short example
/// Here is a short example, to show you how usable this library is:
///
/// \code
/// #include <PVS2D.h>
/// 
/// int level[] = {
/// 	0, 0, 0, 2, 1,
/// 	0, 2, 1, 2, 1,
/// 	1, 2, 1, 1, 1,
/// 	1, 1, 2, 1, 1,
/// 	2, 1, 2, 0, 1,
/// 	2, 0, 0, 0, 1
/// };
/// 
/// int main() {
/// 	// initialize the BSP Tree
/// 	int segsC = sizeof(level) / sizeof(level[0]) / 5;
/// 	PVS2D_BSPTreeNode root;
/// 	PVS2D_BuildBSPTree(level, segsC, &root);
/// 
/// 	// build the portal data
/// 	PVS2D_BuildPortals(&root);
/// 
/// 	// build the leaf graph
/// 	unsigned int nodesC = 0;
/// 	PVS2D_LeafGraphNode* graph = PVS2D_BuildLeafGraph(&root, &nodesC);
/// 
/// 	// ...
/// 	// for each entity mark it in the list of entities inside the leaf
/// 	// ...
/// 
/// 	for (unsigned int leafIdx = 0; leafIdx < nodesC; leafIdx++) {
/// 		// get the PVS of the leaf...
/// 		char* pvs = PVS2D_GetLeafPVS(graph + leafIdx, nodesC);
/// 
/// 		// ...
/// 		// save the pvs into the local storage
/// 		// ...
/// 
/// 		free(pvs);
/// 	}
/// 
/// 	// ... during main loop
/// 	unsigned int curLeaf = PVS2D_FindLeafOfPoint(&root, cameraX, cameraY);
/// 	// for each leaf in our PVS ... draw entities inside the leaf. 
/// }
/// \endcode
////////////////////////////////////////////////////////////