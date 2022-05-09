#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <raylib.h>
//#define RLGL_IMPLEMENTATION
#include <rlgl.h>
#include <pvs2d.h>

int segs[] = {
	-1, 0, -3, 0, 1,
	-3, 0, -3, -1, 1,
	-3, -1, -5, -1, 1,
	-5, -1, -5, 0, 1,
	-5, 0, -4, 0, 1,
	-4, 0, -4, 2, 1,
	-4, 2, -7, 2, 1,
	-7, 2, -7, 0, 1,
	-7, 0, -6, 0, 1,
	-6, 0, -6, -2, 1,
	-6, -2, -3, -2, 1,
	-3, -2, -3, -3, 1,
	-3, -3, -2, -3, 1,
	-2, -3, 0, -2, 1,
	0, -2, 0, -1, 1,
	0, -1, 3, -1, 1, 
	
	3, -1, 3, 0, 1,
	3, 0, 4, 1, 1,
	4, 1, 4, 2, 1,
	4, 2, 3, 3, 1, 
	3, 3, 3, 4, 1,

	3, 4, 0, 4, 1,
	0, 4, 0, 5, 1, 
	0, 5, -3, 5, 1,
	-3, 5, -3, 3, 1,
	-3, 3, -2, 3, 1,
	-2, 3, -2, 2, 1,
	-2, 2, -1, 2, 1,
	-1, 2, -1, 0, 1, 

	0, 1, 2, 1, 1,
	2, 1, 2, 2, 1,
	2, 2, 0, 2, 1,
	0, 2, 0, 1, 1,
	1, 1, 1, 2, 1,
	1, 1, 2, 2, 1
};

typedef struct _leafstack {
	struct PVS2D_LeafGraphNode* node;
	struct _leafstack* next;
} _leafstack;

typedef struct _treeDrawHelper {
	struct PVS2D_BSPTreeNode* node;
	unsigned int subtreeLeafC;
	unsigned int leafId;
	struct _treeDrawHelper* left;
	struct _treeDrawHelper* right;
	int x, y;
} _treeDrawHelper;

_treeDrawHelper* _initTreeDrawHelper(PVS2D_BSPTreeNode* node, int* cx, int cy) {
	_treeDrawHelper* tdh = malloc(sizeof(_treeDrawHelper));
	if (!tdh) return 0;
	tdh->node = node;
	tdh->x = 0;
	tdh->y = 0;
	tdh->subtreeLeafC = 0;
	int mcx = *cx;
	if (node->left) {
		tdh->left = _initTreeDrawHelper(node->left, cx, cy + 20);
	}
	else {
		tdh->left = malloc(sizeof(_treeDrawHelper));
		if (!tdh->left) return 0;
		*(tdh->left) = (_treeDrawHelper){ 0 };
		tdh->left->x = *cx;
		*cx += 10;
		tdh->left->y = cy + 20;
		tdh->left->leafId = node->leftLeaf;
		tdh->left->subtreeLeafC = 1;
	}

	if (node->right) {
		tdh->right = _initTreeDrawHelper(node->right, cx, cy + 20);
	}
	else {
		tdh->right = malloc(sizeof(_treeDrawHelper));
		if (!tdh->right) return 0;
		*(tdh->right) = (_treeDrawHelper){ 0 };
		tdh->right->x = *cx;
		*cx += 10;
		tdh->right->y = cy + 20;
		tdh->right->leafId = node->rightLeaf;
		tdh->right->subtreeLeafC = 1;
	}

	tdh->subtreeLeafC = tdh->left->subtreeLeafC + tdh->right->subtreeLeafC;
	tdh->x = (mcx + *cx) / 2 - 5;
	tdh->y = cy;
	return tdh;
}

typedef struct _bspNodeStack {
	struct PVS2D_BSPTreeNode* node;
	struct _bspNodeStack* next;
} _bspNodeStack;

unsigned int _getOpqPortalCount(PVS2D_BSPTreeNode* node) {
	unsigned int cnt = 0;
	for (PVS2D_PortalStack* prtst = node->portals; prtst; prtst = prtst->next) {
		if (prtst->portal->seg.opq)		// only count opaque portals
			cnt++;
	}
	if (node->left)
		cnt += _getOpqPortalCount(node->left);
	if (node->right)
		cnt += _getOpqPortalCount(node->right);
	return cnt;
}

Model _makeWall(double ax, double ay, double bx, double by) {
	Mesh wall = { 0 };
	wall.vertices = calloc(4, 3 * sizeof(float));
	wall.vertices = (float[]){ 
		ax, 0.0f, -ay, 
		ax, 1.0f, -ay,
		bx, 1.0f, -by,
		bx, 0.0f, -by
	};
	wall.texcoords = calloc(4, 2 * sizeof(float));
	wall.texcoords = (float[]){
		0.0f, 0.0f,
		0.0f, 1.0f,
		1.0f, 1.0f,
		1.0f, 0.0f
	};
	wall.indices = calloc(6, sizeof(unsigned short));
	wall.indices = (unsigned short[]){
		0, 1, 2,
		0, 2, 3
	};
	wall.vertexCount = 4;
	wall.triangleCount = 2;
	UploadMesh(&wall, false);
	return LoadModelFromMesh(wall);
}

void _createWalls(PVS2D_BSPTreeNode* node, PVS2D_LeafGraphNode* leafGraph, Model* walls, char** leafToMesh, unsigned int* curMeshIdx) {
	for (PVS2D_PortalStack* prtst = node->portals; prtst; prtst = prtst->next) {
		if (prtst->portal->seg.opq) {
			// create new mesh
			double ax = prtst->portal->seg.line->bx * prtst->portal->seg.tStart + prtst->portal->seg.line->ax * (1 - prtst->portal->seg.tStart);
			double ay = prtst->portal->seg.line->by * prtst->portal->seg.tStart + prtst->portal->seg.line->ay * (1 - prtst->portal->seg.tStart);
			double bx = prtst->portal->seg.line->bx * prtst->portal->seg.tEnd + prtst->portal->seg.line->ax * (1 - prtst->portal->seg.tEnd);
			double by = prtst->portal->seg.line->by * prtst->portal->seg.tEnd + prtst->portal->seg.line->ay * (1 - prtst->portal->seg.tEnd);
			walls[*curMeshIdx] = _makeWall(ax, ay, bx, by);

			// put into the correct leafs bitmasks that this mesh is there
			if (!leafGraph[prtst->portal->leftLeaf].oob) {
				leafToMesh[prtst->portal->leftLeaf][*curMeshIdx] = 1;
			}
			if (!leafGraph[prtst->portal->rightLeaf].oob) {
				leafToMesh[prtst->portal->rightLeaf][*curMeshIdx] = 1;
			}
			(*curMeshIdx)++;
		}
	}
	if (node->left)
		_createWalls(node->left, leafGraph, walls, leafToMesh, curMeshIdx);
	if (node->right)
		_createWalls(node->right, leafGraph, walls, leafToMesh, curMeshIdx);
}

void _drawWallsOnMap(PVS2D_BSPTreeNode* node) {
	double tS = max(-20.0, node->tSplitStart);
	double tE = min(20.0, node->tSplitEnd);
	double sax = node->line->bx * tS + node->line->ax * (1 - tS);
	double say = node->line->by * tS + node->line->ay * (1 - tS);
	double sbx = node->line->bx * tE + node->line->ax * (1 - tE);
	double sby = node->line->by * tE + node->line->ay * (1 - tE);
	DrawLine(
		floor((sax + 8) * 30), floor((say + 5) * 30),
		floor((sbx + 8) * 30), floor((sby + 5) * 30), (Color) { 100, 100, 100, 50 }
	);
	for (PVS2D_PortalStack* prtst = node->portals; prtst; prtst = prtst->next) {
		if (prtst->portal->seg.opq) {
			sax = prtst->portal->seg.line->bx * prtst->portal->seg.tStart + prtst->portal->seg.line->ax * (1 - prtst->portal->seg.tStart);
			say = prtst->portal->seg.line->by * prtst->portal->seg.tStart + prtst->portal->seg.line->ay * (1 - prtst->portal->seg.tStart);
			sbx = prtst->portal->seg.line->bx * prtst->portal->seg.tEnd + prtst->portal->seg.line->ax * (1 - prtst->portal->seg.tEnd);
			sby = prtst->portal->seg.line->by * prtst->portal->seg.tEnd + prtst->portal->seg.line->ay * (1 - prtst->portal->seg.tEnd);
			DrawLineEx(
				(Vector2) { floor((sax + 8) * 30), floor((say + 5) * 30) }, 
				(Vector2) { floor((sbx + 8) * 30), floor((sby + 5) * 30) }, 
				3, (Color) { 255, 255, 255, 255 });
		}
	}
	if (node->left)
		_drawWallsOnMap(node->left);
	if (node->right)
		_drawWallsOnMap(node->right);
}

char _drawGraph(_treeDrawHelper* node, Color* leafColor, unsigned int curLeaf) {
	char ret = 0;
	if (node->node) {
		// just a node
		DrawLine(node->x, node->y, node->left->x, node->left->y, WHITE);
		DrawLine(node->x, node->y, node->right->x, node->right->y, WHITE);
		DrawCircle(node->x, node->y, 5, WHITE);
		char lret = _drawGraph(node->left, leafColor, curLeaf);	// here we can safely recurse with no checks
		if (lret == 1) {
			DrawLineEx((Vector2) { node->x, node->y }, (Vector2) { node->left->x, node->left->y }, 5, (Color) { 255, 255, 255, 150 });
			ret = 1;
		}
		char rret = _drawGraph(node->right, leafColor, curLeaf);
		if (rret == 1) {
			DrawLineEx((Vector2) { node->x, node->y }, (Vector2) { node->right->x, node->right->y }, 5, (Color) { 255, 255, 255, 150 });
			ret = 1;
		}
	}
	else {
		DrawCircle(node->x, node->y, 5, leafColor[node->leafId]);
		DrawCircleLines(node->x, node->y, 5, WHITE);
		if (node->leafId == curLeaf)
			ret = 1;
	}
	return ret;
}

typedef struct _bunny {
	Model bunny;
	double x, y;
	float scale;
} _bunny;

int main() {
	// init the levle
	int segsC = sizeof(segs) / sizeof(segs[0]) / 5;
	PVS2D_BSPTreeNode root;
	PVS2D_BuildBSPTree(segs, segsC, &root);
	PVS2D_BuildPortals(&root);
	unsigned int nodesC = 0;
	PVS2D_LeafGraphNode* graph = PVS2D_BuildLeafGraph(&root, &nodesC);
	_leafstack** PVS = calloc(nodesC, sizeof(_leafstack*));
	if (!PVS) return -1;
	char** lpvs = calloc(nodesC, sizeof(char*));
	if (!lpvs) return -1;
	for (unsigned int i = 0; i < nodesC; i++) {
		if (!graph[i].oob) {
			char* pvs = PVS2D_GetLeafPVS(graph + i, nodesC);
			if (!pvs) return -1;
			for (unsigned int k = 0; k < nodesC; k++) {
				if (pvs[k]) {
					_leafstack* newNode = malloc(sizeof(_leafstack));
					if (!newNode) return -1;
					newNode->node = graph + k;
					newNode->next = PVS[i];
					PVS[i] = newNode;
				}
			}
			lpvs[i] = pvs;
		}
	}

	// create a list of meshes
	// iterate over tree, put each opaque portal as mesh into the leaf-mesh list into the 
	// each of portal's side leafs, if they are not oob
	unsigned int portalsC = _getOpqPortalCount(&root);
	Model* models = calloc(portalsC + 4, sizeof(Model));
	// + 3 bnuyys and 1 BIG CHUNGUS
	if (!models) return -1;
	unsigned char** leafToMesh = calloc(nodesC, sizeof(unsigned char*));
	if (!leafToMesh) return -1;
	for (unsigned int i = 0; i < nodesC; i++) {
		leafToMesh[i] = calloc(portalsC + 4, sizeof(unsigned char));
		if (!leafToMesh[i]) return -1;
	}
	unsigned int idx = 0;
	
	//_createMeshes(&root, graph, meshes, leafToMesh, &idx);

	const int scMW = 400, scMH = 400;			// map view size
	const int sc3W = 800, sc3H = 700;			// 3d view size
	const int scGW = scMW, scGH = sc3H - scMH;	// graph view size
	// create a list of leafs colors (random)
	Color* leafColor = calloc(nodesC, sizeof(Color));
	if (!leafColor) return -1;
	srand(3);
	for (unsigned int i = 0; i < nodesC; i++) {
		if (graph[i].oob)
			leafColor[i] = BLACK;
		else
			leafColor[i] = ColorFromHSV(rand() % 360, 0.9, 0.9);
		// some pastel color
	}
	// create a list of structs that indicate a node, 
	// its position, children
	int _idx = sc3W + 70;
	_treeDrawHelper* tdh = _initTreeDrawHelper(&root, &_idx, scMH + 50);

	InitWindow(scMW + sc3W, sc3H, "PVS2D Demo");

	// get walls texture
	Image wimg = LoadImage("block.png");
	Texture wtex = LoadTextureFromImage(wimg);
	UnloadImage(wimg);
	Image chimg = LoadImage("chungus.png");
	ImageFlipVertical(&chimg);
	Texture chtex = LoadTextureFromImage(chimg);
	UnloadImage(chimg);
	Model bunny = LoadModel("bnuyy.obj");
	_bunny bunnys[3] = {
		(_bunny){.bunny = bunny, .x = 0.5f, .y = -0.5f, .scale = 0.036f},
		(_bunny){.bunny = bunny, .x = 1.2f, .y = 0.3f, .scale = 0.04f},
		(_bunny){.bunny = bunny, .x = 2.3f, .y = 1.2f, .scale = 0.03f}
	};
	// "UploadMesh" function requires opengl to be initialized
	_createWalls(&root, graph, models, leafToMesh, &idx);
	for (unsigned int i = 0; i < portalsC; i++) {
		models[i].materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = wtex;
	}
	// put models into their leaves
	unsigned int lff = 0;
	for (int i = 0; i < 3; i++) {
		models[portalsC + i] = bunnys[i].bunny;						// bnuys!
		lff = PVS2D_FindLeafOfPoint(&root, bunnys[i].x, bunnys[i].y);
		leafToMesh[lff][portalsC + i] = 1;
	}
	models[portalsC + 3] = _makeWall(0.5, 1.5, 0.5, 3.5);	// big chungus.
	models[portalsC + 3].materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = chtex;
	char* chungusSpace = calloc(nodesC, sizeof(char));
	if (!chungusSpace) return -1;
	PVS2D_FindLeafsOfSegment(&root, 0.5, 1.5, 0.5, 3.5, chungusSpace);
	for (unsigned int i = 0; i < nodesC; i++) {
		if (chungusSpace[i]) {
			leafToMesh[i][portalsC + 3] = 1;
		}
	}

	RenderTexture2D rdTarget = LoadRenderTexture(sc3W, sc3H);
	RenderTexture2D mapTarget = LoadRenderTexture(scMW, scMH);
	unsigned char* pixelToLeaf = calloc(scMW * scMH, sizeof(unsigned char));
	if (!pixelToLeaf) return -1;

	Camera camera = { 0 };
	camera.position = (Vector3){ -5.5f, 0.5f, -1.0f };
	camera.target = (Vector3){ -5.5f, 0.5f, -2.0f };
	camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
	camera.fovy = 60.0f;
	camera.projection = CAMERA_PERSPECTIVE;
	SetCameraMode(camera, CAMERA_FIRST_PERSON);

	SetTargetFPS(60);
	char needRedrawMap = 1;
	char updatePVS = 1;
	unsigned int prevLeaf = -1;		// danger
	unsigned char* visibleModels = calloc(portalsC + 4, sizeof(unsigned char));
	if (!visibleModels) return -1;
	
	while (!WindowShouldClose()) {
		UpdateCamera(&camera);		// i didn't bother with making my own controls

		if (IsKeyPressed(KEY_R))
			updatePVS = 1 - updatePVS;

		double px = camera.position.x;
		double py = camera.position.y;
		double pz = -camera.position.z;	// take in account the fact that we flipped the map

		if (updatePVS) {
			// find the current leaf, and if its changed then update pvs
			unsigned int lf = PVS2D_FindLeafOfPoint(&root, px, pz);
			if (lf != prevLeaf) {
				// recalculate visible models
				prevLeaf = lf;
				if (graph[lf].oob) {
					// if we are in oob node, then obviously we don't have pvs calculated
					// so just draw everything
					for (unsigned int i = 0; i < portalsC + 4; i++) {
						visibleModels[i] = 1;
					}
				}
				else {
					for (unsigned int i = 0; i < portalsC + 4; i++) {
						visibleModels[i] = 0;
					}
					// iterate over all visible leafs form our
					for (_leafstack* curLeaf = PVS[lf]; curLeaf; curLeaf = curLeaf->next) {
						// kinda like bitwise OR
						for (unsigned int i = 0; i < portalsC + 4; i++) {
							visibleModels[i] |= leafToMesh[curLeaf->node->leaf][i];
						}
					}
				}
				
			}
		}

		if (needRedrawMap) {
			needRedrawMap = 0;
			BeginTextureMode(mapTarget);
				ClearBackground(BLACK);
				// for each pixel on screen, find its leaf and color accordingly
				// ik that this is ultra bad solution, but it works :^)
				// then iterate over the tree, draw the splitlines and opaque portals
				for (int x = 0; x < scMW; x++) {
					for (int y = 0; y < scMH; y++) {
						
						double rx = (x + 0.5) / 30.0 - 8;
						double ry = (y + 0.5) / 30.0 - 5;
						unsigned int leaf = PVS2D_FindLeafOfPoint(&root, rx, ry);
						if (graph[leaf].oob) {
							DrawPixel(x, y, BLACK);
						}
						else {
							DrawPixel(x, y, leafColor[leaf]);
						}
						if (x % 30 == 0 || y % 30 == 0) {
							DrawPixel(x, y, (Color) { 20, 20, 20, 20 });
						}
						pixelToLeaf[y * scMW + x] = leaf;
					}
				}
				_drawWallsOnMap(&root);
			EndTextureMode();

		}
		
		// draw dat 3d
		BeginTextureMode(rdTarget);
			//DrawLine(0, 0, 100, 100, RED);
			ClearBackground(WHITE);
			BeginMode3D(camera);
				rlDisableBackfaceCulling();
				for (unsigned int i = 0; i < portalsC; i++) {
					if (visibleModels[i]) {
						DrawModel(models[i], (Vector3) { 0, 0, 0 }, 1.0f, WHITE);
					}
				}
				// draw bunnys
				for (unsigned int i = portalsC; i < portalsC + 3; i++) {
					if (visibleModels[i]) {
						DrawModel(
							models[i], 
							(Vector3) { bunnys[i - portalsC].x, 0.0, -bunnys[i - portalsC].y }, 
							bunnys[i - portalsC].scale, 
							ColorFromHSV(rand() % 360, 0.9, 0.9));
					}
				}
				// draw chungus
				if (visibleModels[portalsC + 3]) {
					DrawModel(models[portalsC + 3], (Vector3) { 0, 0, 0 }, 1.0f, WHITE);
				}
				// dispatch render batch before reenabling culling
				rlDrawRenderBatchActive();
				rlEnableBackfaceCulling();
			EndMode3D();
		EndTextureMode();

		BeginDrawing();
			ClearBackground(WHITE);
			DrawTexture(mapTarget.texture, sc3W, 0, WHITE);

			// draw player position on map
			DrawLine(sc3W + (px + 8) * 30, 0, sc3W + (px + 8) * 30, scMH, RAYWHITE);
			DrawLine(sc3W, (-pz + 8.3) * 30, sc3W + scMW, (-pz + 8.3) * 30, RAYWHITE);

			// highlight the leaves that are in PVS
			if (!graph[prevLeaf].oob) {
				for (int x = 0; x < scMW; x++) {
					for (int y = 0; y < scMH; y++) {
						if (lpvs[prevLeaf][pixelToLeaf[y * scMW + x]]) {
							DrawPixel(x + sc3W, scMH - y - 1, (Color){255, 255, 255, 150});
						}
					}
				}
			}
			// - probably iterate over the texture's pixels and if it's color is smth
			// then light it up
			
			// then draw graph using the positions we calculated earlier, 
			// add some outline to the leaf we are in and path from root to it
			DrawRectangle(sc3W, scMH, scGW, scGH, BLACK);
			_drawGraph(tdh, leafColor, prevLeaf);
			
			// draw the 3d stuff we drawn previously
			DrawTexture(rdTarget.texture, 0, 0, WHITE);
			DrawTextureRec(rdTarget.texture, (Rectangle) { 0, 0, sc3W, -sc3H }, (Vector2) { 0, 0 }, WHITE);
			// render models...

			// and most importantly
			DrawFPS(0, 0);
			if (updatePVS)
				DrawText("PVS UPDATE: ON", 0, 25, 24, GREEN);
			else
				DrawText("PVS UPDATE: OFF", 0, 25, 24, RED);
		EndDrawing();
		
	}

	UnloadRenderTexture(rdTarget);
	CloseWindow();
	return 0;
}