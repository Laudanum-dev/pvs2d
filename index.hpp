////////////////////////////////////////////////////////////
/// \mainpage
///
/// \section welcome Добро пожаловать.
/// Добро пожаловать на официальную страницу документации PVS2D. 
/// Здесь вы найдете детальное описание всех структур и функций библиотеки.<br/>
/// <a href="pvs2d_8h.html">Основная страница документации</a>
///	
/// \section example Пример
/// Короткий пример использования программы
///
/// \code
/// #include <PVS2D.h>
/// 
/// // вертикальные стены в сцене
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
/// 	// создание BSP дерева
/// 	int segsC = sizeof(level) / sizeof(level[0]) / 5;
/// 	PVS2D_BSPTreeNode root;
/// 	PVS2D_BuildBSPTree(level, segsC, &root);
/// 
/// 	// построение порталов
/// 	PVS2D_BuildPortals(&root);
/// 
/// 	// построение графа листьев
/// 	unsigned int nodesC = 0;
/// 	PVS2D_LeafGraphNode* graph = PVS2D_BuildLeafGraph(&root, &nodesC);
/// 
/// 	// ...
/// 	// для каждой сущности в сцене - указать в какой листе она находится
/// 	// ...
///     
///     // предподсчет PVS во время компиляции сцены
/// 	for (unsigned int leafIdx = 0; leafIdx < nodesC; leafIdx++) {
/// 		// вычислить PVS листа
/// 		char* pvs = PVS2D_GetLeafPVS(graph + leafIdx, nodesC);
/// 
/// 		// ...
/// 		// сохранить PVS на диск
/// 		// ...
/// 
/// 		free(pvs);
/// 	}
/// 
/// 	// ... во время игры
/// 	unsigned int curLeaf = PVS2D_FindLeafOfPoint(&root, cameraX, cameraY);
/// 	// для каждого листа в PVS текущего листа - нарисовать сущности в листе
/// }
/// \endcode
////////////////////////////////////////////////////////////