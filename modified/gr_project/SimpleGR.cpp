/* SHAYAN TAHERI (A01956093) */

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <limits>
#include <cmath>
#include <cassert>
#include <algorithm>

using namespace std;

#include "SimpleGR.h"

bool CompareByBox::operator()(const IdType a,
								const IdType b) const {
	double aW = fabs(static_cast<double>(nets[a].one.x)
					- static_cast<double>(nets[a].two.x));
	double aH =	fabs(static_cast<double>(nets[a].one.y)
					- static_cast<double>(nets[a].two.y));
	pair<double, double> costA(aW + aH, min(aW, aH) / max(aW, aH));

	double bW =	fabs(static_cast<double>(nets[b].one.x)
					- static_cast<double>(nets[b].two.x));
	double bH =	fabs(static_cast<double>(nets[b].one.y)
					- static_cast<double>(nets[b].two.y));
	pair<double, double> costB(bW + bH, min(bW, bH) / max(bW, bH));

	return costA < costB;
}

CostType UnitCost::viaCost(void) const {
	return viaFactor * edgeBase;
}

CostType UnitCost::operator()(IdType edgeId, IdType netId) const {
	if (simplegr.edges[edgeId].type == VIA) {
		return viaCost();
	} else {
		return edgeBase;
	}
}

CostType DLMCost::viaCost(void) const {
	return viaFactor * edgeBase;  // 3x as costly as a "regular" segment
}

CostType DLMCost::operator()(IdType edgeId, IdType netId) const {
	if (simplegr.edges[edgeId].type == VIA) {
		return viaCost();
	} else {
		const CapType capacity = simplegr.edges[edgeId].capacity;
		const CapType newUsage = simplegr.edges[edgeId].usage
				+ max(simplegr.minWidths[simplegr.edges[edgeId].layer],
						simplegr.nets[netId].wireWidth)
				+ simplegr.minSpacings[simplegr.edges[edgeId].layer];

		if (newUsage > capacity) {
			return edgeBase	+ simplegr.edges[edgeId].historyCost *
					min(powMax, pow(powBase, static_cast<CostType>(newUsage - capacity) / static_cast<CostType>(capacity)));
		} else {
			return edgeBase
					+ simplegr.edges[edgeId].historyCost * static_cast<CostType>(newUsage) / static_cast<CostType>(capacity);
		}
	}
}

void SimpleGR::routeFlatNets(bool allowOverflow, const CostFunction &f) {
	unsigned flatNetsRouted = 0;
	vector<IdType> order;
	for (IdType i = 0; i < nets.size(); ++i) {
		order.push_back(i);
	}

	sort(order.begin(), order.end(), CompareByBox(nets));

	bool bboxConstrain = true;

	for (unsigned i = 0; i < order.size(); ++i) {
		if (i % 100 == 0)
			cout << i << "\r" << flush;

		const Net &net = nets[order[i]];

		if (net.routed)
			continue;

		if (net.one.x == net.two.x || net.one.y == net.two.y) {
			routeNet(order[i], allowOverflow, bboxConstrain, f);
			if (net.routed)
				++flatNetsRouted;
		}
	}

	cout << flatNetsRouted << " flat nets routed" << endl;
}

CostType SimpleGR::routeNet(const IdType netId, bool allowOverflow,
							bool bboxConstrain, const CostFunction &f) {
	vector<IdType> path;

	Net &net = nets[netId];

	CostType totalCost;

	if (bboxConstrain) {
		const Point botleft(min(net.one.x, net.two.x),
							min(net.one.y, net.two.y), 0);
		const Point topright(max(net.one.x, net.two.x),
							max(net.one.y, net.two.y), 0);

		const bool doNotAllowOverflow = false;
		totalCost = routeMaze(netId,
		                      doNotAllowOverflow,
		                      botleft, topright,
		                      f, path);

		if (path.size() == 0) {
			// the initial attempt within the bounding box failed, try again
			totalCost = routeMaze(netId,
			                      allowOverflow,
			                      Point(0, 0, 0),
			                      Point(xTiles, yTiles, 0),
			                      f, path);
		}
	} else {
		totalCost = routeMaze(netId,
		                      allowOverflow,
		                      Point(0, 0, 0),
		                      Point(xTiles, yTiles, 0),
		                      f, path);
	}

	for (unsigned i = 0; i < path.size(); ++i) {
		addSegment(netId, path[i]);
	}

	net.routed = (path.size() > 0);

	return totalCost;
}

void SimpleGR::ripUpNet(const IdType netId) {
	Net &net = nets[netId];
	for (unsigned i = 0; i < net.segments.size(); ++i) {
		ripUpSegment(netId, net.segments[i]);
	}
	net.segments.clear();
	net.routed = false;
}

void SimpleGR::routeNets(bool allowOverflow, const CostFunction &f) {
	vector<IdType> order;
	for (unsigned i = 0; i < nets.size(); ++i) {
			order.push_back(i);
	}

	sort(order.begin(), order.end(), CompareByBox(nets));

	bool bboxConstrain = true;

	for (unsigned i = 0; i < order.size(); ++i) {
		if (i % 100 == 0)
			cout << i << "\r" << flush;

		if (nets[order[i]].routed)
			continue;

		routeNet(order[i], allowOverflow, bboxConstrain, f);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Implement an A* search based maze routing algorithm
// and a corresponding back-trace procedure
// The function needs to correctly deal with the following conditions:
// 1. Only search within a bounding box defined by botleft and topright points
// 2. Control if any overflow on the path is allowed or not
///////////////////////////////////////////////////////////////////////////////
CostType SimpleGR::routeMaze(const IdType netId, bool allowOverflow,
						const Point &botleft, const Point &topright,
						const CostFunction &f, vector<IdType> &path) {
	// record the wire width of the net
	const CapType netMinWidth = nets[netId].wireWidth;

	// find out the ID of the source and sink tiles
	Net &net = nets[netId];
	const IdType startTile = xyzToTileId(net.one.x, net.one.y, net.one.z);
	const IdType targetTile = xyzToTileId(net.two.x, net.two.y, net.two.z);

	// insert the source tile into the priority queue
	//priorityQueue.updateTile(startTile, 0., 0., numeric_limits<IdType>::max());

	// Instantiate the Cost function to calculate the Manhattan distance between
	// two tiles.
	const LBFunction &lb =
			(nets[netId].segments.size() == 0 ?
					static_cast<LBFunction&>(em) : static_cast<LBFunction&>(lbm));


	/* Start Point of the Code */

	/* Defining the parameters and variables */

	IdType current_tile;
	IdType iteration = 0;
	IdType N_openset = 0;
	vector<IdType> closedset;
	CoordType xx, yy, zz;
	vector<IdType> neighbors_id;
	vector<Point> neighbors_point;
	IdType cont_c = 0;
	IdType cont_o = 0;
	CostType tentative_actualCost;
	CostType tentative_totalCost;
	CostType temp_f;
	IdType target_index;
	IdType edges_numb2 = 0;
	IdType edges_numb3 = 0;
	IdType neighbors_numb = 0;
	IdType temp_id;
	IdType con_a = 0;
	IdType con_c = 0;
	IdType coni1 = 0;
	IdType coni2 = 0;

	/* "allowOverflow" = True --> It means that there is no constraint on any edge. So, the usage of an edge can be greater than its capacity.
	 * But, the routing of a net is not allowed outside of the bounding box. */

	/* "closetset": This priority queue has been defined for keeping IDs of the Tiles that has been used for routing of each net.
	 * I have used this queue for making the "path".*/

	/* "priorityQueue = openset": This priority queue has been used with all of its elements (totalCost, actualCost, heapLoc) and its functions for
	 * Implementing the A star algorithm. The current tile and its adjacent elements and their information have been included in this priority queue.
	 */

	/* Defining "startpoint" and "targetpoint" of the net. */

	const Point startpoint (net.one.x, net.one.y, net.one.z);
	const Point targetpoint (net.two.x, net.two.y, net.two.z);

	/* Initializing both "priorityQueue" and "closedset" with "startTile". */

	temp_f = lb.operator ()(startpoint,targetpoint);

	priorityQueue.updateTile(startTile, temp_f, 0., numeric_limits<IdType>::max());
	closedset.push_back(startTile);

	/* "actualCost" = The "g score" in A star algorithm. */
	/* "totalCost" = The "f score" in A star algorithm. */
	/* "heapLoc" = The "Tile ID". */
	/* "parent" = For storing the parents of the future "Tiles" */
	/* "current_tile" = The tile that has the lowest f score. */

	/* While the openset is not empty (according to A* algorithm) do another iteration */

	while (priorityQueue.empty() != 1) {

	/* Get the tile that has the lowest "f score" and put its ID in "current_tile" */

	current_tile = priorityQueue.getBestTile();

	/* Put the "current_tile" in closedset that means this tile has been used. */

	if (iteration >= 1) {
	closedset.push_back(current_tile);}

	/* If "current_tile" = "target_tile", exit the routine because the "Target" has been reached. */

	if (current_tile == targetTile) {
			break;}

	/* Eliminate the lowest f score tile (current_tile) from the openset. */

	priorityQueue.removeBestTile();

	/* Calculate the X, Y, and Z dimensions of "current_tile". */

	xx = current_tile % xTiles;
	yy = (current_tile / xTiles) % yTiles;
	zz = current_tile / (xTiles * yTiles);

	neighbors_numb = 0;

	/* Now, using the calculated X, Y, and Z dimension for finding adjacent neighbors of "current_tile". */

	edges_numb3 = edges.size();

	/* "coni1" and "coni2" : They are for determining the effect of bounding box on neighbors of the current_tile. It means that they will determine
	 * the situation of having a neighbor according to the bounding box.
	 * For example: If "coni1" == 1 --> There is an available neighbor for current_tile in the bounding box.
	 * Notice: These two parameter won't give any information about the existence of neighbors in closedset or blocklist.
	 */

	/* If "zz == 1": The "current_tile" is in layer one. Assuming only Horizontally movement in this layer. */

	/* If Right neighbor doesn't exceed bounding box and the overflow is allowable, consider it.
	 * But if the overflow is not allowable, then if the edge between two tile has an usage lower than its capacity, consider it. */

	if ((zz == 1) && (zz < numLayers)) {

	if ((xx + tileWidth) <= max(net.one.x, net.two.x) && (xx < yTiles)) {

	coni1 = 0;

	if (allowOverflow) {

	neighbors_id.push_back(xyzToTileId ((xx + tileWidth), yy, zz));
	neighbors_point.push_back(Point ((xx + tileWidth), yy, zz));
	neighbors_numb = neighbors_numb + 1;
	coni1 = 1;}

	if (!allowOverflow) {
	temp_id = xyzToTileId ((xx + tileWidth), yy, zz);
	for (IdType nn = 0; nn <= edges_numb3 - 1; nn++) {
	if (((edges[nn].tile1 == temp_id) && (edges[nn].tile2 == current_tile)) || ((edges[nn].tile2 == temp_id) && (edges[nn].tile1 == current_tile))) {
	if (edges[nn].usage < edges[nn].capacity) {

	neighbors_id.push_back(xyzToTileId ((xx + tileWidth), yy, zz));
	neighbors_point.push_back(Point ((xx + tileWidth), yy, zz));
	neighbors_numb = neighbors_numb + 1;
	coni1 = 1;}}}}}

	/* If Left neighbor doesn't exceed bounding box and the overflow is allowable, consider it.
	* But if the overflow is not allowable, then if the edge between two tile has an usage lower than its capacity, consider it. */

	if ((xx - tileWidth) >= min(net.one.x, net.two.x) && (xx < xTiles)) {

	coni2 = 0;

	if (allowOverflow) {

	neighbors_id.push_back(xyzToTileId ((xx - tileWidth), yy, zz));
	neighbors_point.push_back(Point ((xx - tileWidth), yy, zz));
	neighbors_numb = neighbors_numb + 1;
	coni2 = 1;}

	if (!allowOverflow) {
	temp_id = xyzToTileId ((xx - tileWidth), yy, zz);
	for (IdType nn = 0; nn <= edges_numb3 - 1; nn++) {
	if (((edges[nn].tile1 == temp_id) && (edges[nn].tile2 == current_tile)) || ((edges[nn].tile2 == temp_id) && (edges[nn].tile1 == current_tile))) {
	if (edges[nn].usage < edges[nn].capacity) {

	neighbors_id.push_back(xyzToTileId ((xx - tileWidth), yy, zz));
	neighbors_point.push_back(Point ((xx - tileWidth), yy, zz));
	neighbors_numb = neighbors_numb + 1;
	coni2 = 1;}}}}}}

	/* If "zz == 0": The "current_tile" is in layer zero. Assuming only Vertically movement in this layer. */

	/* If Up neighbor doesn't exceed bounding box and the overflow is allowable, consider it.
	* But if the overflow is not allowable, then if the edge between two tile has an usage lower than its capacity, consider it. */

	if ((zz == 0) && (zz < numLayers)) {

	if ((yy + tileHeight) <= max(net.one.y, net.two.y) && (yy < yTiles)) {

	coni1 = 0;

	if (allowOverflow) {

	neighbors_id.push_back(xyzToTileId (xx, (yy + tileHeight), zz));
	neighbors_point.push_back(Point (xx, (yy + tileHeight), zz));
	neighbors_numb = neighbors_numb + 1;
	coni1 = 1;}

	if (!allowOverflow) {
	temp_id = xyzToTileId (xx, (yy + tileHeight), zz);
	for (IdType nn = 0; nn <= edges_numb3 - 1; nn++) {
	if (((edges[nn].tile1 == temp_id) && (edges[nn].tile2 == current_tile)) || ((edges[nn].tile2 == temp_id) && (edges[nn].tile1 == current_tile))) {
	if (edges[nn].usage < edges[nn].capacity) {

	neighbors_id.push_back(xyzToTileId (xx, (yy + tileHeight), zz));
	neighbors_point.push_back(Point (xx, (yy + tileHeight), zz));
	neighbors_numb = neighbors_numb + 1;
	coni1 = 1;}}}}}

	/* If Down neighbor doesn't exceed bounding box and the overflow is allowable, consider it.
	* But if the overflow is not allowable, then if the edge between two tile has an usage lower than its capacity, consider it. */

	if ((yy - tileHeight) >= min(net.one.y, net.two.y) && (yy < yTiles)) {

	coni2 = 0;

	if (allowOverflow) {

	neighbors_id.push_back(xyzToTileId (xx, (yy - tileHeight), zz));
	neighbors_point.push_back(Point (xx, (yy - tileHeight), zz));
	neighbors_numb = neighbors_numb + 1;
	coni2 = 1;}

	if (!allowOverflow) {
	temp_id = xyzToTileId (xx, (yy - tileHeight), zz);
	for (IdType nn = 0; nn <= edges_numb3 - 1; nn++) {
	if (((edges[nn].tile1 == temp_id) && (edges[nn].tile2 == current_tile)) || ((edges[nn].tile2 == temp_id) && (edges[nn].tile1 == current_tile))) {
	if (edges[nn].usage < edges[nn].capacity) {

	neighbors_id.push_back(xyzToTileId (xx, (yy - tileHeight), zz));
	neighbors_point.push_back(Point (xx, (yy - tileHeight), zz));
	neighbors_numb = neighbors_numb + 1;
	coni2 = 1;}}}}}}

	/* If "current_tile" is in Down layer, then it can have adjacent neighbor in Top layer. If there isn't any available neighbor in
	 * Up or Down of the current_tile (because of bounding box, closedset, or blocklist) for flat routing, then consider its neighbor in Top layer. */

	if ((zz == 0) && (zz < numLayers)) {

		con_a = 0;
		for (IdType w1 = 0; w1 <= iteration; w1++) {
		if (closedset[w1] == xyzToTileId (xx, (yy + tileHeight), zz)) {
		con_a = 1;}}

		con_c = 0;
		for (IdType w2 = 0; w2 <= iteration; w2++) {
		if (closedset[w2] == xyzToTileId (xx, (yy - tileHeight), zz)) {
		con_c = 1;}}

	if (((coni1 == 0) || (con_a == 1)) && ((coni2 == 0) || (con_c == 1))) {

	neighbors_id.push_back(xyzToTileId (xx, yy, zz + 1));
	neighbors_point.push_back(Point (xx, yy, zz + 1));
	neighbors_numb = neighbors_numb + 1;}}

	/* If "current_tile" is in Top layer, then it can have adjacent neighbor in Down layer. If there isn't any available neighbor in
	* Right or Left of the current_tile (because of bounding box, closedset, or blocklist) for flat routing, then consider its neighbor in Down layer. */

	if ((zz == 1) && (zz < numLayers)) {

		con_a = 0;
		for (IdType w1 = 0; w1 <= iteration; w1++) {
		if (closedset[w1] == xyzToTileId ((xx + tileWidth), yy, zz)) {
		con_a = 1;}}

		con_c = 0;
		for (IdType w2 = 0; w2 <= iteration; w2++) {
		if (closedset[w2] == xyzToTileId ((xx - tileWidth), yy, zz)) {
		con_c = 1;}}

	if (((coni1 == 0) || (con_a == 1)) && ((coni2 == 0) || (con_c == 1))) {

	neighbors_id.push_back(xyzToTileId (xx, yy, zz - 1));
	neighbors_point.push_back(Point (xx, yy, zz - 1));
	neighbors_numb = neighbors_numb + 1;}}

	/* Calculate actualCost (g score) and totalCost (h score) that each neighbor can have according to current situation (not previous). */

	/* Consider each neighbor of "current_tile" : */

	for (IdType cc = 0; cc <= neighbors_numb; cc++) {
		tentative_actualCost = lb.operator ()(startpoint,neighbors_point[cc]);
		tentative_totalCost = lb.operator ()(startpoint,neighbors_point[cc]) + lb.operator ()(neighbors_point[cc],targetpoint);

		/* If the selected neighbor is in the "closedset" that means it has been used for current net before. So, discard it and go for other neighbors. */

		cont_c = 0;
		for (IdType dd = 0; dd <= iteration; dd++) {
		if (closedset[dd] == neighbors_id[cc]) {
			cont_c = 1;}}

		if (cont_c == 1) {
			continue;}


		/* If the selected neighbor is in the "openset", but its previous totalCost (f score) is lower than its current totalCost,
		 * discard it and go for other neighbors. */

		cont_o = 0;
		for (IdType ff = 0; ff <= N_openset; ff++) {
				if (priorityQueue.getData(neighbors_id[cc]).heapLoc != numeric_limits<IdType>::max()) {
					cont_o = 1;}}

		if ((cont_o == 1) && (tentative_totalCost >= priorityQueue.getData(neighbors_id[cc]).totalCost)) {
			continue;
		}

		/* If the selected neighbor is in the "openset", and its current totalCost (f score) is lower than its previous totalCost,
		* update its information (parent, actualCost, and totalCost) */

		if ((cont_o == 1) && (tentative_totalCost < priorityQueue.getData(neighbors_id[cc]).totalCost)) {
			priorityQueue.updateTile(neighbors_id[cc],tentative_totalCost,tentative_actualCost,current_tile);
		}

		/* If the selected neighbor is not in openset, put it with all of its information in openset (priorityQueue) */
		/* "N_openset" = It specifies the number of tiles in openset. */

		if (cont_o == 0) {
			priorityQueue.updateTile(neighbors_id[cc],tentative_totalCost,tentative_actualCost,current_tile);
			N_openset = N_openset + 1;
		}}

	/* "iteration" = Number of the elements of "closedset" = Number of iterations of While Loop */

	iteration = iteration + 1;}

	/* "target_index": The location of target in closedset. */

	for (IdType gg = 0; gg <= iteration; gg++) {
		if (closedset[gg] == targetTile) {
			target_index = gg;}}

	edges_numb2 = edges.size();

	/* Put the location of the edges of the routed nets in "path" according to their index in "edges" vector. */

	for (IdType ti = 0; ti <= target_index - 1; ti ++) {
	for (IdType en = 0; en <= edges_numb2 - 1; en++) {
	if (((edges[en].tile1 == closedset[ti]) && (edges[en].tile2 == closedset[ti + 1])) || ((edges[en].tile2 == closedset[ti]) && (edges[en].tile1 == closedset[ti + 1])))
	{	path[ti] = en;}}}


	/* End Point of the Code */

	// calculate the accumulated cost of the path
	const CostType finalCost =
			priorityQueue.visited(targetTile) ?
					priorityQueue.getData(targetTile).actualCost :
					numeric_limits<CostType>::max();

	// clean up
	priorityQueue.clear();
	closedset.clear();

	return finalCost;
}

void SimpleGR::doRRR(void) {
	if (params.maxRipIter == 0)
		return;

	vector<IdType> netsToRip;

	unsigned iterations = 0;

	DLMCost dlm(*this);

	cout << endl;

	if (params.maxRipIter != numeric_limits<unsigned>::max()) {
		cout << "Performing at most " << params.maxRipIter
				<< " rip-up and re-route iteration(s)" << endl;
	}

	double startCPU = cpuTime();

	const bool bboxConstrain = true;
	const bool allowOverflow = true;
	const bool donotallowOverflow = false;

	unsigned prevSegs = numeric_limits<unsigned>::max(),
			 prevVias = numeric_limits<unsigned>::max(),
			 prevOverfullEdges = numeric_limits<unsigned>::max();
	double prevOverflow = numeric_limits<double>::max();

	// outer RRR loop, each loop is one RRR iteration
	while (true) {
		// figure our which nets to rip-up and
		// update the costs of routing segments
		for (unsigned i = 0; i < edges.size(); ++i) {
			if (edges[i].usage > edges[i].capacity) {
				netsToRip.insert(netsToRip.end(), edges[i].nets.begin(), edges[i].nets.end());
				// edge history cost increments in each iteration
				// if it is overflowing
				edges[i].historyCost += historyIncrement;
			}
		}

		for (unsigned i = 0; i < nets.size(); ++i) {
			// Pick up the unrouted nets
			if (!nets[i].routed) {
				netsToRip.push_back(i);
			}
		}

		if (netsToRip.size() == 0) {
			cout << "No more nets to rip up, quitting" << endl;
			break;
		}

		// get rid of the duplicated nets in the queue
		sort(netsToRip.begin(), netsToRip.end());
		netsToRip.erase(unique(netsToRip.begin(), netsToRip.end()), netsToRip.end());

		prevSegs = totalSegments;
		prevVias = totalVias;
		prevOverfullEdges = overfullEdges;
		prevOverflow = totalOverflow;

		// sort the queue by the size of the nets
		sort(netsToRip.begin(), netsToRip.end(), CompareByBox(nets));

		cout << endl << "number of nets that need to be ripped up: "
				<< netsToRip.size() << endl;

		// inner RRR loop, each loop rips up and reroute a net.
		for (unsigned i = 0; i < netsToRip.size(); ++i) {
			if (i % 100 == 0)
				cout << i << "\r" << flush;

			const IdType netId = netsToRip[i];

			unsigned overfullEdgesBefore = overfullEdges;

			// rip up the net
			ripUpNet(netId);

			// re-route the net
			if (totalOverflow <= 50. &&
				2. * static_cast<double>(overfullEdges) >= totalOverflow &&
				overfullEdgesBefore != overfullEdges) {
				// try to route without allowing overflow
				routeNet(netId,
							donotallowOverflow,
							bboxConstrain,
							dlm);
				if (!nets[netId].routed) {
					// failing that, allow overflow
					routeNet(netId,
								allowOverflow,
								bboxConstrain,
								dlm);
				}
			} else {
				routeNet(netId,
							allowOverflow,
							bboxConstrain,
							dlm);
			}

			// End as soon as possible
			if (overfullEdges == 0)
				break;
		}

		netsToRip.clear();
		++iterations;
		cout << "after rip-up iteration " << iterations << endl;
		printStatisticsLight();
		double cpuTimeUsed = cpuTime();

		if (iterations >= params.maxRipIter) {
			cout << "Iterations exceeded, quitting" << endl;
			break;
		}
		if (cpuTimeUsed > startCPU + params.timeOut) {
			cout << "Timeout exceeeded, quitting" << endl;
			break;
		}
	}
}

void SimpleGR::greedyImprovement(void) {
	if (totalOverflow > 0) {
		cout << "skipping greedy improvement due to illegal solution" << endl;
		return;
	}

	UnitCost uc(*this);

	const bool donotallowOverflow = false;
	const bool noBBoxConstrain = false;

	vector<IdType> order;
	for (unsigned i = 0; i < nets.size(); ++i) {
			order.push_back(i);
	}
	sort(order.begin(), order.end(), CompareByBox(nets));

	cout << "performing " << params.maxGreedyIter
			<< " greedy improvement iteration(s)" << endl;

	for (unsigned iterations = 1; iterations <= params.maxGreedyIter;
			++iterations) {
		cout << endl << "examining " << order.size() << " nets " << endl;
		for (unsigned i = 0; i < order.size(); ++i) {
			if (i % 100 == 0)
				cout << i << "\r" << flush;

			ripUpNet(order[i]);

			routeNet(order[i], donotallowOverflow, noBBoxConstrain, uc);
		}
		cout << "after greedy improvement iteration " << iterations << endl;
		printStatisticsLight();
	}
}

void SimpleGR::initialRouting(void) {
	unsigned totalNets = 0;
	for (unsigned i = 0; i < nets.size(); ++i) {
		totalNets++;
	}
	cout << "total nets " << totalNets << endl;

	DLMCost dlm(*this);

	cout << "routing flat nets" << endl;
	const bool donotallowOverflow = false;
	routeFlatNets(donotallowOverflow, dlm);
	cout << "CPU time: " << cpuTime() << " seconds " << endl;

	cout << "routing remaining nets" << endl;
	const bool allowOverflow = true;
	routeNets(allowOverflow, dlm);

	printStatistics();
}

double SimpleGR::congEstScoreSegment(IdType netId, const Point &a, const Point &b) {
	double xFirstScore = 0.;

	for (unsigned j = min(a.x, b.x); j < max(a.x, b.x); ++j) {
		unsigned ycoord = min(a.y, b.y);
		double usage = 0., cap = 0., hist = 0., typUsage = 0., useful = 0.;
		for (unsigned k = 0; k < tiles.size(); ++k) {
			unsigned edgeId = tiles[k][ycoord][j].incX;
			if (edgeId < edges.size()) {
				usage += edges[edgeId].usage;
				cap += edges[edgeId].capacity;
				if (edges[edgeId].capacity > 0.) {
					hist += edges[edgeId].historyCost;
					typUsage += minWidths[k] + minSpacings[k];
					useful += 1.;
				}
			}
		}
		hist /= useful;
		typUsage /= useful;
		usage += typUsage;

		if (usage <= cap) {
			xFirstScore += edgeBase + hist * max(1., usage / cap);
		} else {
			xFirstScore += edgeBase
					+ hist * min(powMax, pow(powBase, (usage - cap) / (cap)));
		}
	}

	for (unsigned j = min(a.y, b.y); j < max(a.y, b.y); ++j) {
		unsigned xcoord = max(a.x, b.x);
		double usage = 0., cap = 0., hist = 0., typUsage = 0., useful = 0.;
		for (unsigned k = 0; k < tiles.size(); ++k) {
			unsigned edgeId = tiles[k][j][xcoord].incY;
			if (edgeId < edges.size()) {
				usage += edges[edgeId].usage;
				cap += edges[edgeId].capacity;
				if (edges[edgeId].capacity > 0.) {
					hist += edges[edgeId].historyCost;
					typUsage += minWidths[k] + minSpacings[k];
					useful += 1.;
				}
			}
		}
		hist /= useful;
		typUsage /= useful;
		usage += typUsage;

		if (usage <= cap) {
			xFirstScore += edgeBase + hist * max(1., usage / cap);
		} else {
			xFirstScore += edgeBase
					+ hist * min(powMax, pow(powBase, (usage - cap) / (cap)));
		}
	}

	double yFirstScore = 0.;

	for (unsigned j = min(a.y, b.y); j < max(a.y, b.y); ++j) {
		unsigned xcoord = min(a.x, b.x);
		double usage = 0., cap = 0., hist = 0., typUsage = 0., useful = 0.;
		for (unsigned k = 0; k < tiles.size(); ++k) {
			unsigned edgeId = tiles[k][j][xcoord].incY;
			if (edgeId < edges.size()) {
				usage += edges[edgeId].usage;
				cap += edges[edgeId].capacity;
				if (edges[edgeId].capacity > 0.) {
					hist += edges[edgeId].historyCost;
					typUsage += minWidths[k] + minSpacings[k];
					useful += 1.;
				}
			}
		}
		hist /= useful;
		typUsage /= useful;
		usage += typUsage;

		if (usage <= cap) {
			yFirstScore += edgeBase + hist * max(1., usage / cap);
		} else {
			yFirstScore += edgeBase
					+ hist * min(powMax, pow(powBase, (usage - cap) / (cap)));
		}
	}

	for (unsigned j = min(a.x, b.x); j < max(a.x, b.x); ++j) {
		unsigned ycoord = max(a.y, b.y);
		double usage = 0., cap = 0., hist = 0., typUsage = 0., useful = 0.;
		for (unsigned k = 0; k < tiles.size(); ++k) {
			unsigned edgeId = tiles[k][ycoord][j].incX;
			if (edgeId < edges.size()) {
				usage += edges[edgeId].usage;
				cap += edges[edgeId].capacity;
				if (edges[edgeId].capacity > 0.) {
					hist += edges[edgeId].historyCost;
					typUsage += minWidths[k] + minSpacings[k];
					useful += 1.;
				}
			}
		}
		hist /= useful;
		typUsage /= useful;
		usage += typUsage;

		if (usage <= cap) {
			yFirstScore += edgeBase + hist * max(1., usage / cap);
		} else {
			yFirstScore += edgeBase
					+ hist * min(powMax, pow(powBase, (usage - cap) / (cap)));
		}
	}

	return min(xFirstScore, yFirstScore);
}
