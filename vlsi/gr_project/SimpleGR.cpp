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

	// the code that I'm writing

	IdType current_tile;
	IdType indx = 0;
	IdType t1, t2;
	unsigned iteration = 0;
	unsigned N_openset = 0;
	PQueue closedset;
	CoordType xx, yy, zz;
	CoordType xt1, yt1, zt1;
	CoordType xt2, yt2, zt2;
	vector<IdType> neighbors_id;
	vector<Point> neighbors_point;
	int cont_c = 0;
	int cont_o = 0;
	IdType ngh = 0;
	CostType tentative_actualCost;
	CostType tentative_totalCost;
	IdType target_index;
	int vect_size = 0;
	int type_size = 0;
	int relt = 0;
	int edges_numb = 0;
	int vect_size2 = 0;
	int type_size2 = 0;
	int relt2 = 0;
	int edges_numb2 = 0;
	int edge_new = 0;
	int difx = 0;
	int dify = 0;
	vector<IdType> blocklist;
	int b_numb = 0;
	int neighbors_numb = 0;

	//UnitCost uc;
	//DLMCost dlc;

	//&f = & (nets[netId].segments.size() == 0 ? static_cast<LBFunction&>(uc) : static_cast<LBFunction&>(dlc));


	priorityQueue.updateTile(startTile, 0., 0., numeric_limits<IdType>::max());
	closedset.updateTile(startTile, 0., 0., numeric_limits<IdType>::max());

	const Point startpoint (net.one.x, net.one.y, net.one.z);
	const Point targetpoint (net.two.x, net.two.y, net.two.z);

	priorityQueue.datas[0].actualCost = lb.operator ()(startpoint,startpoint);
	priorityQueue.datas[0].totalCost = priorityQueue.datas[0].actualCost + lb.operator ()(startpoint,targetpoint);
	priorityQueue.datas[0].heapLoc = startTile;

	while (priorityQueue.empty() != 1) {

	current_tile = priorityQueue.getBestTile();

	for (IdType i = 0; i <= N_openset; i++) {
		if (priorityQueue.datas[i].heapLoc == current_tile) {
			indx = i;}}

	priorityQueue.datas[indx].actualCost = lb.operator ()(startpoint,tileIdToXYZ(current_tile));
	priorityQueue.datas[indx].totalCost = priorityQueue.datas[indx].actualCost + lb.operator ()(tileIdToXYZ(current_tile),targetpoint);

	if (iteration >= 1) {
	closedset.updateTile(current_tile,priorityQueue.datas[indx].totalCost,priorityQueue.datas[indx].actualCost,priorityQueue.datas[indx].parent);}

	if (current_tile == targetTile) {
			break;}

	priorityQueue.removeBestTile();

	xx = current_tile % xTiles;
	yy = (current_tile / xTiles) % yTiles;
	zz = current_tile / (xTiles * yTiles);

	neighbors_numb = 0;

	if (xx + 1 <= max(net.one.x, net.two.x)) {
	neighbors_id.push_back(xyzToTileId (xx + 1, yy, zz));
	neighbors_point.push_back(Point (xx + 1, yy, zz));
	neighbors_numb = neighbors_numb + 1;}

	if (xx - 1 >= min(net.one.x, net.two.x)) {
	neighbors_id.push_back(xyzToTileId (xx - 1, yy, zz));
	neighbors_point.push_back(Point (xx - 1, yy, zz));
	neighbors_numb = neighbors_numb + 1;}

	if (yy + 1 <= max(net.one.y, net.two.y)) {
	neighbors_id.push_back(xyzToTileId (xx, yy + 1, zz));
	neighbors_point.push_back(Point (xx, yy + 1, zz));
	neighbors_numb = neighbors_numb + 1;}

	if (yy - 1 >= min(net.one.y, net.two.y)) {
	neighbors_id.push_back(xyzToTileId (xx, yy - 1, zz));
	neighbors_point.push_back(Point (xx, yy - 1, zz));
	neighbors_numb = neighbors_numb + 1;}

	if (zz + 1 <= 1) {
	neighbors_id.push_back(xyzToTileId (xx, yy, zz + 1));
	neighbors_point.push_back(Point (xx, yy, zz + 1));
	neighbors_numb = neighbors_numb + 1;}

	if (zz - 1 >= 0) {
	neighbors_id.push_back(xyzToTileId (xx, yy, zz - 1));
	neighbors_point.push_back(Point (xx, yy, zz - 1));
	neighbors_numb = neighbors_numb + 1;}

	for (int cc = 0; cc <= neighbors_numb; cc++) {
		tentative_actualCost = lb.operator ()(startpoint,neighbors_point[cc]);
		tentative_totalCost = lb.operator ()(startpoint,neighbors_point[cc]) + lb.operator ()(neighbors_point[cc],targetpoint);

		cont_c = 0;
		for (IdType dd = 0; dd <= iteration; dd++) {
				if (closedset.datas[dd].heapLoc == neighbors_id[cc]) {
					cont_c = 1;}}
		if ( cont_c == 1) {
			continue;}

		cont_o = 0;
		for (IdType ff = 0; ff <= N_openset; ff++) {
				if (priorityQueue.datas[ff].heapLoc == neighbors_id[cc]) {
					ngh = ff;
					cont_o = 1;}}

		if ((cont_o == 1) && (tentative_totalCost >= priorityQueue.datas[ngh].totalCost)) {
			continue;
		}

		if ((cont_o == 1) && (tentative_totalCost < priorityQueue.datas[ngh].totalCost)) {
			priorityQueue.datas[ngh].parent = current_tile;
			priorityQueue.datas[ngh].actualCost = tentative_actualCost;
			priorityQueue.datas[ngh].totalCost = tentative_totalCost;
		}

		if (cont_o == 0) {
			priorityQueue.updateTile(neighbors_id[cc],tentative_totalCost,tentative_actualCost,current_tile);
			N_openset = N_openset + 1;
		}}
	iteration = iteration + 1;}

	for (IdType gg = 0; gg <= iteration; gg++) {
		if (closedset.datas[gg].heapLoc == targetTile) {
			target_index = gg;}}

	vect_size = edges.size();
	type_size = sizeof (Edge);
	relt = 4 / type_size;
	edges_numb = vect_size * relt;

	for (int edge_i = 0; edge_i <= edges_numb; edge_i++) {
	for (IdType tile_i = 0; tile_i <= target_index - 1; tile_i++) {
	if (((edges[edge_i].tile1 == closedset.datas[tile_i].heapLoc) && (edges[edge_i].tile2 == closedset.datas[tile_i + 1].heapLoc)) || ((edges[edge_i].tile2 == closedset.datas[tile_i].heapLoc) && (edges[edge_i].tile1 == closedset.datas[tile_i + 1].heapLoc)))
	{

		edges[edge_i].usage = edges[edge_i].usage + max(minWidths[edges[edge_i].layer], netMinWidth) + minSpacings[edges[edge_i].layer];
		edges[edge_i].nets.push_back(netId);
		edges[edge_i].historyCost = edges[edge_i].historyCost + f.operator ()(edges[edge_i].tile1,edges[edge_i].tile2);

	}}}



	edge_new = edges_numb + 1;
	for (IdType tile_i = 0; tile_i <= target_index - 1; tile_i++) {
	for (int edge_i = 0; edge_i <= edges_numb; edge_i++) {
	if (!((((edges[edge_i].tile1 == closedset.datas[tile_i].heapLoc) && (edges[edge_i].tile2 == closedset.datas[tile_i + 1].heapLoc)) || ((edges[edge_i].tile2 == closedset.datas[tile_i].heapLoc) && (edges[edge_i].tile1 == closedset.datas[tile_i + 1].heapLoc)))))
	{
		// Make Edge // using edges_new //edges_new++

		t1 = closedset.datas[tile_i].heapLoc;
		t2 = closedset.datas[tile_i + 1].heapLoc;

		xt1 = t1 % xTiles;
		yt1 = (t1 / xTiles) % yTiles;
		zt1 = t1 / (xTiles * yTiles);

		xt2 = t2 % xTiles;
		yt2 = (t2 / xTiles) % yTiles;
		zt2 = t2 / (xTiles * yTiles);

		if (zt1 == zt2) {

		edges[edge_new].tile1 = closedset.datas[tile_i].heapLoc;
		edges[edge_new].tile2 = closedset.datas[tile_i + 1].heapLoc;

		if ((zt1 == 0) && (zt2 == 0)) {
		edges[edge_new].layer = 0;}

		if ((zt1 == 1) && (zt2 == 1)) {
		edges[edge_new].layer = 1;}

		dify = abs(yt2 - yt1);
		difx = abs(xt2 - xt1);

		if (dify == 1) {
		edges[edge_new].capacity = tileWidth;
		edges[edge_new].type = HORIZ;}

		if (difx == 1) {
		edges[edge_new].capacity = tileHeight;
		edges[edge_new].type = VERT;}

		edges[edge_new].usage = netMinWidth;
		edges[edge_new].historyCost = f.operator ()(edges[edge_new].tile1,edges[edge_new].tile2);
		edges[edge_new].nets.push_back(netId);}

		if (zt1 != zt2) {
		edges[edge_new].tile1 = numeric_limits < IdType > ::max();
		edges[edge_new].tile2 = numeric_limits < IdType > ::max();
		edges[edge_new].layer = numeric_limits < CoordType > ::max();
		edges[edge_new].capacity = 0;
		edges[edge_new].usage = 0;
		edges[edge_new].historyCost = 1;
		edges[edge_new].type = VIA;}

		edge_new = edge_new + 1;

	}}}

	vect_size2 = edges.size();
	type_size2 = sizeof (Edge);
	relt2 = 4 / type_size2;
	edges_numb2 = vect_size2 * relt2;

	for (IdType ti = 0; ti <= target_index - 1; ti ++) {
	for (int en = 0; en <= edges_numb2; en++) {
	if (((edges[en].tile1 == closedset.datas[ti].heapLoc) && (edges[en].tile2 == closedset.datas[ti + 1].heapLoc)) || ((edges[en].tile2 == closedset.datas[ti].heapLoc) && (edges[en].tile1 == closedset.datas[ti + 1].heapLoc)))
	{	path[ti] = en;}}}

	b_numb = blocklist.size();

	for (int qq = 0; qq <= edges_numb2; qq++) {
	for (int hh = 0; hh <= edges_numb2; hh++) {

	if (((edges[qq].tile1 == edges[hh].tile1) || (edges[qq].tile2 == edges[hh].tile2) || (edges[qq].tile1 == edges[hh].tile2) || (edges[qq].tile2 == edges[hh].tile1)) && (hh != qq)) {
	if ((edges[qq].usage >= edges[qq].capacity) && (edges[hh].usage >= edges[hh].capacity)) {

		if ((edges[qq].tile1 == edges[hh].tile1) || (edges[qq].tile1 == edges[hh].tile2)) {
		blocklist[b_numb] = edges[qq].tile1;}

		if ((edges[qq].tile2 == edges[hh].tile2) || (edges[qq].tile2 == edges[hh].tile1)) {
		blocklist[b_numb] = edges[qq].tile2;}

		b_numb = b_numb + 1;}}

	}}


	// A* search kernel
	// Loop until all "frontiers" in the priority queue are exhausted, or when
	// the sink tile is found.

	// ====> A* search code starts here  <====

	// A* search code ends here

	// now build up the path, if we found one

	// ====> back-tracing code starts here <====

	// back-tracking code ends here

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
