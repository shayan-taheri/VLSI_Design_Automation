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
	priorityQueue.updateTile(startTile, 0., 0., numeric_limits<IdType>::max());

	// Instantiate the Cost function to calculate the Manhattan distance between
	// two tiles.
	const LBFunction &lb =
			(nets[netId].segments.size() == 0 ?
					static_cast<LBFunction&>(em) : static_cast<LBFunction&>(lbm));

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
