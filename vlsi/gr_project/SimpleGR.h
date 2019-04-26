#ifndef _SIMPLEGR_H_
#define _SIMPLEGR_H_

#include <algorithm>

typedef unsigned short CoordType;
typedef unsigned IdType;
typedef unsigned LenType;
typedef unsigned short CapType;
typedef double CostType;

class Point {
public:
	CoordType x, y, z;

	Point() :
			x(0), y(0), z(0) {
	}
	Point(CoordType X, CoordType Y, CoordType Z) :
			x(X), y(Y), z(Z) {
	}
	Point(const Point &orig) :
			x(orig.x), y(orig.y), z(orig.z) {
	}

	const Point &operator=(const Point &assign) {
		x = assign.x;
		y = assign.y;
		z = assign.z;
		return *this;
	}
};

inline bool operator==(const Point &a, const Point &b) {
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

inline bool operator!=(const Point &a, const Point &b) {
	return a.x != b.x || a.y != b.y || a.z != b.z;
}

inline bool operator<(const Point &a, const Point &b) {
	return a.x < b.x || (a.x == b.x && a.y < b.y)
			|| (a.x == b.x && a.y == b.y && a.z < b.z);
}

template<typename T> class MemEfficientMultiSet {
private:
	typedef pair<T, size_t> elementType;
	vector<elementType> theData;

public:

	MemEfficientMultiSet() :
			theData(0) {
	}
	MemEfficientMultiSet(const MemEfficientMultiSet &orig) :
			theData(orig.theData) {
	}

	void clear(void) {
		theData.clear();
	}

	void insert(const T &data) {
		typename vector<elementType>::iterator pos = std::lower_bound(
				theData.begin(), theData.end(),
				std::make_pair(data, static_cast<size_t>(0)));
		if (pos != theData.end() && pos->first == data) {
			pos->second += 1;
		} else {
			theData.insert(pos, make_pair(data, 1));
		}
	}

	void remove(const T &data) {
		typename vector<elementType>::iterator pos = std::lower_bound(
				theData.begin(), theData.end(),
				std::make_pair(data, static_cast<size_t>(0)));
		assert(pos != theData.end() && pos->first == data);
		if (pos->second > 1) {
			pos->second -= 1;
		} else {
			theData.erase(pos);
		}
	}

	bool contains(const T &data) const {
		typename vector<elementType>::const_iterator pos = std::lower_bound(
				theData.begin(), theData.end(),
				std::make_pair(data, static_cast<size_t>(0)));
		return pos != theData.end() && pos->first == data;
	}

	size_t getCount(const T &data) const {
		typename vector<elementType>::const_iterator pos = std::lower_bound(
				theData.begin(), theData.end(),
				std::make_pair(data, static_cast<size_t>(0)));
		if (pos != theData.end() && pos->first == data) {
			return pos->second;
		} else {
			return static_cast<size_t>(0);
		}
	}

	size_t size(void) const {
		return theData.size();
	}

	typedef typename vector<elementType>::iterator iterator;

	iterator begin(void) {
		return theData.begin();
	}

	iterator end(void) {
		return theData.end();
	}
};

typedef MemEfficientMultiSet<IdType> SegmentHash;

class Net {
public:
	LenType numSegments, numVias;
	CapType wireWidth;
	Point one, two;
	bool routed;
	vector<IdType> segments;

	Net() :
			numSegments(0),
			numVias(0),
			wireWidth(0),
			one(0,0,0),
			two(0,0,0),
			routed(false) {
	}
	Net(const Net &orig) :
			numSegments(orig.numSegments),
			numVias(orig.numVias),
			wireWidth(orig.wireWidth),
			one(orig.one),
			two(orig.two),
			routed(orig.routed),
			segments(orig.segments)
	{ }
};

enum EdgeType {
	HORIZ, VERT, VIA
};

class Edge {
public:
	IdType tile1, tile2;
	CoordType layer;
	CapType capacity, usage;
	CostType historyCost;
	EdgeType type;
	vector<IdType> nets;

	Edge() :
			tile1(numeric_limits < IdType > ::max()), tile2(
					numeric_limits < IdType > ::max()), layer(
					numeric_limits < CoordType > ::max()), capacity(0), usage(
					0), historyCost(1), type(VIA) {
	}
	Edge(const Edge &orig) :
			tile1(orig.tile1), tile2(orig.tile2), layer(orig.layer), capacity(
					orig.capacity), usage(orig.usage), historyCost(
					orig.historyCost), type(orig.type), nets(orig.nets) {
	}
};

class Tile {
public:
	IdType incX, decX, incY, decY, incZ, decZ;

	Tile() :
			incX(numeric_limits < IdType > ::max()), decX(
					numeric_limits < IdType > ::max()), incY(
					numeric_limits < IdType > ::max()), decY(
					numeric_limits < IdType > ::max()), incZ(
					numeric_limits < IdType > ::max()), decZ(
					numeric_limits < IdType > ::max()) {
	}
	Tile(const Tile &orig) :
			incX(orig.incX), decX(orig.decX), incY(orig.incY), decY(orig.decY), incZ(
					orig.incZ), decZ(orig.decZ) {
	}
};

class CompareByBox {
	const vector<Net> &nets;
public:
	CompareByBox(const vector<Net> &n) :
			nets(n) {
	}

	bool operator()(const IdType a, const IdType b) const;
};

class BitBoard {
private:
	vector<IdType> setBits;
	vector<bool> bits;
public:

	BitBoard() :
			setBits(0), bits(0) {
	}
	BitBoard(IdType size) :
			setBits(0), bits(size, false) {
	}
	const vector<IdType> &getSetBits(void) {
		return setBits;
	}
	void setBit(IdType id);
	bool isBitSet(IdType id) const;
	void clear(void);
	void resize(IdType size);
};

class PQueue {
	class tileData {
	public:
		IdType heapLoc;      // location in the priority queue's heap. If it equals to numeric_limits<IdType>::max(), that means the tile isn't in the queue.
		CostType totalCost;  // actualCost + Manhattan distance cost
		CostType actualCost; // aggregated edge cost along the path
		IdType parent;       // tile where it propagated from, mostly used by the back-trace process
	};

	vector<tileData> data;
	BitBoard dataValid;
	vector<IdType> heap;

public:

	PQueue() :
			data(), dataValid(), heap() {
	}
	;
	vector<tileData> datas;
	void resize(unsigned newSize) {
		data.resize(newSize);
		dataValid.resize(newSize);
	}
	bool empty() const {
		return heap.empty();
	}
	void clear();

	IdType getBestTile(void) const;
	void removeBestTile(void);
	void updateTile(IdType tileId, CostType totalCost, CostType actualCost,
					IdType parent);
	const tileData& getData(IdType tileId) const;
	bool visited(IdType tileId) const;
};

class SimpleGR;

class CostFunction {
public:
	virtual CostType operator()(IdType, IdType) const = 0;
	virtual CostType viaCost(void) const = 0;
	virtual ~CostFunction() { }
};

class DLMCost: public CostFunction {
	const SimpleGR &simplegr;
public:
	DLMCost(const SimpleGR &_simplegr) :
			simplegr(_simplegr) {
	}
	CostType operator()(IdType, IdType) const;
	CostType viaCost(void) const;
};

class UnitCost: public CostFunction {
	const SimpleGR &simplegr;
public:
	UnitCost(const SimpleGR &_simplegr) :
			simplegr(_simplegr) {
	}
	CostType operator()(IdType, IdType) const;
	CostType viaCost(void) const;
};

class LBFunction {
public:
	virtual double operator()(const Point &a, const Point &b) const = 0;
	virtual ~LBFunction() { }
};

class LBManhattan: public LBFunction {
	const SimpleGR &simplegr;
public:
	LBManhattan(const SimpleGR &_simplegr) :
			simplegr(_simplegr) {
	}
	CostType operator()(const Point &a, const Point &b) const;
};

class ExactManhattan: public LBFunction {
	const SimpleGR &simplegr;
public:
	ExactManhattan(const SimpleGR &_simplegr) :
			simplegr(_simplegr) {
	}
	CostType operator()(const Point &a, const Point &b) const;
};

class SimpleGRParams {
	friend class SimpleGR;

private:
	void setDefault(void);
	void print(void) const;

public:

	bool layerAssign;
	unsigned maxRipIter, maxGreedyIter;
	double timeOut;
	string outputFile;

	SimpleGRParams(void) {
		setDefault();
	}
	SimpleGRParams(int argc, char **argv);

	static void usage(const char* exename);
};

class SimpleGR {
	friend class DLMCost;
	friend class UnitCost;
	friend class LBManhattan;
	friend class ExactManhattan;

private:

// benchmark stats
	IdType xTiles, yTiles, numLayers, routableNets, nonViaEdges,
			nonViaFullEdges;
	LenType minX, minY, tileWidth, tileHeight, halfWidth, halfHeight;

// routing stats
	unsigned totalOverflow, overfullEdges, totalSegments, totalVias;

// data structures
	vector<CapType> vertCaps, horizCaps, minWidths, minSpacings, viaSpacings;
	vector<Net> nets;
	vector<string> netNames;
	vector<IdType> netIds;
	vector<vector<vector<Tile> > > tiles, fulltiles;
	vector<Edge> edges, fulledges;
	PQueue priorityQueue;
	map<string, IdType> netNameToIdx;

	ExactManhattan em;
	LBManhattan lbm;

	SimpleGRParams params;

	IdType xyzToTileId(CoordType x, CoordType y, CoordType z);
	Point tileIdToXYZ(IdType id);

public:

	SimpleGR(const SimpleGRParams &_params = SimpleGRParams()) :
			xTiles(0), yTiles(0), numLayers(0), routableNets(0), nonViaEdges(0), nonViaFullEdges(0),
			minX(0), minY(0), tileWidth(0), tileHeight(0), halfWidth(0), halfHeight(0),
			totalOverflow(0), overfullEdges(0), totalSegments(0), totalVias(0),
			em(*this), lbm(*this), params(_params)
	{ }

	void parseInput(const char *filename);
	void parseInputMapper(const char *filename);
	void parseSolution(const char *filename);

	void writeRoutes(const string &filename);

	void buildGrid(bool skipEmpty);

	double congEstScoreSegment(IdType netId, const Point &a, const Point &b);

	void addSegment(const IdType netId, IdType edgeId);
	void ripUpSegment(const IdType netId, IdType edgeId);
	void ripUpNet(const IdType netId);

	void routeFlatNets(bool allowOverflow, const CostFunction &f);
	CostType routeNet(const IdType netId, bool allowOverflow,
							bool bboxConstrain, const CostFunction &f);
	void routeNets(bool allowOverflow, const CostFunction &f);

	CostType routeMaze(const IdType netId, bool allowOverflow,
						const Point &botleft, const Point &topright,
						const CostFunction &f, vector<IdType> &path);

	void initialRouting(void);
	void doRRR(void);
	void greedyImprovement(void);

	void printParams(void) {
		params.print();
	}
	void printStatistics(bool checkRouted = true, bool final = false);
	void printStatisticsLight(void);

	void plotXPM(const string &filename);
};

double cpuTime(void);

const CostType powMax = 1.e12;
const CostType powBase = 5.;
const CostType edgeBase = 2.;
const CostType viaFactor = 3.;
const CostType epsilon = 1.;
const CostType historyIncrement = 0.4;

const string SimpleGRversion = "1.0";

#endif
