#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <limits>
#include <climits>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <cstdlib>

using namespace std;

#include "SimpleGR.h"

void expect(string s1, string s2) {
	if (s1 != s2) {
		cout << "Parsing error. Expected `" << s2 << "' but got `" << s1
				<< "' instead" << endl;
		exit(0);
	}
}

double scoreTree(const vector<pair<Point, Point> > &tree) {
	double score = 0.;

	for (unsigned i = 0; i < tree.size(); ++i) {
		score += static_cast<double>(max(tree[i].first.x, tree[i].second.x)
				- min(tree[i].first.x, tree[i].second.x));
		score += static_cast<double>(max(tree[i].first.y, tree[i].second.y)
				- min(tree[i].first.y, tree[i].second.y));
	}

	return score;
}

void SimpleGR::parseInput(const char *filename) {
	ifstream infile(filename);

	if (!infile.good()) {
		cout << "Could not open `" << filename << "' for reading" << endl;
		exit(0);
	} else {
		cout << "Reading from `" << filename << "' ..." << endl;
	}

	string tmp;

	infile >> tmp;
	expect(tmp, string("grid"));

	infile >> xTiles >> yTiles >> numLayers;

	if (numLayers <= 2)
		params.layerAssign = false;

	cout << "grid size " << xTiles << "x" << yTiles << endl;

	infile >> tmp;
	expect(tmp, string("vertical"));
	infile >> tmp;
	expect(tmp, string("capacity"));

	for (unsigned i = 0; i < numLayers; ++i) {
		unsigned cap;
		infile >> cap;
		vertCaps.push_back(cap);
	}

	infile >> tmp;
	expect(tmp, string("horizontal"));
	infile >> tmp;
	expect(tmp, string("capacity"));

	for (unsigned i = 0; i < numLayers; ++i) {
		unsigned cap;
		infile >> cap;
		horizCaps.push_back(cap);
	}

	infile >> tmp;
	expect(tmp, string("minimum"));
	infile >> tmp;
	expect(tmp, string("width"));

	for (unsigned i = 0; i < numLayers; ++i) {
		unsigned minwidth;
		infile >> minwidth;
		minWidths.push_back(minwidth);
	}

	infile >> tmp;
	expect(tmp, string("minimum"));
	infile >> tmp;
	expect(tmp, string("spacing"));

	for (unsigned i = 0; i < numLayers; ++i) {
		unsigned minspacing;
		infile >> minspacing;
		minSpacings.push_back(minspacing);
	}

	infile >> tmp;
	expect(tmp, string("via"));
	infile >> tmp;
	expect(tmp, string("spacing"));

	for (unsigned i = 0; i < numLayers; ++i) {
		unsigned viaspacing;
		infile >> viaspacing;
		viaSpacings.push_back(viaspacing);
	}

	infile >> minX >> minY >> tileWidth >> tileHeight;

	halfWidth = static_cast<LenType>(trunc(0.5 * tileWidth));
	halfHeight = static_cast<LenType>(trunc(0.5 * tileHeight));

	infile >> tmp;
	expect(tmp, string("num"));
	infile >> tmp;
	expect(tmp, string("net"));

	unsigned numNets;

	infile >> numNets;

	cout << "total nets " << numNets << endl;

	nets.reserve(numNets);

	for (unsigned i = 0; i < numNets; ++i) {
		Net temp;
		unsigned numPins;
		string name;
		IdType id;

		infile >> name >> id >> numPins >> temp.wireWidth;

		//For this project, all nets have exactly 2 pins.
		assert(numPins == 2);

		// translate absolute coords to grid coords
		double pinX, pinY;
		unsigned layer;

		infile >> pinX >> pinY >> layer;
		temp.one.x = static_cast<unsigned>(floor((pinX - minX) / tileWidth));
		temp.one.y = static_cast<unsigned>(floor((pinY - minY) / tileHeight));
		temp.one.z = layer - 1;

		infile >> pinX >> pinY >> layer;
		temp.two.x = static_cast<unsigned>(floor((pinX - minX) / tileWidth));
		temp.two.y = static_cast<unsigned>(floor((pinY - minY) / tileHeight));
		temp.two.z = layer - 1;


		if (temp.one != temp.two) {
			++routableNets;

			netNameToIdx[name] = nets.size();
			nets.push_back(temp);
			netNames.push_back(name);
			netIds.push_back(id);
		} else {
			netNameToIdx[name] = numeric_limits<IdType>::max();
		}
	}

	// build the routing grid graph
	bool skipEmpty = true;
	buildGrid(skipEmpty);

	unsigned capacityChanges;
	infile >> capacityChanges;

	for (unsigned i = 0; i < capacityChanges; ++i) {
		unsigned gridCol1, gridRow1, layer1, gridCol2, gridRow2, layer2, newCap;

		infile >> gridCol1 >> gridRow1 >> layer1 >> gridCol2 >> gridRow2
				>> layer2 >> newCap;

		assert(layer1 == layer2);
		if (gridCol1 == gridCol2) {
			// vert segment
			assert(gridRow1 == gridRow2+1 || gridRow1+1 == gridRow2);
			if (fulltiles[layer1 - 1][min(gridRow1, gridRow2)][gridCol1].incY
					== numeric_limits<IdType>::max()) {
				if (newCap != 0.) {
					cout << "Increasing capacity on a previously empty edge."
							<< endl;
					exit(0);
				}
			} else {
				fulledges[fulltiles[layer1 - 1][min(gridRow1, gridRow2)][gridCol1].incY].capacity =
						newCap;
				if (newCap == 0.) {
					fulltiles[layer1 - 1][min(gridRow1, gridRow2)][gridCol1].incY =
							numeric_limits<IdType>::max();
					fulltiles[layer1 - 1][max(gridRow1, gridRow2)][gridCol1].decY =
							numeric_limits<IdType>::max();
				}
			}
		} else if (gridRow1 == gridRow2) {
			// horiz segment
			assert(gridCol1 == gridCol2+1 || gridCol1+1 == gridCol2);
			if (fulltiles[layer1 - 1][gridRow1][min(gridCol1, gridCol2)].incX
					== numeric_limits<IdType>::max()) {
				if (newCap != 0.) {
					cout << "Increasing capacity on a previously empty edge."
							<< endl;
					exit(0);
				}
			} else {
				fulledges[fulltiles[layer1 - 1][gridRow1][min(gridCol1,
																gridCol2)].incX].capacity =
						newCap;
				if (newCap == 0.) {
					fulltiles[layer1 - 1][gridRow1][min(gridCol1, gridCol2)].incX =
							numeric_limits<IdType>::max();
					fulltiles[layer1 - 1][gridRow1][max(gridCol1, gridCol2)].decX =
							numeric_limits<IdType>::max();
				}
			}
		} else {
			cout << "Bad capacity change" << endl;
			exit(0);
		}
	}

	tiles.clear();
	tiles.swap(fulltiles);
	edges.clear();
	edges.swap(fulledges);
	nonViaEdges = nonViaFullEdges;

	infile.close();
}


void SimpleGR::parseInputMapper(const char *filename) {
	ifstream infile(filename);

	if (!infile.good()) {
		cout << "Could not open `" << filename << "' for reading" << endl;
		exit(0);
	} else {
		cout << "Reading from `" << filename << "' ..." << endl;
	}

	string tmp;

	infile >> tmp;
	expect(tmp, string("grid"));

	infile >> xTiles >> yTiles >> numLayers;

	if (numLayers <= 2)
		params.layerAssign = false;

	cout << "grid size " << xTiles << "x" << yTiles << endl;

	infile >> tmp;
	expect(tmp, string("vertical"));
	infile >> tmp;
	expect(tmp, string("capacity"));

	for (unsigned i = 0; i < numLayers; ++i) {
		unsigned cap;
		infile >> cap;
		vertCaps.push_back(cap);
	}

	infile >> tmp;
	expect(tmp, string("horizontal"));
	infile >> tmp;
	expect(tmp, string("capacity"));

	for (unsigned i = 0; i < numLayers; ++i) {
		unsigned cap;
		infile >> cap;
		horizCaps.push_back(cap);
	}

	infile >> tmp;
	expect(tmp, string("minimum"));
	infile >> tmp;
	expect(tmp, string("width"));

	for (unsigned i = 0; i < numLayers; ++i) {
		unsigned minwidth;
		infile >> minwidth;
		minWidths.push_back(minwidth);
	}

	infile >> tmp;
	expect(tmp, string("minimum"));
	infile >> tmp;
	expect(tmp, string("spacing"));

	for (unsigned i = 0; i < numLayers; ++i) {
		unsigned minspacing;
		infile >> minspacing;
		minSpacings.push_back(minspacing);
	}

	infile >> tmp;
	expect(tmp, string("via"));
	infile >> tmp;
	expect(tmp, string("spacing"));

	for (unsigned i = 0; i < numLayers; ++i) {
		unsigned viaspacing;
		infile >> viaspacing;
		viaSpacings.push_back(viaspacing);
	}

	infile >> minX >> minY >> tileWidth >> tileHeight;

	halfWidth = static_cast<LenType>(trunc(0.5 * tileWidth));
	halfHeight = static_cast<LenType>(trunc(0.5 * tileHeight));

	infile >> tmp;
	expect(tmp, string("num"));
	infile >> tmp;
	expect(tmp, string("net"));

	unsigned numNets;

	infile >> numNets;

	cout << "total nets " << numNets << endl;

	nets.reserve(numNets);

	for (unsigned i = 0; i < numNets; ++i) {
		Net temp;
		unsigned numPins;
		string name;
		IdType id;

		infile >> name >> id >> numPins >> temp.wireWidth;

		//For this project, all nets have exactly 2 pins.
		assert(numPins == 2);

		// translate absolute coords to grid coords
		double pinX, pinY;
		unsigned layer;

		infile >> pinX >> pinY >> layer;
		temp.one.x = static_cast<unsigned>(floor((pinX - minX) / tileWidth));
		temp.one.y = static_cast<unsigned>(floor((pinY - minY) / tileHeight));
		temp.one.z = layer - 1;

		infile >> pinX >> pinY >> layer;
		temp.two.x = static_cast<unsigned>(floor((pinX - minX) / tileWidth));
		temp.two.y = static_cast<unsigned>(floor((pinY - minY) / tileHeight));
		temp.two.z = layer - 1;

		if (temp.one != temp.two) {
			++routableNets;

			netNameToIdx[name] = nets.size();
			nets.push_back(temp);
			netNames.push_back(name);
			netIds.push_back(id);
		} else {
			netNameToIdx[name] = numeric_limits<IdType>::max();
		}
	}

	// build the routing grid graph
	bool skipEmpty = false;
	buildGrid(skipEmpty);

	unsigned capacityChanges;
	infile >> capacityChanges;

	for (unsigned i = 0; i < capacityChanges; ++i) {
		unsigned gridCol1, gridRow1, layer1, gridCol2, gridRow2, layer2, newCap;

		infile >> gridCol1 >> gridRow1 >> layer1 >> gridCol2 >> gridRow2
				>> layer2 >> newCap;

		assert(layer1 == layer2);
		if (gridCol1 == gridCol2) {
			// vert segment
			assert(gridRow1 == gridRow2+1 || gridRow1+1 == gridRow2);
			fulledges[fulltiles[layer1 - 1][min(gridRow1, gridRow2)][gridCol1].incY].capacity =
					newCap;
		} else if (gridRow1 == gridRow2) {
			// horiz segment
			assert(gridCol1 == gridCol2+1 || gridCol1+1 == gridCol2);
			fulledges[fulltiles[layer1 - 1][gridRow1][min(gridCol1, gridCol2)].incX].capacity =
					newCap;
		} else {
			assert(0);
		}
	}

	tiles.clear();
	tiles.swap(fulltiles);
	edges.clear();
	edges.swap(fulledges);
	nonViaEdges = nonViaFullEdges;

	infile.close();
}

void SimpleGR::parseSolution(const char* filename) {
	ifstream infile(filename);

	if (!infile.good()) {
		cout << "Could not open `" << filename << "' for reading" << endl;
		exit(0);
	} else {
		cout << "Reading from `" << filename << "' ..." << endl;
	}

	string netname;

	infile >> netname;

	while (infile.good()) {
		if (netNameToIdx.find(netname) == netNameToIdx.end()) {
			cout << "unknown net with name `" << netname << "'" << endl;
			exit(0);
		}

		unsigned netId = netNameToIdx[netname];

		char junk;
		string restofline;
		getline(infile, restofline);

		while (infile.peek() == '(') {
			unsigned x1, y1, z1, x2, y2, z2;
			infile >> junk >> x1 >> junk >> y1 >> junk >> z1 >> junk >> junk
					>> junk >> x2 >> junk >> y2 >> junk >> z2;
			getline(infile, restofline);

			unsigned tilex1, tiley1, tilez1, tilex2, tiley2, tilez2;

			tilex1 = static_cast<unsigned>(floor((x1 - minX) / tileWidth));
			tilex2 = static_cast<unsigned>(floor((x2 - minX) / tileWidth));
			tiley1 = static_cast<unsigned>(floor((y1 - minY) / tileHeight));
			tiley2 = static_cast<unsigned>(floor((y2 - minY) / tileHeight));
			tilez1 = z1 - 1;
			tilez2 = z2 - 1;

			if (tilex1 >= xTiles) {
				cout << "illegal x position " << x1 << endl;
				exit(0);
			}
			if (tiley1 >= yTiles) {
				cout << "illegal y position " << y1 << endl;
				exit(0);
			}
			if (tilez1 >= numLayers) {
				cout << "illegal layer " << z1 << endl;
				exit(0);
			}

			if (tilex2 >= xTiles) {
				cout << "illegal x position " << x2 << endl;
				exit(0);
			}
			if (tiley2 >= yTiles) {
				cout << "illegal y position " << y2 << endl;
				exit(0);
			}
			if (tilez2 >= numLayers) {
				cout << "illegal layer " << z2 << endl;
				exit(0);
			}

			x1 = tilex1;
			y1 = tiley1;
			z1 = tilez1;
			x2 = tilex2;
			y2 = tiley2;
			z2 = tilez2;

			if (x1 != x2) {
				for (unsigned j = min(x1, x2); j < max(x1, x2); ++j) {
					IdType edgeId = tiles[z1][y1][j].incX;
					assert(edgeId != numeric_limits<IdType>::max());
					addSegment(netId, edgeId);
				}
			} else if (y1 != y2) {
				for (unsigned j = min(y1, y2); j < max(y1, y2); ++j) {
					IdType edgeId = tiles[z1][j][x1].incY;
					assert(edgeId != numeric_limits<IdType>::max());
					addSegment(netId, edgeId);
				}
			} else {
				for (unsigned j = min(z1, z2); j < max(z1, z2); ++j) {
					IdType edgeId = tiles[j][y1][x1].incZ;
					assert(edgeId != numeric_limits<IdType>::max());
					addSegment(netId, edgeId);
				}
			}
		}

		getline(infile, restofline); // the trailing !

		// read in the next one
		infile >> netname;
	}
}


void SimpleGR::writeRoutes(const string &filename) {
	ofstream outfile(filename.c_str());

	if (!outfile.good()) {
		cout << "Could not open `" << filename << "' for writing." << endl;
		return;
	}

	cout << "Writing `" << filename << "' ..." << flush;

	for (unsigned i = 0; i < nets.size(); ++i) {
		bool netRouted = nets[i].routed;

		if (netRouted) {
			vector<unsigned> usedEdges(nets[i].segments);

			sort(usedEdges.begin(), usedEdges.end());

			vector<pair<Point, Point> > segments;

			// add routed segments
			assert(usedEdges.size() > 0);
			Point startPt = tileIdToXYZ(edges[usedEdges[0]].tile1), endPt =
					tileIdToXYZ(edges[usedEdges[0]].tile2);
			for (unsigned j = 1; j < usedEdges.size(); ++j) {
				unsigned prevEdge = usedEdges[j - 1];
				unsigned currEdge = usedEdges[j];

				if (edges[prevEdge].type != edges[currEdge].type
						|| edges[prevEdge].tile2 != edges[currEdge].tile1) {
					segments.push_back(make_pair(startPt, endPt));
					startPt = tileIdToXYZ(edges[currEdge].tile1);
					endPt = tileIdToXYZ(edges[currEdge].tile2);
				} else {
					endPt = tileIdToXYZ(edges[currEdge].tile2);
				}
			}
			segments.push_back(make_pair(startPt, endPt));

			outfile << netNames[i] << " " << netIds[i] << " " << segments.size()
					<< endl;
			for (unsigned j = 0; j < segments.size(); ++j) {
				outfile << "("
						<< minX
								+ tileWidth
										* static_cast<double>(segments[j].first.x)
								+ halfWidth << ","
						<< minY
								+ tileHeight
										* static_cast<double>(segments[j].first.y)
								+ halfHeight;
				outfile << "," << segments[j].first.z + 1;
				outfile << ")-("
						<< minX
								+ tileWidth
										* static_cast<double>(segments[j].second.x)
								+ halfWidth << ","
						<< minY
								+ tileHeight
										* static_cast<double>(segments[j].second.y)
								+ halfHeight;
				outfile << "," << segments[j].second.z + 1;
				outfile << ")" << endl;
			}
			outfile << "!" << endl;
		}
	}

	cout << " done" << endl;
}


const char* coolMap[] = {
/* columns rows colors chars-per-pixel */
"a c #ffffff", "b c #00009f", "c c #0000af", "d c #0000bf", "e c #0000cf",
		"f c #0000df", "g c #0000ef", "h c #0000ff", "i c #000fff",
		"j c #001fff", "k c #002fff", "l c #003fff", "m c #004fff",
		"n c #005fff", "o c #006fff", "p c #007fff", "q c #008fff",
		"r c #009fff", "s c #00afff", "t c #00bfff", "u c #00cfff",
		"v c #00dfff", "w c #00efff", "x c #00ffff", "y c #0fffef",
		"z c #1fffdf", "A c #2fffcf", "B c #3fffbf", "C c #4fffaf",
		"D c #5fff9f", "E c #6fff8f", "F c #7fff7f", "G c #8fff6f",
		"H c #9fff5f", "I c #afff4f", "J c #bfff3f", "K c #cfff2f",
		"L c #dfff1f", "M c #efff0f", "N c #ffff00", "O c #ffef00",
		"P c #ffdf00", "Q c #ffcf00", "R c #ffbf00", "S c #ffaf00",
		"T c #ff9f00", "U c #ff8f00", "V c #ff7f00", "W c #ff6f00",
		"X c #ff5f00", "Y c #ff4f00", "Z c #ff3f00", "0 c #ff2f00",
		"1 c #ff1f00", "2 c #ff0f00", "3 c #ff0000", "4 c #ef0000",
		"5 c #df0000", "6 c #cf0000", "7 c #bf0000", "8 c #af0000",
		"9 c #9f0000", ", c #8f0000", ". c #7f0000" };

const char* Colors[] = { "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k",
		"l", "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y",
		"z", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M",
		"N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "0",
		"1", "2", "3", "4", "5", "6", "7", "8", "9", ",", "." };

void SimpleGR::plotXPM(const string &filename) {
	const unsigned numColors = 64;

	string fullfilename = filename + string(".xpm");

	ofstream xpmFile(fullfilename.c_str());

	xpmFile << "/* XPM */" << endl;
	xpmFile << "static char *congestion[] = {" << endl;
	xpmFile << "/* columns rows colors chars-per-pixel */" << endl;
	xpmFile << "\"" << xTiles - 1 << " " << yTiles - 1 << " " << numColors
			<< " 1\"," << endl;
	for (unsigned i = 0; i < numColors; ++i)
		xpmFile << "\"" << coolMap[i] << "\"," << endl;
	xpmFile << "/* pixels */" << endl;

	vector<vector<double> > image;

	double maxRatio = 1.;

	for (unsigned rj = 2; rj <= yTiles; ++rj) {
		unsigned j = yTiles - rj;
		vector<double> horizLine;
		for (unsigned i = 0; i < xTiles - 1; ++i) {
			double xUsage = 0., xCap = 0., yUsage = 0., yCap = 0.;
			for (unsigned k = 0; k < tiles.size(); ++k) {
				unsigned xEdgeId = tiles[k][j][i].incX;
				unsigned yEdgeId = tiles[k][j][i].incY;

				if (xEdgeId != UINT_MAX) {
					xUsage += edges[xEdgeId].usage;
					xCap += edges[xEdgeId].capacity;
				}
				if (yEdgeId != UINT_MAX) {
					yUsage += edges[yEdgeId].usage;
					yCap += edges[yEdgeId].capacity;
				}
			}

			double xRatio = static_cast<double>(xUsage)
					/ static_cast<double>(xCap);
			double yRatio = static_cast<double>(yUsage)
					/ static_cast<double>(yCap);

			double totalRatio = 0.5 * (xRatio + yRatio);

			maxRatio = max(maxRatio, totalRatio);

			horizLine.push_back(totalRatio);
		}
		image.push_back(horizLine);
	}

	cout << "maxratio was " << maxRatio << endl;

	for (unsigned j = 0; j < yTiles - 1; ++j) {
		xpmFile << "\"";
		for (unsigned i = 0; i < xTiles - 1; ++i) {
			unsigned index = static_cast<unsigned>(floor(
					(image[j][i] / maxRatio)
							* static_cast<double>(numColors - 1)));

			xpmFile << Colors[index];
		}
		if (j == yTiles - 2) {
			xpmFile << "\"" << endl;
		} else {
			xpmFile << "\"," << endl;
		}
	}
	xpmFile << ");" << endl;
	xpmFile.close();
}
