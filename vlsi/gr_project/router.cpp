#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <limits>

using namespace std;

#include "SimpleGR.h"

int main(int argc, char **argv) {
	cout << "SimpleGR " << SimpleGRversion << " (" << sizeof(void*) * 8
			<< "-bit) Compiled on " << COMPILETIME;
#ifdef __GNUC__
	cout << " with GCC " << __GNUC__ << "." << __GNUC_MINOR__ << "."
			<< __GNUC_PATCHLEVEL__;
#endif
	cout << endl;

	// parameters
	string outputfile;

	for (int i = 2; i < argc; ++i) {
		if (argv[i] == string("-o")) {
			if (i + 1 < argc) {
				outputfile = argv[++i];
			} else {
				cout << "option -o requires an argument" << endl;
				SimpleGRParams::usage(argv[0]);
				return 0;
			}
		}
	}

	SimpleGRParams parms(argc, argv);

	SimpleGR simplegr(parms);

	// read in the nets and create the grid
	simplegr.parseInput(argv[1]);

	cout << "CPU time: " << cpuTime() << " seconds" << endl;

	simplegr.printParams();

	simplegr.initialRouting();

	double rrrStartCpu = cpuTime();

	simplegr.doRRR();

	cout << endl << "after all rip-up iterations" << endl;
	simplegr.printStatistics();

	double rrrEndCpu = cpuTime();

	cout << "RRR CPU time: " << rrrEndCpu - rrrStartCpu << " seconds " << endl;

	simplegr.printStatistics();

	cout << endl << "greedy improvement phase" << endl;
	double greedyStartCpu = cpuTime();
	simplegr.greedyImprovement();

	cout << endl << "after greedy improvement phase" << endl;
	simplegr.printStatistics();

	double greedyEndCpu = cpuTime();

	cout << "greedy CPU time: " << greedyEndCpu - greedyStartCpu << " seconds "
			<< endl;

	cout << endl;
	simplegr.printStatistics(true, true);

	if (!parms.outputFile.empty()) {
		simplegr.writeRoutes(parms.outputFile);
	}

	return 0;
}
