all: SimpleGR.exe mapper.exe

CXX = g++

OFLAGS = -pedantic -Wall -fomit-frame-pointer -funroll-all-loops -O3 -DNDEBUG
#OFLAGS = -g

LINKFLAGS = -static

SimpleGR.exe: SimpleGR.o IO.o Utils.o router.o
	/bin/rm -f SimpleGR.exe
	$(CXX) $(LINKFLAGS) SimpleGR.o IO.o Utils.o router.o -o SimpleGR.exe

mapper.exe: IO.o Utils.o mapper.o
	/bin/rm -f mapper.exe
	$(CXX) $(LINKFLAGS) IO.o Utils.o mapper.o -o mapper.exe

SimpleGR.o: SimpleGR.h SimpleGR.cpp
	/bin/rm -f SimpleGR.o
	$(CXX) $(OFLAGS) SimpleGR.cpp -c

Utils.o: SimpleGR.h Utils.cpp
	/bin/rm -f Utils.o
	$(CXX) $(OFLAGS) Utils.cpp -c

IO.o: SimpleGR.h IO.cpp
	/bin/rm -f IO.o
	$(CXX) $(OFLAGS) IO.cpp -c

router.o: SimpleGR.h router.cpp
	/bin/rm -f router.o
	$(CXX) $(OFLAGS) -DCOMPILETIME="\"`date`\"" router.cpp -c

mapper.o: SimpleGR.h mapper.cpp
	/bin/rm -f mapper.o
	$(CXX) $(OFLAGS) mapper.cpp -c

clean:
	/bin/rm -f SimpleGR.o IO.o Utils.o router.o SimpleGR.exe mapper.o mapper.exe
