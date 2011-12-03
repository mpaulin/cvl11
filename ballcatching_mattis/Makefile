CC=g++
LIBS=-lopencv_core -lopencv_legacy
FLAGS=-Wall -O2 -g

all: blobtrack_test FGBGTest

blobtrack_test: blobtrack_test.cpp pixelDisjointSet.cpp config.hpp
	$(CC) $(FLAGS) $^ -o blobtrack_test $(LIBS)

FGBGTest: FGBGTest.cpp pixelDisjointSet.cpp blobDetection.cpp config.hpp
	$(CC) $(FLAGS) $^ -o FGBGTest $(LIBS)

