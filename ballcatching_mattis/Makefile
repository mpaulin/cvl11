CC := g++
LIBS := -lopencv_core -lopencv_legacy
FLAGS := -Wall -O2
FGBG_OBJECTS := FGBGTest.o config.o pixelDisjointSet.o blobDetection.o blobTracking.o trajectoryFiltering.o motionFiltering.o ballDetection.o

all: blobtrack_test FGBGTest

blobtrack_test: blobtrack_test.o
	$(CC) $(FLAGS) $^ -o $@ $(LIBS)

FGBGTest: $(FGBG_OBJECTS)
	$(CC) $(FLAGS) $^ -o $@ $(LIBS)

%.o: %.cpp %.hpp
	$(CC) $(FLAGS) -c $< -o $@ 

clean:
	rm *.o