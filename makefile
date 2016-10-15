SHELL := /bin/bash
PNAV_DIR=$(CURDIR)/PNav/
RESOURCES_DIR=$(CURDIR)/Resources/
CV_DIR=$(CURDIR)/CV/
MAVLINK_DIR=$(CURDIR)/Resources/
BUILD_DIR=$(CURDIR)/build/



CXX=g++
CXXFLAGS=-std=c++11 -Wall -O3 
LINK_FLAGS=-I$(PNAV_DIR) -I$(RESOURCES_DIR) -I$(CV_DIR) -I$(MAVLINK_DIR)
SHARED_SRC=$(RESOURCES_DIR)processInterface.cpp $(PNAV_DIR)serial_port.cpp $(PNAV_DIR)autopilot_interface.cpp $(RESOURCES_DIR)waypoints.cpp $(RESOURCES_DIR)fileIO.cpp $(PNAV_DIR)flight_logger.cpp
PNAV_LIBS=-I/usr/local/include -lpthread -fms-extensions
CV_LIBS=-I/usr/local/include -lraspicam -lraspicam_cv -L/opt/vc/lib -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lpthread
CV_FLAGS=$(CXXFLAGS)

SHARED_O_TARGETS= processInterface.o serial_port.o autopilot_interface.o waypoints.o fileIO.o flight_logger.o
SHARED_O=$(RESOURCES_DIR)processInterface.o $(PNAV_DIR)serial_port.o $(PNAV_DIR)autopilot_interface.o $(RESOURCES_DIR)waypoints.o $(RESOURCES_DIR)fileIO.o $(PNAV_DIR)flight_logger.o

ifdef DEBUG
	CV_FLAGS+=-DDEBUG
endif

ifdef TEST
	CV_FLAGS+=-DTEST
endif

all : PNav CV
	
# Link Commands
PNav : $(PNAV_DIR)PNav.o  $(SHARED_O)
	$(CXX) $(PNAV_DIR)PNav.o $(SHARED_O) -o $(BUILD_DIR)/PNav $(PNAV_LIBS)
	
CV : $(CV_DIR)CV.o  $(SHARED_O)
	$(CXX) $(CV_DIR)CV.o $(SHARED_O) -o $(BUILD_DIR)/CV $(CV_LIBS)
	
# Compilation Commands
$(PNAV_DIR)PNav.o : 
	$(CXX) $(CXXFLAGS) -c $(PNAV_DIR)PNav.cpp -o $(PNAV_DIR)PNav.o $(LINK_FLAGS)

$(CV_DIR)CV.o : 
	$(CXX) $(CV_FLAGS) -c $(CV_DIR)CV.cpp -o $(CV_DIR)/CV.o $(LINK_FLAGS)

$(RESOURCES_DIR)processInterface.o : 
	$(CXX) $(CXXFLAGS) -c $(RESOURCES_DIR)processInterface.cpp -o $(RESOURCES_DIR)processInterface.o

$(PNAV_DIR)serial_port.o : 
	$(CXX) $(CXXFLAGS) -c $(PNAV_DIR)serial_port.cpp -o $(PNAV_DIR)serial_port.o -I$(MAVLINK_DIR)

$(PNAV_DIR)autopilot_interface.o : 
	$(CXX) $(CXXFLAGS) -c $(PNAV_DIR)autopilot_interface.cpp -o $(PNAV_DIR)autopilot_interface.o -I$(MAVLINK_DIR)

$(RESOURCES_DIR)waypoints.o : 
	$(CXX) $(CXXFLAGS) -c $(RESOURCES_DIR)waypoints.cpp -o $(RESOURCES_DIR)waypoints.o

$(RESOURCES_DIR)fileIO.o : 
	$(CXX) $(CXXFLAGS) -c $(RESOURCES_DIR)fileIO.cpp -o $(RESOURCES_DIR)fileIO.o
	
$(PNAV_DIR)flight_logger.o : 
	$(CXX) $(CXXFLAGS) -c $(PNAV_DIR)flight_logger.cpp -o $(PNAV_DIR)flight_logger.o -I$(MAVLINK_DIR) -I$(PNAV_DIR)

run : runPNav runCV proc

runPNav:
	cd ./build && ds ./PNav -f ../InputFile &>Plog
runCV:
	cd ./build && ds ./CV -f ../InputFile &>Clog
proc:
	ps
	
.PHONY: clean run runPNav runCV proc
	
clean:
	rm $(PNAV_DIR)*.o $(RESOURCES_DIR)*.o $(CV_DIR)*.o $(BUILD_DIR)CV $(BUILD_DIR)PNav
