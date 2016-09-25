
PNAV_DIR=/home/pi/NGCP/MAQSS/PNav/
RESOURCES_DIR=/home/pi/NGCP/MAQSS/Resources/
CV_DIR=/home/pi/NGCP/MAQSS/CV/
MAVLINK_DIR=/home/pi/NGCP/MAQSS/Resources/mavlink/

CXX=g++
CXXFLAGS=-std=c++11 -Wall -I$(PNAV_DIR) -I$(RESOURCES_DIR) -I$(CV_DIR) -I$(MAVLINK_DIR)
SHARED_SRC=$(RESOURCES_DIR)processInterface.cpp $(PNAV_DIR)serial_port.cpp $(PNAV_DIR)autopilot_interface.cpp $(RESOURCES_DIR)waypoints.cpp $(RESOURCES_DIR)fileIO.cpp
LIBS=-I/usr/local/include -lraspicam -L/opt/vc/lib -lmmal -lmmal_core -lmmal_util -I. -lpthread -fms-extensions

all: PNav CV

PNav: $(PNAV_DIR)PNav.cpp $(SHARED_SRC)
	$(CXX) $(CXXFLAGS) -o ./build/PNav $(PNAV_DIR)PNav.cpp $(SHARED_SRC) $(LIBS)
CV: $(CV_DIR)CV.cpp $(SHARED_SRC)
	$(CXX) $(CXXFLAGS) -o ./build/CV $(CV_DIR)CV.cpp $(SHARED_SRC) $(LIBS)
	
.PHONY: clean
	
clean:
	rm build/CV build/PNav
