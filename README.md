# MAQSS
Multiple Autonomous Quadcopter Search System

# Setup
1. Clone the MAQSS repository recursively (--recursive). There are dependencies like Eigen that are necessary.
2. Go into Resources directory and clone xbeeplus recursively.
3. In MAQSS, mkdir build.
4. In xbeeplus, mkdir build.
5. On the pi, cd into xbeeplus build and run "cmake ..."
6. Run make in MAQSS. Make sure you're doing this on the pi and not locally.
7. Go into MAQSS build directory and run ./offboard -f ../InputFile.
