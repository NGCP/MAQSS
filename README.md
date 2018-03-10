# MAQSS
Multiple Autonomous Quadcopter Search System

# Setup
1. Clone the multithreadedv1 branch of the MAQSS repository recursively (git clone -b --recursive multithreadedv1 https://github.com/NGCP/MAQSS.git).
2. Go into Resources directory and clone xbeeplus recursively.
3. In both the MAQSS and xbeeplus directory, mkdir build.
5. On the pi, cd into xbeeplus build and run "cmake ..."
6. Run make in MAQSS. Make sure you're doing this on the pi and not locally.
7. Go into MAQSS build directory and run ./offboard -f ../InputFile.
