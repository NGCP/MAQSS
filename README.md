# MAQSS
Multiple Autonomous Quadcopter Search System

# Setup
## Make sure all of the instructions are executed on the raspberry pi.
1. Clone the multithreadedv1 branch of the MAQSS repository recursively (git clone -b --recursive multithreadedv1 https://github.com/NGCP/MAQSS.git).
2. Go into Resources directory and clone xbeeplus recursively.
3. In both the MAQSS and xbeeplus directory, mkdir build.
4. On the pi, cd into xbeeplus build and run "cmake ..."
5. Run make in MAQSS.
6. Go into MAQSS build directory and run ./offboard -f ../InputFile.
