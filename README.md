# MAQSS
Multiple Autonomous Quadcopter Search System

# Setup
1. Clone the MAQSS repository recursively (-r). There are dependencies like Eigen that are necessary.
2. Go into Resources directory and clone xbeeplus recursively.
3. In MAQSS, mkdir build.
4. Run MAKE in MAQSS. Make sure you're doing this on the pi and not locally.
5. Go into build directory and run ./offboard -f ../InputFile.
