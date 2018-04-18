# MAQSS

Multiple Autonomous Quadcopter Search System

# Setup

## Make sure all of the instructions are executed on the raspberry pi.

1. Clone the `multithreadedv1` branch of the MAQSS repository recursively 
   ```bash
   git clone -b multithreadedv1 --recursive https://github.com/NGCP/MAQSS.git)
   ```

2. Go into `MAQSS/Resources` directory and clone `xbeeplus` recursively.
   ```bash
   cd MAQSS/Resources && git clone -b master --recursive https://github.com/NGCP/xbeeplus.git
   ```

3. In both the `MAQSS` and `xbeeplus` directories, run `mkdir build`.
4. On the pi, run `cd xbeeplus/build` and then `cmake ..`.
5. Run `make` in `MAQSS`.
6. `cd` into `MAQSS/build` and run `sudo ./offboard -f ../InputFile`.

If you are getting a `Permission denied` error, run (assuming you're in
`MAQSS/build`) `chmod 777 ../Logging/create_log.sh` and redo step 6. 
