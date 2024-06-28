export LD_LIBRARY_PATH="$(pwd)/physics/xdynSurface/"
cd physics/xdynSurface
clear;./xdyn-for-cs ../../assets/models/dtmb_hull/dtmb-wave-propeller-PID.yml -v -a 127.0.0.1 -p 12346 -d --dt 0.2