export LD_LIBRARY_PATH="$(pwd)/physics/xdynSurface/"
cd physics/xdynSurface
clear;./xdyn-for-cs ../../assets/models/test_ship/test_ship_in_waves.yml -v -a 127.0.0.1 -p 12346 -d --dt 0.2