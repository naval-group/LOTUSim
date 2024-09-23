export LD_LIBRARY_PATH="$(pwd)/physics/xdynSurface/"
cd physics/xdynSurface
clear;./xdyn-for-cs ../../assets/models/test_ship/testship-nowave-static.yml -v -a 127.0.0.1 -p 12345 -d --dt 0.2
