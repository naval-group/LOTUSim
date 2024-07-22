export LD_LIBRARY_PATH="$(pwd)/physics/xdynUnderwater"
cd physics/xdynUnderwater
clear;./xdyn-for-cs ../../assets/models/lrauv_xdyn/lrauv.yml -v -a 127.0.0.1 -p 12345 -d --dt 0.2