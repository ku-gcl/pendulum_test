ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
g++ -o cleanup cleanup.cpp -lpigpiod_if2 -lrt
sudo ./cleanup
