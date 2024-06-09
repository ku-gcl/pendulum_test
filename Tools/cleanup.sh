ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
g++ -o /home/ubuntu/pendulum_project/pendulum_test/cleanup /home/ubuntu/pendulum_project/pendulum_test/cleanup.cpp -lpigpiod_if2 -lrt
sudo /home/ubuntu/pendulum_project/pendulum_test/cleanup
