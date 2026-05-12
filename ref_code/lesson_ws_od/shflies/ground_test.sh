roslaunch mavros swarm0_px4.launch & sleep 5;
roslaunch px4ctrl test_ground_run.launch;
wait;
