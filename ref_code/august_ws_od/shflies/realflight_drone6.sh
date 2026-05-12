gnome-terminal --window -e 'bash -c "sleep 1; roslaunch mavros swarm6_px4.launch & sleep 5; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch px4ctrl drone6.launch; exec bash"' \
