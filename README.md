# Alvik-Factory
Using Arduino Alvik robots, coordinates routing deliveries and pickups with line following.

This currently works for a single Arduino Alvik robot to drive on a line of black duct tape. The start position is fixed on a blue sticker placed on the tape along with two red stickers placed at the drop off locations. The robot will remain on the duct tape line and stop when it gets to the correct position. 

The Supervisory.py allows the user to check the status of the connected Alvik(s) and send commands via ROS2. The FactoryAGV.py responds to the commands via ROS2 topics.
For example,
# Supervisory sends this:
{"command": "request_service", "destination": "D"}

# FactoryAGV responds with this:
{"response": "service_assigned", "agv_name": "Alvik1", ...}

The BaseAGV.ino runs on the Alvik containing the movement and logic to stay on the line, start/stop moving when given the command to do so or when certain conditions are met. 

Requirements:
ROS2 Jazzy
Python 3
Arduino Alvik with micro-ROS

Before flashing BaseAGV.ino code to the robot, you must replace lines 48-50 to your WiFi network name, WiFi password and your computer's IP address.
Then upload BaseAGV.ino to the Arduino Alvik's ESP32 with Arduino IDE.

To bridge the communication between the micro-ROS and ROS2, use the microros agent with Docker. In a new terminal on the ROS2 Jazzy desktop/laptop ensure that you can start and host the agent which is able to translate the commands to one device to the other. 
1) sudo apt update && sudo apt install -y docker.io
2) sudo systemctl start docker
3) sudo systemctl enable docker
4) sudo usermod -aG docker $USER
5) newgrp docker
6) docker –version
7) docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v 6

Once the agent is running on the computer, turn the Alvik on and wait to see that it connects to the agent. In the terminal, new packets from the Alvik should be arriving frequently if the WiFi and IP are correct.

In a new terminal, change the directory to one that you will keep the FactoryAGV.py and Supervisory.py and then source ROS2 using:
source /opt/ros/jazzy/setup.bash
python3 FactoryAGV.py

Start another terminal, change the directory to the project's folder and then source ROS2 again:
source /opt/ros/jazzy/setup.bash
python3 Supervisory.py

The robot must be placed on over the blue sticker (or technically anything blue) before it can be considered in the IDLE position and thus will not accept commands. This way the start up procedure is relatively consistently such that the robot will always be on the starting sticker. The robot must be still on the sticker for two seconds before it is considered IDLE and ready to be in service. Once ready, the robot will wait until given an order from the user. In the Supervisory.py terminal, sending the command 1 D will request a delivery to the drop off location D if there is a robot available and in service. When the conditions are met, FactoryAGV.py receives the command from the supervisor and commands the robot to remain in place until it is loaded with a part. Once the part is loaded, the user sends the number 2 in the same Supervisory terminal which will in turn command the robot to begin moving along the track. The robot will continue to move along the tape until it encounters a red sticker, if the destination is D, it stops at the first red sticker but if the destination is E it will instead stop at the second red sticker. The robot will remain in place at the destination until the user sends the number 3 in the terminal which means that the robot has delivered the order to the destination and can now return to the starting location. 
