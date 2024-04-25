# mechanumdrive_arduino

This node is designed to provide a ros2_control hardware interface for an Arduino running firmware from `ros_arduino_bridge`.
It is designed to be used with a `mechanumbot_controller` from `ros2-mechanum-bot`.
It is expected to communicate via serial and to have four motors, each with velocity control and position/velocity feedback.




It is based on the diffbot controller from ROS2 and diffbot hardware interface by joshnewmans

For a tutorial on how to develop a hardware interface like this, check out the video below:

https://youtu.be/J02jEKawE5U
