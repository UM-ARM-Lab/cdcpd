## Example roslaunch usage

This will set the IP address in use for recieving data from the robot to `10.10.10.99`. Requires changes in the Java code as well to use this, as the robot uses hard coded values for `realtime.local`'s IP address.

`roslaunch victor_hardware_interface dual_arm_lcm_bridge.launch left_arm_recv_url_override:=true left_arm_recv_url:=udp://10.10.10.99:30002 right_arm_recv_url_override:=true right_arm_recv_url:=udp://10.10.10.99:3001`
