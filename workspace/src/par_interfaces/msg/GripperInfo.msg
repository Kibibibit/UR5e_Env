# GripperInfo.msg
## Contains data about the gripper detected by the system

### The ip address of the gripper
string ip
### The port of the gripper
string port

### The maximum width the gripper can open in milimetres
float64 max_width
### The maximum force the gripper can exert, in newtons
float64 max_force
### The offset in milimetres caused by the rubber fingertips on the gripper
float64 fingertip_offset