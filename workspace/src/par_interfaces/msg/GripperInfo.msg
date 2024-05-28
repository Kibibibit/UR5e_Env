# GripperInfo.msg
## Contains data about the gripper detected by the system

### The type of gripper, will probably be RG2 or RG6
string gripper_type

### The ip address of the gripper
string ip
### The port of the gripper
string port

### The maximum width the gripper can open in metres
int max_width
### The maximum force the gripper can exert, in newtons
int max_force