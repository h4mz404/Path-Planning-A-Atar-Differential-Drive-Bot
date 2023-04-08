# Implementation of the A* algorithm on a Differential Drive (non-holonomic) TurtleBot3 robot



### ENPM661 - Planning For Autonomous Robots 

**PROJECT 3 PHASE 2**

Hamza Shah Khan: 119483152 | hamzask@umd.edu

Vishnu Mandala: 119452608 | vishnum@umd.edu

## Part 01: 2D Implementation

Part01 of this project implements a path planning algorithm using A* algorithm on a differential drive TurtleBot3 robot, to find the optimal path from a start node to a goal node in a 2D environment with obstacles. The obstacles are defined using half-plane equations, and the map parameters include width, height, radius and clearance.

The program first creates a 2D map of the environment with the obstacles marked as black and the free space marked as white. The boundaries and clearances are marked as gray. The start and goal nodes are also defined on this map.

The program then uses A* algorithm to find the optimal path from the start node to the goal node considering the differential drive constraints of the Turtlebot3 robot while also avoiding the obstacles. The algorithm is visualized using animation, with each visited node marked in blue and the optimal path marked in red. The final cost of the path is also displayed.

### **Dependencies**

* python 3.11 (any version above 3 should work)
	
* Python running IDE (We used VS Code)

### **Libraries**
* NumPy
* Queue
* Time
* OpenCV
* Math

### **Contents**

* Part 01
  * proj3p2_hamza_vishnu.py
* Part 02
  * proj3p2part2
    * src	
      * proj3p2part2.py		
    * launch	
      * astar_turtlebot.launch			
    * world	
      * map.world			
* proj3p2_hamza_vishnu.pdf
* README.md

### **Instructions**
1. Download the zip file and extract it
	
2. Install python and the required dependencies: 

	`pip install numpy opencv-python`
	
3. Run the code:

	`$python3 Part01/proj3p2_hamza_vishnu.py`
	
4. Type in the Clearance Value, Start Node(x y 0) and Goal Node(x y) and the Wheel RPM1 and RPM2
	
5. The optimal path will be displayed on the screen and saved to a video file named animation.mp4 in the same directory as the program. The final cost of the path will be displayed in the console.

### **Example Output**

Enter the clearance: 3

Generating Map...

Enter the start node (in the format 'x y o'): 20 20 30

Enter the goal node (in the format 'x y'): 580 30

Enter RPM1: 5

Enter RPM2: 10

Final Cost:  670.2745208843769

Goal Node Reached!

Shortest Path:  [(20, 20, 30), (26.54182893409348, 31.118210587843492, 1.657708333333333), (30.091457280357737, 37.45474852055482, 2.297667800925926), (33.5701264595284, 43.83051789461317, 2.308831538305041), (37.04755392228185, 50.206964601877374, 2.3090262835015434), (40.524959722811715, 56.58342312269703, 2.3090296807233046), (44.00236514545578, 62.95988184959693, 2.309029739985951), (47.65488698399943, 65.2290444569928, 1.174654741019755), (51.25778415417394, 71.5354464700618, 2.2892411993711224), (54.73739051716784, 77.91070441997292, 2.308684540922363), (58.38992602010202, 80.17984503255012, 1.174648719213868), (69.48004834590228, 86.76917878319325, 1.154866094324064), (73.08512228362427, 93.07433669341641, 2.2888959974232086), (76.73844087659373, 95.34221631502459, 1.1743035190661604), (87.82860288200627, 101.93148328277462, 1.1548600724992653), (91.43367748206859, 108.23664081429416, 2.2888958923758205), (100.5014443681751, 108.59889583390247, 0.03992851723366709), (118.65143996539231, 108.61153786691169, 0.0006965308006317481), (136.8014399640525, 108.61175840017195, 1.2150592855464938e-05), (154.9514399640522, 108.61176224725214, 2.1196034203422172e-07), (158.69237574861688, 106.4915230683845, 5.1488103108771135), (170.44048459219454, 101.16310200037294, 5.2386284426026615), (182.19692772803631, 95.85309472426717, 5.240195270011651), (193.95351599490846, 90.54340878234726, 5.240222602445342), (205.71010679343283, 85.23372844595889, 5.240223079244463), (217.46669763612041, 79.92404820735592, 5.240223087561958), (221.75129922342876, 74.05942856007044, 4.105848087707058), (233.40052431022775, 68.51815728393971, 5.220434546042917), (245.15528155102754, 63.20441898699257, 5.239877887593887), (256.9118404203249, 57.89466795409822, 5.240217065885387), (260.83070384889265, 56.124774130142896, 5.24022298266225), (272.58729468263437, 50.815093871732124, 5.240223085877135), (276.87189626977033, 44.95047422432072, 4.105848087677667), (288.5211213565664, 39.409202948184024, 5.220434546042404), (297.55851648857885, 40.23449939666915, 0.09106758041429527), (315.7084935857872, 40.26333289829338, 0.0015886233472271508), (333.8584935788176, 40.2638358829221, 2.771265172385141e-05), (352.0084935788155, 40.26384465720954, 4.834318134049635e-07), (370.15849357881524, 40.263844810272104, 8.433199411619918e-09), (388.308493578815, 40.263844812942196, 1.4711247862492523e-10), (399.53130090900635, 33.903127234871356, 5.148810307182149), (411.27940975224055, 28.57470616610255, 5.238628442538205), (423.03585288807636, 23.264698889983542, 5.240195270010527), (434.7924411549484, 17.955012948063384, 5.240222602445322), (446.54903195347276, 12.645332611675002, 5.240223079244462), (464.6232513559701, 14.30216482719866, 0.09141278038237562), (473.69823981759475, 14.316636225973147, 0.0015946451688925525), (491.848239810572, 14.317141117210975, 2.781769905734786e-05), (509.9982398105699, 14.31714992475812, 4.852643057781794e-07), (528.1482398105696, 14.317150078400884, 8.465166223019353e-09), (539.3710471398053, 20.677867658157794, 1.1343750001476702), (557.517493609356, 21.037006249420145, 0.019788541669242692), (568.7381045504578, 27.401597562851187, 1.1347202001157857), (577.8113267037343, 27.581221495811363, 0.019794563490908707), (577.8113267037343, 27.581221495811363, 0.019794563490908707)]

Runtime: 11.02682089805603 seconds

### **Animation**

https://drive.google.com/file/d/1d204RiwlYIIe02Jf-8912CeFpOY6TO14/view?usp=share_link


### **Link to Github Repo**

https://github.com/h4mz404/Path-Planning-A-Star-Differential-Drive-Bot

## Part 02: Gazebo Visualization

This part of the project is to simulate the path planning implementation on Gazebo with the TurtleBot3 Burger robot.

The output video shows the TurtleBot motion in Gazebo environment. The motion of 
the TurtleBot is a result of the solution generated by the A* algorithm, implemented in Part01, and the 
corresponding velocity commands published to the appropriate ROS topic. 

### **Dependencies**

* Ubuntu 20.04
* ROS Noetic
* Gazebo
* catkin
* Turtlebot3 packages

### **Libraries**

* math
* numpy
* opencv
* time
* matplotlib
* queue
* rospy
* geometry_msgs

### **Instructions**
1. Download the zip file and extract it
	
2. Install python and the required dependencies:

	`$pip install numpy opencv-python`
	
3. Navigate to Part02/proj3p2part2, open terminal in this folder and run the command

	  `$catkin_make`
	  
	  to initialize it as a ROS workspace. Then source this ROS workspace using "source".
	  
	  `$source catkin_ws/devel/setup.bash`
	  
	  Once this is done run the Gazebo simulation using the command 
	  
	  `$roslaunch proj3p2part2 astar_turtlebot.launch`
	   
4. Type in the Clearance Value, Start Node(x y 0) and Goal Node(x y) and the Wheel RPM1 and RPM2 when asked by the program in the 
	    terminal
5. The optimal path followed by the Turtlebot3 will be simulated in Gazebo. The final cost of the path will be displayed in the console.

### **Example Output**
