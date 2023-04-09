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

Final Cost:  664.3138513011415

Goal Node Reached!

Shortest Path:  [(20, 20, 30), (35.72076941345914, 29.070827362972288, 30.0), (42.26259834755264, 40.18903795081577, 95.02786624203819), (47.625138385156, 51.92161212802336, 30.0), (54.1669673192495, 63.03982271586683, 95.02786624203819), (59.52950735685286, 74.77239689307446, 30.0), (66.07133629094633, 85.89060748091796, 95.02786624203819), (71.43387632854966, 97.62318165812557, 30.0), (79.2942610352793, 102.15859533961168, 30.0), (95.01503044873844, 111.22942270258405, 30.0), (99.31490441136336, 111.26256822086647, 325.1547309848169), (103.59534663003146, 110.85289868027837, 30.182597226855364), (107.89509320054447, 110.89974042665413, 325.3373282116723), (120.74026947475298, 109.71164154058074, 30.36519445371073), (125.03984502718045, 109.77217903978534, 325.51992543852765), (137.88874059020918, 108.62500194405989, 30.547791680566093), (150.78682332051963, 108.84769985748848, 325.702522665383), (158.26912261001243, 103.71244695792697, 325.702522665383), (171.12160709474105, 102.60620329200711, 30.730388907421457), (184.01891503189748, 102.86998436171882, 325.8851198922383), (199.0161523547165, 92.64719742844855, 325.8851198922383), (214.0133896775355, 82.42441049517828, 325.8851198922383), (229.01062700035453, 72.20162356190801, 325.8851198922383), (244.00786432317355, 61.97883662863774, 325.8851198922383), (259.0051016459926, 51.756049695367466, 325.8851198922383), (271.86104464888683, 50.69075068339304, 30.912986134276764), (276.15984541078075, 50.792371199935175, 326.0677171190937), (291.18956931286533, 40.61740685468509, 326.0677171190937), (304.04884039530003, 39.59306330536447, 31.095583361132128), (316.9442061803752, 39.93900224172742, 326.2503143459491), (329.80667486995833, 38.955624548227696, 31.278180587987492), (342.70087331581135, 39.34263736137087, 326.4329115728044), (355.56640910770693, 38.40023550121251, 31.460777814842857), (368.4593093875668, 38.82831826443681, 326.6155087996598), (381.32778174582006, 37.92690179939236, 31.64337504169822), (394.2192530460864, 38.396050169295954, 326.7981060265152), (407.0905314049469, 37.535628245293026, 31.825972268553585), (419.9804429265188, 38.045837461815246, 326.9807032533705), (427.575374350322, 33.078681165932196, 326.9807032533705), (442.7652371979279, 23.144368574166137, 326.9807032533705), (457.95510004553375, 13.210055982400082, 326.9807032533705), (470.82905381078155, 12.390637329428579, 32.00856949540889), (478.5257329317975, 17.198626046981484, 32.00856949540889), (491.41395389139956, 17.749890933451205, 327.1633004802258), (504.2904524416695, 16.97148386547474, 32.191166722264256), (517.1768520731787, 17.56379882866294, 327.3458977070812), (530.0557647612852, 16.826411243534505, 32.37376394911962), (537.7216580642385, 21.683334638826572, 32.37376394911962), (550.6061056200117, 22.316693669003023, 327.5284949339366), (563.4873017742758, 21.620333048384794, 32.556361175974985), (578.7880690671005, 31.38296692715053, 32.55636117597499), (578.7880690671005, 31.38296692715053, 32.55636117597499)]     

Runtime: 187.375009059906 seconds

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
	  
	  Once this is done, Start the ROS Master by running
	  
	  `$roscore`
	  
	  run the launch file in another terminal using the command 
	  
	  `$roslaunch proj3p2part2 astar_turtlebot.launch`
	  
	  and run the ROS executable file in another terminal using the command
	  
	  `$rosrun proj3p2part2 algorithm.py`
	   
4. Type in the Clearance Value, Start Node(x y 0) and Goal Node(x y) and the Wheel RPM1 and RPM2 when asked by the program in the 
	    terminal
5. The optimal path calculated using A* is displayed. The final cost of the path will be displayed in the console. Close the path display window to start the Gazebo Simulation.

### **Example Output**

hamza@Ubuntu20:~$ rosrun proj3p2part2 algorithm.py 

Enter the clearance (in m): 0.09

Enter the start node (in the format 'x y o')(in m): 0 0 270

Enter the goal node (in the format 'x y')(in m): 5 0

Enter RPM1: 5

Enter RPM2: 10

Final Cost:  694.5847247472822

Goal Node Reached!

Shortest Path:  [(0.0, 0.0, 270.0), (0.042225922573384755, -0.07491980613137839, 5.844375000000004), (0.12097189669458697, -0.10948969834468603, 5.2507621821795825), (0.19935567550999123, -0.14487315643677434, 5.240406936357604), (0.4344878329557108, -0.25106600707349247, 5.2402262948471545), (0.6696196557663294, -0.35725959865597323, 5.240223143656361), (0.9047514727394046, -0.463453203163832, 5.240223088685588), (0.9831287450298185, -0.4988510714082768, 5.240223087726654), (1.163870939052338, -0.4822827492262963, 0.09141278053034275), (1.388123938606409, -0.3547106316518319, 1.1359696451714738), (1.460266981842885, -0.2286312476895962, 2.288566359365769), (1.529860610115319, -0.10112690795327506, 2.3086727687133806), (1.599409512580292, 0.026401833566566557, 2.3090235138542226), (1.6689576347524557, 0.15393100062274523, 2.309029632408346), (1.7385057433128224, 0.2814601751021264, 2.3090297391431234), (1.808053851635738, 0.40898934971100154, 2.309029741005052), (1.8811042884058047, 0.45437260186021744, 1.1746547410375325), (2.244028081173231, 0.46181038669889696, 0.020491199371432515), (2.3188619498391945, 0.41943235007115476, 5.149167764768618), (2.404367565207752, 0.30200399111063225, 4.104259678187218), (2.4877186292341578, 0.18303664318111434, 4.086031837121299), (2.5710318604572695, 0.06404279774833, 4.085713862560482), (2.6543444316333797, -0.0549515098119161, 4.085708315670921), (2.7376569912953137, -0.17394582543368675, 4.085708218908514), (2.82096955075639, -0.292940141196086, 4.085708217220548), (2.904282110213963, -0.41193445696093856, 4.085708217191102), (3.1372276614023167, -0.5228417298197761, 5.220083217190583), (3.2155924925628265, -0.5582671320775014, 5.239871758857241), (3.2939695479069524, -0.5936654806762769, 5.240216958972981), (3.372346816412292, -0.6290633573015588, 5.240222980797222), (3.7338312045191397, -0.595926713611884, 0.09141277866501822), (3.8085822043718807, -0.5534026744228284, 1.1359696451389343), (3.8825459767373163, -0.5095236787026569, 1.1541913593652013), (4.104395439480484, -0.37781616611511093, 1.1545092270467041), (4.285858631579115, -0.37416104458833255, 0.020139772071814728), (4.510270070477471, -0.2468678433441176, 1.1347263271350305), (4.732164242595128, -0.11523566881353176, 1.1541696703733555), (4.954013755170135, 0.016471759836724997, 1.1545088486942907), (4.954013755170135, 0.016471759836724997, 1.1545088486942907)] 

Actions:  [[0, 5], [5, 0], [5, 0], [10, 5], [10, 5], [10, 5], [5, 0], [5, 5], [5, 10], [0, 10], [0, 10], [0, 10], [0, 10], [0, 10], [0, 10], [0, 5], [10, 10], [5, 0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 5], [5, 0], [5, 0], [5, 0], [10, 10], [0, 5], [0, 5], [5, 10], [5, 5], [5, 10], [5, 10], [5, 10]] 

Runtime: 16.92942976951599 seconds


Sending data for Gazebo Simulation...

Simulation Start

0.09 1.03125
0.09 -1.03125
0.09 -1.03125
0.12 -1.03125
0.12 -1.03125
0.12 -1.03125
0.09 -1.03125
0.11 0.0
0.12 1.03125
0.11 2.0625
0.11 2.0625
0.11 2.0625
0.11 2.0625
0.11 2.0625
0.11 2.0625
0.09 1.03125
0.16 0.0
0.09 -1.03125
0.11 -2.0625
0.11 -2.0625
0.11 -2.0625
0.11 -2.0625
0.11 -2.0625
0.11 -2.0625
0.11 -2.0625
0.12 -1.03125
0.09 -1.03125
0.09 -1.03125
0.09 -1.03125
0.16 0.0
0.09 1.03125
0.09 1.03125
0.12 1.03125
0.11 0.0
0.12 1.03125
0.12 1.03125
0.12 1.03125

Simulation End

### **Animation**

https://drive.google.com/file/d/1rxclzxc4yldDlTF_SU80OIRsapl0sdM5/view?usp=share_link

### **Gazebo Simulation**

https://drive.google.com/file/d/1jdrvGES6-6qGE_QHW5TyElNrYU74TZu-/view?usp=share_link



