# Path Planning(RRT*) and Motion control - Furbot
This is a repository for the project named Path planning and motion control(trajectory tracking).
This project is a part of Theis , Robotics Engineering, during my master's at the University of Genova, Italy.

## Software Requirement 

Matlab 2021

# Map Definition
We already provide a sample map creating in RRTFurbot.m source code. We defined all closed polygon in that maps which shows the obstacles.
In the sample map, there are 2 different obstacles whose edges are defined by given x and y coordinates.
One sample is shown by followings;

X_max=40
y_max=40
the above is the size of the map.

The Obstacle are defined by:
Obstacle=[25,25,4,4]
Obstacle1=[10,30,4,4]
<p align="center">
  <img src="outputs/Map.png" width="400"/>  
  
</p>

We have defined the parking positon or Goal position, Parking Slot has been defined and shown in the figure.


# Sampling Based Planner.

In the sampling-based method, We have to generate points on the map which fall on the empty region of the map.
Then we calculate which node has a connection to which nodes.  In this way, we obtain the undirected graph of generated random points.
Random Nodes generated, the Max number of nodes is defined,and the obstacle free path is generated  untill the max number of nodes is generated.
The Below Image is the RRT* Path planning:

<p align="center">
  <img src="outputs/RRTstar path.png" width="400"/>  
  
</p>
To smooth the Path form RRT* Algorithm , we have used Bezier Curve equation.
Fucntion mybez.m, RRT* way points are feed and output is way points of smooth curve.

<p align="center">
  <img src="outputs/Smooth Path.png" width="400"/>  
  
</p>

# Motion Control using trajecotry tracking Algorithm.

PurePursuit controller is implemented to control the vehicle in the defined path, while reducing the cross track error.

[video](https://github.com/srini0613/Path-Planning-With-RRT-/blob/main/outputs/result.avi)
![simulation](https://github.com/srini0613/Path-Planning-With-RRT-/blob/main/outputs/Motion%20control%20of%20Furbot.gif)

# Program execution.
1) Run Code/RRTFurbot.m for Path planning.
2) Run Code/Main.m for Motion control of Furbot (1 for PurePursuit controller) (2 for PID controller)
