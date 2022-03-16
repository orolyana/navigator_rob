# Navigaor Robot

## Team Members: 

1) Ibrahim Abas: GSR/7382/14
2) Diyawdin Menu: GSR/1558/14




## Running Instructions 

### Update your source first:

```
$ cd ~/catkin_ws && catkin_make
$ source devel/setup.bash  
```


### Terminals:

```
$ roscore
```

```
$ export TURTLEBOT3_MODEL=waffle
$ roslaunch navigator_robo turtlebot3_custom_world.launch
```
```
$ rosrun navigator_robo robotmovement.py
```

```
$ rosrun navigator_robo navigationui.py
```


### Project Description: 

1) Given a list of locations in a city (imitated through a gazebo simulation), our bot will find the shortest route that it can take through all points. 
2) Move between nodes along the fastest path, Use odometry to detect when goal is reached, Stay centered on the path
3) While navigating, the bot will need to follow basic traffic rules: stop at stop signs, avoid driving on the sidewalk, avoid traffic cones. 

Our robot can find the shortest route between any number of destinations on the map and navigate to the points without colliding into obstacles, stopping at stop signs, and avoiding bumping into one other moving robot. The other moving robot takes the same cyclical path around the street and our navigation bot stops and lets them pass when the situation comes up. 

