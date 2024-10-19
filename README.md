# SpiritHighLevel
Now it has a foxglove visualization

Potential features: 
Task 1. get the map data from the the above messages and publish to the foxglove studio and show. 
Task 2. get the robot poses, and visualize the robot trajectory
Task 3. get the robot force data and visualize. 

the topics that the visualization should subscribe is:

We have a implemented a fake generation node for testing visualization. 


Message structures
https://docs.google.com/document/d/14VMS8K1z5dEPHZ4L8xZxGQBFie7c2LfstKQuhG2EZtA/edit

# Estimated map from given data (image-like)
# often produced with Gaussian Process Regression

#
std_msgs/Header header

#
nav_msgs/MapMetaData meta

#
float32[][] data # predicted values

#
float32[][] uncertainty


Tasks for your first step:
1.  run visualizer.py successfully
2. understand visualizerzation
3. download the foxglove studio
4. load the lassie-spirit-current-visualization.json
5. look at the foxglove studio doc, and configure the topic send to the right 3d panel'
6. implement a fake data generation (in order to publish fake data in the message format shown above, you should build a custom message)
7. implement a data tranform, which tranforms map data into 2d occupany grid data format. 

