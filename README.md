# SpiritHighLevel
Now it has a foxglove visualization
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



Task 1. get the map data from the the above messages and publish to the foxglove studio and show. 
Task 2. get the robot poses, and visualize the robot trajectory
Task 3. get the robot force data and visualize. 

