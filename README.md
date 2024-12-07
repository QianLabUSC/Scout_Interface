# SpiritHighLevel

BUILDING:
    cd into SpiritHighLevel Folder
        
    type: "colcon build"

    then source: "source install/setup.bash"
RUNNING:
    FIRST: OPEN asme.yaml
        in the SpiritHighLevel/src/spirit_high_launch/config
        find the line called csv_file
            change this to a new test name
    then run "ros2 launch spirit_high_launch launch_asme_demo.launch.py"
        and if doing fake data: "ros2 launch spirit_high_launch launch_fake_testing.launch.py"
RUNNING FOXGLOVE: 
    after doing ros2 launch previously
    open foxglove
    connect to ws://localhost:8765
    it should appear
MODIFYING DEMO VISUALIZATION:
    in the SpiritHighLevel/src/spirit_high_launch/config
    can change parameters for mapping and visualization
Now it has a foxglove visualization

Potential features: 
Task 1. get the map data from the the above messages and publish to the foxglove studio and show. 
Task 2. get the robot poses, and visualize the robot trajectory
Task 3. get the robot force data and visualize. 

the topics that the visualization should subscribe is:

We have a implemented a fake generation node for testing visualization. 


Message structures
https://docs.google.com/document/d/14VMS8K1z5dEPHZ4L8xZxGQBFie7c2LfstKQuhG2EZtA/edit



