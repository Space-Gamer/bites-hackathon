# BOSCH BITES Hackathon


## Team Members

- [Archana A Chandaragi](https://github.com/Itryok)
- [Arjun Achar](https://github.com/achararjun)
- [Pradeep J](https://github.com/Space-Gamer)
- [Sanraj Lachhiramka](https://github.com/Sanraj-Lachhiramka)


## Problem Statement

Using ROS, Build a two-PC communication model that will acheive the following 

We define 2 PCs(Linux Machines) as follows: PC1 and PC2 
 - PC1 will send out data to the PC2 (either as ROS topic or Request/Response model) 
 - Data sent out by PC1 contains following data:
 - An array of structures called location_list of size _NUMBER_OF_LOCATIONS_ with structure as shown below
	```
    Single_location:
		Radius
		Azimuth angle
		Elevation angle
    ```

 - The data from PC1 must be sent cyclically( period can be defined by the teams, suggest to keep the period within 1 second)
 - Verify the published data using Wireshark tool.
 - PC1 must process based data as per the [Algorithm](#algorithm-development) mentioned in below section
 - PC2 must be capable of receiving the data published by PC1 (Teams are allowed to use publisher-subscriber or server-client methods)
 - PC2 ROS Node should convert data into x, y, z and use specific ROS message type to visualize the same in RVIZ 
	* Hint : Teams are free to look up open source and understand.
        * Double-Hint ** Pcl-classes
 - Once conversion is done, another topic of the type as described above must be published and using RVIZ, it mist be visualized.
 
 ## __Algorithm development__:

Teams must come up with a way to achive the following in well-defined steps:

Step1:
- Each location must have different azimuth and elevation angle and initially all locations must be at origin.
- For every cycle, change the elevation and azimuth angles in a specific pattern(can be decided by teams).
- Send out this data for all locations from PC1 and receive this at PC2. Convert this data into x, y, z and visualize it in RVIZ.

## Our Approach



## Tech Stack
- Python
- C++
- ROS
- Rviz

## Steps to run the project

1. Clone the repository in the `src` directory of your catkin workspace using the following command
```bash
git clone https://github.com/Space-Gamer/bites-hackathon.git
```
2. Navigate to the root of your catkin workspace and build the project using the following commands
```bash
catkin_make
source devel/setup.bash
```
3. Running the project
- To run the basic implementation of the project, run the following commands
    ```bash
    roslaunch bites_hackathon bites_hackathon.launch ros_service:=false ## For using Publisher-Subscriber model
    ```
    or
    ```bash
    roslaunch bites_hackathon bites_hackathon.launch ros_service:=true ## For using ROS Service model
    ```
- To run the improvised implementation of the project with moving obstacle and varying frame rate, run the following commands
    ```bash
    roslaunch bites_hackathon bites_hackathon_moving_obj.launch python:=true ## For running the python implementation
    ```
    or
    ```bash
    roslaunch bites_hackathon bites_hackathon_moving_obj.launch python:=false ## For running the C++ implementation
    ```

4. To visualize the data in RVIZ, navigate to the rviz tab and add a new display of type `Markers`.
