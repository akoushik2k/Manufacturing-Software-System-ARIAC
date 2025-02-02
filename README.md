# Manufacturing Software System ARIAC
---
## Introduction
This project contains motion planning & completing orders RWA 5, based on ARIAC (Agile Robotics for Industrial Automation Competition) v2023.5.0. 
The project is implemented in Python and C++ and focuses on retrieving kitting trays and parts for each order in the ARIAC environment.
<sup>[ARIAC 2023 Documentation](https://pages.nist.gov/ARIAC_docs/en/2023.5.0/index.html)</sup>

MoveIt Order Fulfillment System with ROS

###Task 1-Order Processing:
 - Subscribing to the /ariac/orders topic to receive incoming orders.
 - Processing each order with the required actions to fulfill it.

###Task 2-Motion Planning:
 - Utilizing MoveIt to plan and execute robot motions like pick and place objects.
 - Based on the task requirements and robot capabilities we are deciding whether to perform motion planning in Joint space or in Cartesian space. 

###Task 3-Python-C++ Communication:
 - Since we are using a combination of both Pyhton and C++ we have implemented servers in C++ to encapsulate robot motion control methods.
 - And clients in Python to send requests to the C++ servers, enabling seamless communication between Python and C++ nodes.

###Task 4-Dynamic Planning Scene Update:
 - Ensure that objects added to the planning scene have unique names to prevent potential bugs.
 - Update objects' poses in the planning scene whenever their positions change to maintain accurate planning data.

###Task 5-Simulation Considerations:
 - Setting the parameter `use_sim_time` for any node that interacts with simulation.

## File structure
```
final_group1
.
├── CMakeLists.txt
├── config
│   ├── floor_robot_targets.yaml
│   └── sensors.yaml
├── include
│   ├── floor_robot_cpp_python.hpp
│   └── utils.hpp
├── launch
│   └── final_group1.launch.py
├── meshes
├── package.xml
├── rviz
├── final_group1
│   ├── competition_interface.py
│   ├── __init__.py
│   └── kitting_interface.py
├── script
│   ├── competition_node.py
│   └── kitting_node.py
└── src
    ├── floor_robot_cpp_python.cpp
    └── main_cpp_python.cpp

```


## Requirements
- [Python 3.8.10](https://realpython.com/installing-python/)
    - threading
    - enum
- [ROS2 Galactic](https://docs.ros.org/en/galactic/Installation.html)
- [ARIAC Framework](https://pages.nist.gov/ARIAC_docs/en/2023.5.0/getting_started/installation.html)
- Files required in Workspace source folder
    - ariac
    - final_group1
    - robot_msgs

## Usage
Downlaod the official setup from ARIAC for simulation and world setup [ARIAC GITHUB](https://github.com/usnistgov/ARIAC/tree/ariac2023)
[Note]: The code is written based on the ARIAC 2023 models so only download those


Make sure you have created a copy of ```rwa5_spring2024.yaml``` and placed it in the given location:
```
<your_ariac_workspace_directory>/src/ariac/ariac_gazebo/config/trials/
```

[Note]: Ensure that you have built and sourced ariac environment properly. If not, follow these steps:

1. ```cd <your_ariac_workspace>```
2. ```colcon build```
3. ```source install/setup.bash```

1. To run the ARIAC simulation, enter:
    ```
    ros2 launch ariac_gazebo ariac.launch.py trial_name:rwa5_spring2024
    ```

2. To launch the RWA5 Group1 submission, in a new terminal, run:
    ```
    ros2 launch final_group1 final_group1.launch.py
    ```

[Note]: Ensure that the new terminal has also sourced the final_group1 package. You can source it using:

--- End of file ---
