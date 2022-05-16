# multi_robot_system
The multi_robot_system is divided into three classes:
- area_partition: handles the predefined areas to be covered. The input to the area_partition are the latitude and longitude points of the main polygon and the NED origin.
- task_allocator: manages the task allocation according to the number of robots and tasks to be performed. The input parameters of the task allocator are the number of robots and the different tasks or sub polygons to cover.
- robot: manages the information and orders of the robots.

In order to simulate the robots the multi_robot_system package has been created to work with the cola2_architecture (https://iquarobotics.com/cola2). The hole process, the robot architecture simulation and the multi_robot_system strategies are handled from the MRS.launch.

