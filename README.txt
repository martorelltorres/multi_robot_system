To run the MRS_stack at this point do the following steps:

1) Open one terminal and launch the stonefish using the following command:
    roslaunch multi_robot_system stonefish.launch

2) Open another terminal and launch the MRS_simulation:
    roslaunch multi_robot_system MRS.launch

3) Open another terminal and launch the neural network:
    First, set the conda environment, using:
    conda activate yolov5_PLOME
    Then run the CNN code:
    roslaunch sea_cucumber_detection MRS_inference.launch
