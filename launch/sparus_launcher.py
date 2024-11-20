import subprocess
import time

def launch_robot(robot_name, robot_xacro):
    cmd = [
        'roslaunch', 'cola2_sparus2', 'sim_start.launch',
        'robot_name:={}'.format(robot_name),
        'robot_xacro:={}'.format(robot_xacro)
    ]
    subprocess.Popen(cmd)

if __name__ == "__main__":
    robots = ['robot0', 'robot1', 'robot2', 'robot3', 'robot4', 'robot5']
    robot_xacro = '$(find sparus2_description)/urdf/payload.urdf.xacro'

    for robot in robots:
        launch_robot(robot, robot_xacro)
        time.sleep(2)