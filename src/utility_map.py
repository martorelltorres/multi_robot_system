import rospy
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from multi_robot_system.msg import BufferedData  
from math import sqrt
from geometry_msgs.msg import PoseWithCovarianceStamped
import pickle

class UtilityGridMap:
    def __init__(self, resolution=1.0, width=200, height=200):
        rospy.init_node('utility_gridmap_node', anonymous=True)

        # Get config parameters from the parameter server
        self.number_of_robots = self.get_param('number_of_robots')
        self.number_of_auvs = self.get_param('number_of_auvs')
        self.number_of_asvs = self.get_param('number_of_asvs')
        self.pickle_path = self.get_param('pickle_path','/home/uib/MRS_ws/src/multi_robot_system/config/mission.pickle')
        
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.resolution = resolution
        self.width = width
        self.height = height
        self.map = np.zeros((width, height))
        self.asvs_positions = [[0,0,0]]  
        # self.robots_information = []  
        self.comm_signal = []
        self.acquired_data = []
        self.robots_information = [[None, None, None, None, None] for _ in range(self.number_of_auvs)]
        self.read_area_info()

        for robot_ in range(self.number_of_auvs):
            self.robots_information.append([])
            self.comm_signal.append([])
            self.acquired_data.append(0)
        
        self.grid_pub = rospy.Publisher('/utility_gridmap', OccupancyGrid, queue_size=1)
        rospy.Subscriber('/mrs/asv0_data_buffered', BufferedData, self.update_acquired_data, 0, queue_size=1)
        #Subscribers       
        for robot_agent in range(self.number_of_auvs):
            rospy.Subscriber('/robot'+str(robot_agent)+'/acoustic_communication',
                            PoseWithCovarianceStamped,
                            self.update_acoustic_info,
                            robot_agent,
                            queue_size=1)
                   
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(1)  # Frecuencia de actualización
        self.run()
    
    def read_area_info(self):
        # Open the pickle file in binary mode
        with open(self.pickle_path, 'rb') as file:
            # Load the data from the file
            data = pickle.load(file)

        # Access different data from the loaded data
        self.cluster_centroids = data['array1']
        self.voronoi_polygons = data['array2']
        self.main_polygon = data['array3']
        self.main_polygon_centroid = data['array4']
        self.voronoi_offset_polygons = data['array5']
        self.regular_objects = data['array6']
        self.priority_objects = data['array7']
            
    def update_acoustic_info(self, msg, robot_agent):
        # tranform from quaternion to euler angles
        rpy = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # normalize the position covariance
        normalized_covariance = self.normalize(msg.pose.covariance[0],0,4.5,1,0)
        # fill the robots_information array with the robots information received from the PoseWithCovariance 
        self.robots_information[robot_agent] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, rpy[2], normalized_covariance]
        rssi = self.get_communication_signal(0, robot_agent)
        utility = 1+self.acquired_data[robot_agent]+ rssi
        # print("Storage: "+str(self.acquired_data[robot_agent])+" RSSI: "+str(rssi)+ "utility: "+str(utility)+" AUV position: "+str(self.robots_information[robot_agent][0])+","+str(self.robots_information[robot_agent][1]))
        self.update_grid(robot_agent, utility)

    def normalize(self,value, min_val, max_val, new_min, new_max):
        normalized_value = new_min + (value - min_val) * (new_max - new_min) / (max_val - min_val)
        return normalized_value
    
    def update_acquired_data(self, msg, asv):
        # Procesar los datos obtenidos y actualizar la utilidad
        for auv_id in range(self.number_of_auvs):
            rssi = self.get_communication_signal(asv, auv_id)
            self.acquired_data[auv_id] = msg.storage[auv_id]
            utility = 1+ self.acquired_data[auv_id] + rssi
            # print("Storage: "+str(self.acquired_data[auv_id])+" RSSI: "+str(rssi)+ "utility: "+str(utility)+" AUV position: "+str(self.robots_information[auv_id][0])+","+str(self.robots_information[auv_id][1]))
            self.update_grid(auv_id, utility)

    def get_communication_signal(self, asv_id, auv_id):
        distance = self.get_distance(asv_id, auv_id)
        rssi = -47.537 - (0.368 * distance) + (0.00132 * distance**2) - (0.0000016 * distance**3)
        normalized_value = (rssi + 45) / (-85 + 45)
        self.comm_signal[auv_id] = normalized_value
        return normalized_value
    
    def get_distance(self, asv_id, auv_id):
        x_diff =  self.asvs_positions[asv_id][0] - self.robots_information[auv_id][0]
        y_diff =  self.asvs_positions[asv_id][1] - self.robots_information[auv_id][1] 
        distance =  sqrt(x_diff**2 + y_diff**2 )
        return(distance)
       
    def update_grid(self, auv_id, utility):
        # Transform AUV position to match grid map's reference frame
        x = self.robots_information[auv_id][0] -(self.main_polygon_centroid.x - (self.width * self.resolution) / 2.0)
        y = self.robots_information[auv_id][1] -(self.main_polygon_centroid.y - (self.height * self.resolution) / 2.0)

        # Convert to grid coordinates
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)

        # Ensure the grid indices are within bounds
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            self.map[grid_x, grid_y] = utility

    def publish_gridmap(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "world_ned"  

        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height

        grid_msg.info.origin = Pose()
        grid_msg.info.origin.position.x = self.main_polygon_centroid.x - (self.width * self.resolution) / 2.0
        grid_msg.info.origin.position.y = self.main_polygon_centroid.y - (self.height * self.resolution) / 2.0
        grid_msg.info.origin.position.z = 0 
        grid_msg.info.origin.orientation.x = 0 
        grid_msg.info.origin.orientation.y = 0 
        grid_msg.info.origin.orientation.z = 0 
        grid_msg.info.origin.orientation.w = 1

        min_val = self.map.min()
        max_val =  self.map.max()
        # print("Max: "+str(max_val)+" Min: "+str(min_val)) 
        normalized_map = np.interp(self.map, (min_val, max_val), (0, 100))
        # print(normalized_map)
        grid_msg.data = np.array(normalized_map, dtype=np.int32).flatten().tolist()

        self.grid_pub.publish(grid_msg)

    def get_param(self, param_name, default = None):
        if rospy.has_param(param_name):
            param_value = rospy.get_param(param_name)
            return param_value
        elif default is not None:
            return default
        else:
            rospy.logfatal('[%s]: invalid parameters for %s in param server!', self.name, param_name)
            rospy.logfatal('[%s]: shutdown due to invalid config parameters!', self.name)
            exit(0)
    
    def run(self):
        while not rospy.is_shutdown():
            self.publish_gridmap()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        UtilityGridMap()
    except rospy.ROSInterruptException:
        pass