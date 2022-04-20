 # obtain the distance[m] between the different points of the polygon

        # self.distance =[]
        # for i in range(len(self.north_position)):

        #     if(i==(len(self.north_position)-1)):
        #         x_distance = self.north_position[i]-self.north_position[(len(self.north_position)-1)]
        #         y_distance = self.east_position[i]-self.east_position[(len(self.north_position)-1)]

        #     else:
        #         x_distance = self.north_position[i]-self.north_position[(i+1)]
        #         y_distance = self.east_position[i]-self.east_position[(i+1)]

        #     cartesian_distance = np.sqrt(x_distance**2 + y_distance**2)
        #     self.distance.append(cartesian_distance)

        # # find the max distance reference_points
        # max_distance = max(self.distance)
        # index =  self.distance.index(max_distance)
        # self.reference_points =[]
        # self.reference_points.append (index)
        # self.reference_points.append (index-1)

        # if (index+1 == len(self.north_position)-1):
        #     self.reference_points.append(0)

    def define_path_coverage(self):
        # define the ecuation of the line between the reference_points
        # The reference_point are the points that defines the major edge of the polygon, for example the 0 and 5 vertex
        point_A = self.local_points[self.reference_points[0]]
        point_B = self.local_points[self.reference_points[1]]
        # Extract reference line ecuation parameters
        self.slope_reference = (point_B[1]-point_B[0])/(point_A[1]-point_A[0])
        self.n_reference = point_B[1] - (self.slope_reference*point_B[0]) 
        # self.line_ecuations.append([self.reference_points[0],self.reference_points[1],self.slope_reference,self.n_reference])
        self.find_neighbour_points(self.reference_points[0],self.reference_points[1]) 
        if(self.neig_A > self.neig_B):
            self.neig_B = self.neig_A + 1
        else:
            self.neig_B = self.neig_A - 1

        point_A = self.local_points[self.neig_A]
        point_B = self.local_points[self.neig_B]
    
        self.extract_line_ecuation(point_A,point_B)
        self.find_angle_btw_edges()
        # for i in range(len(self.reference_points)):
        self.extract_desired_point(self.reference_points[1])

    def extract_desired_point(self,reference_point):
        if(self.angle>90):
            self.angle = self.angle-90
        y = self.point_distance
        x = np.tan(self.angle)*y
        self.desired_point = []
        reference_local_point = self.local_points[reference_point]
        desired_point_north = reference_local_point[0] + x
        desired_point_east = reference_local_point[1] + y
        desired_point = [desired_point_north,desired_point_east]
        self.desired_point.append(desired_point)
      

    def find_angle_btw_edges(self):
        for i in range(len(self.line_ecuations)):
            m1=self.slope_reference
            m2=self.line_ecuations[i][2]
            print(m1)
            print(m2)
            pi= 22/7
            self.angle = abs((m1-m2)/(1+(m2*m1)))
            self.angle= np.arctan(self.angle)
            self.angle = self.angle*(180/pi)
        
        return(self.angle)


    def extract_line_ecuation(self, point_1, point_2):
        print("**************extract_line_ecuation******************")
        self.slope = (point_2[1]-point_2[0])/(point_1[1]-point_1[0])
        # ecuacion recta punto, pendiente: y=m(x-x1)+y1=mx-mx1+y1 => y=mx+n donde n=y1-mx1
        self.n = point_2[1] - (self.slope*point_2[0]) 
        self.line_ecuations.append([self.neig_A,self.neig_B,self.slope,self.n])
        # print("11111111111111111111111111111111111")
        # print(self.line_ecuations)
        return self.line_ecuations

    # The find_neighbour_points method finds the nearest edge from a point, for example given the 5 and 0 points from a 5 edges polygon, 
    # the neares edge of 5 is 4 and the neares edge of 0 is 1
    def find_neighbour_points(self,point_A,point_B):
        if(point_A > point_B):          
            self.neig_A = point_A - 1
            self.neig_B = point_B + 1
        else:
            self.neig_A = point_A + 1
            self.neig_B = point_B - 1
        return(self.neig_A,self.neig_B)