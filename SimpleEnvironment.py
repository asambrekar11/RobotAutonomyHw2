import numpy
import matplotlib.pyplot as pl
import math

class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p=0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration
        #
        # config[0] = numpy.random.uniform(lower_limits[0],upper_limits[0],1)
        # config[1] = numpy.random.uniform(lower_limits[1],upper_limits[1],1)
        table = self.robot.GetEnv().GetBodies()[1]
        transform = self.robot.GetTransform()
        original_transform = self.robot.GetTransform()
        inCollision = True;
        while inCollision==True :
            #if numpy.random.random_sample() > self.p :
                # print "random"
            config[0] = numpy.random.uniform(lower_limits[0],upper_limits[0],1)
            config[1] = numpy.random.uniform(lower_limits[1],upper_limits[1],1)
            #else :
                # print "goal"
               # config[0] = self.goal_config[0]
               # config[1] = self.goal_config[1]
               # break

            transform[0,3] = config[0]
            transform[1,3] = config[1]
            self.robot.SetTransform(transform)
            if self.robot.GetEnv().CheckCollision(self.robot,table)==False :
                self.robot.SetTransform(original_transform)
                break

        return numpy.array(numpy.append(config[0], config[1]))

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        return numpy.linalg.norm(numpy.array(start_config) - numpy.array(end_config))
        

    def Extend(self, start_config, end_config, epsilon):
            
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        interpolated_config = [0] * 2;
        dir_xm = (end_config[0] - start_config[0])
        dir_ym = (end_config[1] - start_config[1])

        ratio=dir_xm/dir_ym
      
        dir_x=math.sqrt(epsilon*epsilon/(1+(1/ratio)*(1/ratio)))

        dir_y=dir_x/abs(ratio)

        if dir_xm < 0:
            dir_x=dir_x*-1
        if dir_ym < 0:
            dir_y=dir_y*-1

        interpolated_config[0] = start_config[0] + dir_x
        interpolated_config[1] = start_config[1] + dir_y

        table = self.robot.GetEnv().GetBodies()[1]
        transform = self.robot.GetTransform()
        original_transform = self.robot.GetTransform()

        transform[0,3] = interpolated_config[0]
        transform[1,3] = interpolated_config[1]

        self.robot.SetTransform(transform)
        if self.robot.GetEnv().CheckCollision(self.robot,table)==False :
            self.robot.SetTransform(original_transform)
            return True , numpy.array(numpy.append(interpolated_config[0], interpolated_config[1]))
            
        else :
            self.robot.SetTransform(original_transform)
            return False, numpy.array([0,0])

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

