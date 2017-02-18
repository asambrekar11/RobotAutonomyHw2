import numpy
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        start = start_config;
        # tree.AddVertex(goal_config)
        while True :
            q_rand = self.planning_env.GenerateRandomConfiguration()
            vid, q_near = tree.GetNearestVertex(q_rand)
            if self.planning_env.Extend(q_rand,q_near) != 'none':
                q_new = self.planning_env.Extend(q_near,q_rand)
                s = tree.AddVertex(q_new)
                tree.AddEdge(s,s+1)
                self.planning_env.PlotEdge(q_near,q_new);


        
        plan.append(start_config)
        plan.append(goal_config)
        
        return plan
