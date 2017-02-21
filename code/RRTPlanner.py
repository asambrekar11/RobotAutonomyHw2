import numpy
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.1):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        self.planning_env.SetGoalParameters(goal_config,0.5)

        # start = start_config;
        goal_reached = False
        # tree.AddVertex(goal_config)
        #i=0

        currentEdgeId=0
         
        while goal_reached == False :
            q_rand = self.planning_env.GenerateRandomConfiguration()
            v= q_rand == start_config
            if v.all() == False:
               vid, q_near = tree.GetNearestVertex(q_rand)
               is_path, q_new = self.planning_env.Extend(q_near,q_rand)
               if is_path != False:
                  s = tree.AddVertex(q_new)
                  tree.AddEdge(vid,s)
                  self.planning_env.PlotEdge(q_new,q_near);
                  #i=i+1
                  # print q_rand
                  print numpy.linalg.norm(q_new - goal_config)
                  if numpy.linalg.norm(q_new - goal_config) < epsilon:
                     goal_reached = True
                     currentEdgeId=s


        #list(tree.edges.keys())
        plan.append(goal_config)
        done=False       
        while done == False:
            vertexId=tree.edges[currentEdgeId]
            vid=vertexId
            plan.append(tree.vertices[vid])
            print tree.vertices[vid]
            currentEdgeId=vid
            if vid==0:
               done=True

        plan.reverse()

        print start_config

        print goal_config
        
        return plan
