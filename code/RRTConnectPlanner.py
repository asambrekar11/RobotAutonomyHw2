import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = .02):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        self.planning_env.SetGoalParameters(goal_config, 0.2)

        # start = start_config;
        connection_reached = False
        # tree.AddVertex(goal_config)
        #i=0
        vidf=0
        vidr=0

        fcheck=start_config
        rcheck=goal_config
         
        while connection_reached == False :
            greedf = False
            greedr = False
            q_rand = self.planning_env.GenerateRandomConfiguration()

            #front tree
            vf= q_rand == start_config
            if vf.all() == False:  
               vidf, q_nearf = ftree.GetNearestVertex(q_rand)
               is_path, q_newf = self.planning_env.Extend(q_nearf,q_rand)
               if is_path != False:
                  sf = ftree.AddVertex(q_newf)
                  ftree.AddEdge(vidf,sf)
                  self.planning_env.PlotEdge(q_newf,q_nearf);  
                  greedf=True     
                  fcheck=q_newf
               while (greedf and numpy.linalg.norm(q_rand - q_newf) > epsilon):
                  vidf, q_nearf = ftree.GetNearestVertex(q_rand)            
                  is_path, q_newf = self.planning_env.Extend(q_nearf,q_rand)
                  if is_path != False:
                     sf = ftree.AddVertex(q_newf)
                     ftree.AddEdge(vidf,sf)
                     self.planning_env.PlotEdge(q_newf,q_nearf);
                     fcheck=q_newf
                  else:
                     greedf = False

               vidr, q_nearr = rtree.GetNearestVertex(q_newf)
               vidf, q_nearf = ftree.GetNearestVertex(q_nearr)                 
               is_path, q_newf = self.planning_env.Extend(q_nearf,q_nearr)
               if is_path != False:
                  sf = ftree.AddVertex(q_newf)
                  ftree.AddEdge(vidf,sf)  
                  self.planning_env.PlotEdge(q_newf,q_nearf);             
                  greedf=True
                  fcheck=q_newf
               while (greedf and numpy.linalg.norm(q_newf - q_nearr) > epsilon):
                  vidr, q_nearr = rtree.GetNearestVertex(q_newf)
                  vidf, q_nearf = ftree.GetNearestVertex(q_nearr)  
                  is_path, q_newf = self.planning_env.Extend(q_nearf,q_nearr)
                  if is_path != False:
                     sf = ftree.AddVertex(q_newf)
                     ftree.AddEdge(vidf,sf)
                     self.planning_env.PlotEdge(q_newf,q_nearf);
                     fcheck=q_newf
                  else:
                     greedf = False
               if numpy.linalg.norm(fcheck - q_nearr) < epsilon:
                  connection_reached = True
                  break



            #rear tree
            vr= q_rand == goal_config
            if vr.all() == False:  
               vidr, q_nearr = rtree.GetNearestVertex(q_rand)
               is_path, q_newr = self.planning_env.Extend(q_nearr,q_rand)
               if is_path != False:
                  sr = rtree.AddVertex(q_newr)
                  rtree.AddEdge(vidr,sr)    
                  rcheck=q_newr  
                  greedr=True         
               while (greedr and numpy.linalg.norm(q_rand - q_newr) > epsilon):
                  vidr, q_nearr = rtree.GetNearestVertex(q_rand)
                  is_path, q_newr = self.planning_env.Extend(q_nearr,q_rand)
                  if is_path != False:
                     sr = rtree.AddVertex(q_newr)
                     rtree.AddEdge(vidr,sr)
                     rcheck=q_newr   
                  else:
                     greedr = False

               vidf, q_nearf = ftree.GetNearestVertex(q_newr)
               vidr, q_nearr = rtree.GetNearestVertex(q_nearf)                 
               is_path, q_newr = self.planning_env.Extend(q_nearr,q_nearf)
               if is_path != False:
                  sr = rtree.AddVertex(q_newr)
                  rtree.AddEdge(vidr,sr) 
                  self.planning_env.PlotEdge(q_newr,q_nearr); 
                  rcheck=q_newr             
                  greedr=True
               while (greedr and numpy.linalg.norm(q_newr - q_nearf) > epsilon):
                  vidf, q_nearf = ftree.GetNearestVertex(q_newr)
                  vidr, q_nearr = rtree.GetNearestVertex(q_nearf)  
                  is_path, q_newr = self.planning_env.Extend(q_nearr,q_nearf)
                  if is_path != False:
                     sr = rtree.AddVertex(q_newr)
                     rtree.AddEdge(vidr,sr)
                     self.planning_env.PlotEdge(q_newr,q_nearr);
                     rcheck=q_newr
                  else:
                     greedr = False
               if numpy.linalg.norm(rcheck - q_nearf) < epsilon:
                  connection_reached = True
                  break

        done=False       
        planf=[]
        currentEdgeId=vidf
        while done == False:
            if currentEdgeId==0:
               vertexId=ftree.edges[1]
            else:
               vertexId=ftree.edges[currentEdgeId]
            vid=vertexId[0]
            planf.append(ftree.vertices[vid])
            currentEdgeId=vid
            if vid==0:
               done=True


        done=False       
        planr=[]
        currentEdgeId=vidr
        while done == False:
            if currentEdgeId==0:
               vertexId=rtree.edges[1]
            else:
               vertexId=rtree.edges[currentEdgeId]
            vid=vertexId[0]
            planr.append(rtree.vertices[vid])
            currentEdgeId=vid
            if vid==0:
               done=True
           
        planf.reverse()
        planf.extend(planr)

        plan=planf
        
        return plan
