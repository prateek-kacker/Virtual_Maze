import numpy as np
import random
class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes Kuitaze
        the robot is placed in.
        '''

        self.location = [0, 0]
        self.heading = 1 # heading up
        self.maze_dim = maze_dim
        self.Q = dict() # state dictionary
        self.learning = True
        self.learning_counter=0
        self.step_1 = 1
    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        ###################### This block is for second trial run######################
        if self.learning == False:
            rotation,movement = self.trial_2_execution()
                        
        self.learning_counter=self.learning_counter+1     ########### State counter
        ################################Trial Run 1##########################################
        if self.learning == True: ############# Trial run 1
            self.map_creation_in_Q_dict(sensors)
            rotation,movement,reached_goal=self.trial_run_1_decision(sensors)
            self.location_tracking(rotation, movement)        
            percentage_covered = self.percentage_covered_function()
            ##### Generating the reset signal to the mouse based on certain criterion
            if self.learning_counter == 1000 or (percentage_covered > 70 and reached_goal==1):
                self.learning= False
                self.heading = 1
                self.location=[0,0]
                if reached_goal ==1:
                    rotation='Reset'
                    movement='Reset'
                self.value_iteration_algorithm_initialization()
                self.value_iteration_algorithm()
                
        return rotation, movement
    def trial_2_execution(self):
        ####### This section finds the utility of the states which are next to the current state ###########
            state= (self.location[0],self.location[1])
            utility = dict()
            if self.Q[state][0]>0:
                next_state_0= (self.location[0]-1,self.location[1])
                if next_state_0 in self.Q:
                    utility[0] = self.Q[next_state_0]['utility']
                else:
                    utility[0] = 0
            else:
                utility[0]=0
            if self.Q[state][1]>0:
                next_state_1 = (self.location[0],self.location[1]+1)
                if next_state_1 in self.Q:
                    utility[1] = self.Q[next_state_1]['utility']
                else:
                    utility[1] = 0
            else:
                utility[1]=0
            if self.Q[state][2]>0:
                next_state_2 = (self.location[0]+1,self.location[1])
                if next_state_2 in self.Q:
                    utility[2] = self.Q[next_state_2]['utility']
                else:
                    utility[2] = 0
            else:
                utility[2]=0
            if self.Q[state][3]>0:
                next_state_3 = (self.location[0],self.location[1]-1)
                if next_state_3 in self.Q:
                    utility[3] = self.Q[next_state_3]['utility']
                else:
                    utility[3] = 0
            else:
                utility[3]=0
            #################### This section initializes variables which 
            max_var = 0 ####### to know the direction of the max_utility
            max_utility = utility[0] ##### Initializing max_utility with utility_0
            rotation_array=dict() ######## This is a guide to rotation angle when self.heading and direction is known and 
            self_heading=dict() ### This helps in giving correct direction to move one we know the current direction and the direction which we should go.  
            rotation_array[0]={0:0,1:90,2:90,3:-90}
            rotation_array[1]={0:-90,1:0,2:90,3:90}
            rotation_array[2]={0:90,1:-90,2:0,3:90}
            rotation_array[3]={0:90,1:90,2:-90,3:0}
            self_heading[0]={0:0,1:1,2:1,3:3}
            self_heading[1]={0:0,1:1,2:2,3:2}
            self_heading[2]={0:3,1:1,2:2,3:3}
            self_heading[3]={0:0,1:0,2:2,3:3}
            ########### THis section is helps to identify the direction of the movement and the number of steps which should be taken.
            for i in range(4):
                if max_utility < utility[i]:
                    max_utility = utility[i]
                    max_var = i
            rotation=rotation_array[self.heading][max_var]
            self.heading=self_heading[self.heading][max_var]
            if max_var==0:############### if the direction decided is going left  
                if self.heading == 2: ######### But we are pointing in right
                    movement = 0
                else:
                    if (self.location[0]-2,self.location[1]) in self.Q :  ####### if state next to next_state in the direction of max_var is available.
                        if self.Q[self.location[0]-2,self.location[1]]['utility'] > max_utility and self.Q[self.location[0]-1,self.location[1]][max_var]>0: ### If that state is accesible and has utilty requirements
                            max_utility_2= self.Q[self.location[0]-2,self.location[1]]['utility']
                            if (self.location[0]-3,self.location[1]) in self.Q: ####### If the state is next to next to the next_state direction of max_var.
                                if self.Q[self.location[0]-2,self.location[1]][max_var]>0 and self.Q[self.location[0]-3,self.location[1]]['utility'] > max_utility_2: ## If the state is accesible and has utility requirements
                                    self.location[0]=self.location[0]-3
                                    movement=3
                                else:
                                    self.location[0]=self.location[0]-2
                                    movement=2
                            else:
                                self.location[0]=self.location[0]-2
                                movement=2
                        else:
                            self.location[0]=self.location[0]-1
                            movement=1
                            
                    else:
                        self.location[0]=self.location[0]-1
                        movement=1
            elif max_var==1: #### If the direction decided is going up
                if self.heading == 3: ####### If point towards down
                    movement = 0
                else:
                    if (self.location[0],self.location[1]+2) in self.Q :
                        if self.Q[self.location[0],self.location[1]+2]['utility'] > max_utility and self.Q[self.location[0],self.location[1]+1][max_var]>0:
                            max_utility_2= self.Q[self.location[0],self.location[1]+2]['utility']
                            if (self.location[0],self.location[1]+3) in self.Q:
                                if self.Q[self.location[0],self.location[1]+2][max_var]>0 and self.Q[self.location[0],self.location[1]+3]['utility'] > max_utility_2:
                                    self.location[1]=self.location[1]+3
                                    movement=3
                                else:
                                    self.location[1]=self.location[1]+2
                                    movement=2
                            else:
                                self.location[1]=self.location[1]+2
                                movement=2
                        else:
                            self.location[1]=self.location[1]+1
                            movement=1
                    else:
                        self.location[1]=self.location[1]+1
                        movement=1
            elif max_var==2: ############ if the direction decided is going right
                if self.heading==0:
                    movement = 0 ######### if the mouse point towards left
                else:
                    if (self.location[0]+2,self.location[1]) in self.Q :
                        if self.Q[self.location[0]+2,self.location[1]]['utility'] > max_utility and self.Q[self.location[0]+1,self.location[1]][max_var]>0:
                            max_utility_2= self.Q[self.location[0]+2,self.location[1]]['utility']
                            if (self.location[0]+3,self.location[1]) in self.Q:
                                if self.Q[self.location[0]+2,self.location[1]][max_var]>0 and self.Q[self.location[0]+3,self.location[1]]['utility'] > max_utility_2:
                                    self.location[0]=self.location[0]+3
                                    movement=3
                                else:
                                    self.location[0]=self.location[0]+2
                                    movement=2
                            else:
                                self.location[0]=self.location[0]+2
                                movement=2
                        else:
                            self.location[0]=self.location[0]+1
                            movement=1
                    else:
                        self.location[0]=self.location[0]+1
                        movement=1
            else: ############## if the direction decided is down
                if self.heading == 1:######## if the mouse is pointing towards up
                    movement = 0
                else:
                    if (self.location[0],self.location[1]-2) in self.Q :
                        if self.Q[self.location[0],self.location[1]-2]['utility'] > max_utility and self.Q[self.location[0],self.location[1]-1][max_var]>0:
                            max_utility_2= self.Q[self.location[0],self.location[1]-2]['utility']
                            if (self.location[0],self.location[1]-3) in self.Q:
                                if self.Q[self.location[0],self.location[1]-2][max_var]>0 and self.Q[self.location[0],self.location[1]-3]['utility'] > max_utility_2:
                                    self.location[1]=self.location[1]-3
                                    movement=3
                                else:
                                    self.location[1]=self.location[1]-2
                                    movement=2
                            else:
                                self.location[1]=self.location[1]-2
                                movement=2
                        else:
                            self.location[1]=self.location[1]-1
                            movement=1
                    else:
                        self.location[1]=self.location[1]-1
                        movement=1

            return rotation,movement
    
    def map_creation_in_Q_dict(self,sensors):
            location_tuple = (self.location[0],self.location[1]) ####### intialization function
            
            ######### This section helps you to pre-process information  from the sensors and convert it to barrier information
            
            if sensors[0]>0:
                anticlockwise=1
            else:
                anticlockwise=0
            if sensors[1]>0:
                straight = 1
            else:
                straight = 0
            if sensors[2]>0:
                clockwise=1
            else:
                clockwise=0
            
            ###### Based on the direction of the movement, the sensor information should be rotated to ge the exact map of the maze            
            if (location_tuple in self.Q): 
                a=1
            else:
                if self.heading ==1:
                    self.Q[location_tuple]= {0:anticlockwise, 1:straight, 2:clockwise, 3:1}
                elif self.heading == 2:
                    self.Q[location_tuple]= {0:1, 1:anticlockwise, 2:straight, 3:clockwise}
                elif self.heading == 3:
                    self.Q[location_tuple] = {0:clockwise, 1:1, 2:anticlockwise, 3:straight}
                else:
                    self.Q[location_tuple] = {0:straight, 1:clockwise, 2:1, 3:anticlockwise}
            return 
    def trial_run_1_decision(self,sensors):
            action_taken=0
            high = 1.2 ## Initializing probability variables
            medium = 1.1 ## 
            low = 1
            ## Creating a weight vector blocks for each quadrant
            weight_vector_block_1=dict()
            weight_vector_block_1[0]={0:low, 1:low, 2:high, 3:high}
            weight_vector_block_1[1]={0:low, 1:high, 2:high, 3:low}
            weight_vector_block_1[2]={0:high, 1:high, 2:low, 3:low}
            weight_vector_block_1[3]={0:high, 1:low, 2:low, 3:high}
            weight_vector_block_2 = dict()
            weight_vector_block_2[0]={0:low, 1:medium, 2:high, 3:medium}
            weight_vector_block_2[1]={0:medium, 1:high, 2:medium, 3:low}
            weight_vector_block_2[2]={0:high, 1:medium, 2:low, 3:medium}
            weight_vector_block_2[3]={0:medium, 1:low, 2:medium, 3:high}
            weight_vector_block_3 = dict()
            weight_vector_block_3[0]={0:low, 1:high, 2:high, 3:low}
            weight_vector_block_3[1]={0:high, 1:high, 2:low, 3:low}
            weight_vector_block_3[2]={0:high, 1:low, 2:low, 3:high}
            weight_vector_block_3[3]={0:low, 1:low, 2:high, 3:high}
            weight_vector_block_4 = dict()
            weight_vector_block_4[0]={0:medium, 1:low, 2:medium, 3:high}
            weight_vector_block_4[1]={0:low, 1:medium, 2:high, 3:medium}
            weight_vector_block_4[2]={0:medium, 1:high, 2:medium, 3:low}
            weight_vector_block_4[3]={0:high, 1:medium, 2:low, 3:medium}
            weight_vector_block_5 = dict()
            weight_vector_block_5[0]={0:medium, 1:high, 2:medium, 3:low}
            weight_vector_block_5[1]={0:high, 1:medium, 2:low, 3:medium}
            weight_vector_block_5[2]={0:medium, 1:low, 2:medium, 3:high}
            weight_vector_block_5[3]={0:low, 1:medium, 2:high, 3:medium}
            weight_vector_block_6 = dict()
            weight_vector_block_6[0]={0:high, 1:low, 2:low, 3:high}
            weight_vector_block_6[1]={0:low, 1:low, 2:high, 3:high}
            weight_vector_block_6[2]={0:low, 1:high, 2:high, 3:low}
            weight_vector_block_6[3]={0:high, 1:high, 2:low, 3:low}
            weight_vector_block_7 = dict()
            weight_vector_block_7[0]={0:high, 1:medium, 2:low, 3:medium}
            weight_vector_block_7[1]={0:medium, 1:low, 2:medium, 3:high}
            weight_vector_block_7[2]={0:low, 1:medium, 2:high, 3:medium}
            weight_vector_block_7[3]={0:medium, 1:high, 2:medium , 3:low}
            weight_vector_block_8 = dict()
            weight_vector_block_8[0]={0:high, 1:high, 2:low, 3:low}
            weight_vector_block_8[1]={0:high, 1:low, 2:low, 3:high}
            weight_vector_block_8[2]={0:low, 1:low, 2:high, 3:high}
            weight_vector_block_8[3]={0:low, 1:high, 2:high, 3:low}
            
            reached_goal=0
            if ((self.maze_dim/2, self.maze_dim/2) in self.Q) or ((self.maze_dim/2-1, self.maze_dim/2) in self.Q) or ((self.maze_dim/2, self.maze_dim/2-1)  in self.Q) or ((self.maze_dim/2-1, self.maze_dim/2-1) in self.Q):
                reached_goal =1
            ############## Based on the location the mouse, assigning the correct weight vector to the mouse
            if sensors[0] == 0 and sensors[1]== 0 and sensors[2] == 0:
                rotation = 90
                movement = 0
            else:
                while action_taken == 0:
                    if reached_goal == 0:
                        if (self.location[0] < self.maze_dim/2-1) and self.location[1]< self.maze_dim/2-1:
                            weight_vectors=weight_vector_block_1[self.heading]
                        elif (self.location[0] < self.maze_dim/2+1) and (self.location[0]>=self.maze_dim/2-1) and self.location[1]<self.maze_dim/2-1:
                            weight_vectors = weight_vector_block_2[self.heading]
                        elif (self.location[0]>=self.maze_dim/2+1) and self.location[1]<self.maze_dim/2-1:
                            weight_vectors = weight_vector_block_3[self.heading]
                        elif (self.location[0] < self.maze_dim/2-1) and self.location[1]<self.maze_dim/2+1 and self.location[1]>=self.maze_dim/2-1:
                            weight_vectors = weight_vector_block_4[self.heading]
                        elif (self.location[0] >= self.maze_dim/2+1) and self.location[1]>=self.maze_dim/2-1 and self.location[1]<self.maze_dim/2+1:
                            weight_vectors = weight_vector_block_5[self.heading]
                        elif (self.location[0] < self.maze_dim/2-1) and self.location[1]>=self.maze_dim/2+1:
                            weight_vectors = weight_vector_block_6[self.heading]
                        elif (self.location[0] < self.maze_dim/2+1) and (self.location[0]>=self.maze_dim/2-1) and self.location[1]>=self.maze_dim/2+1:
                            weight_vectors = weight_vector_block_7[self.heading]
                        elif (self.location[0] >= self.maze_dim/2+1) and self.location[1]>=self.maze_dim/2+1:
                            weight_vectors = weight_vector_block_8[self.heading]
                        ############# Normalizaing the weight vectors
                        weight_vector_normalized = {0:float(weight_vectors[0])/(weight_vectors[0]+weight_vectors[1]+weight_vectors[2]), 1:float(weight_vectors[1])/(weight_vectors[0]+weight_vectors[1]+weight_vectors[2]), 2:float(weight_vectors[2])/(weight_vectors[0]+weight_vectors[1]+weight_vectors[2])}
                        ############### Deciding on action based on randomly generated probability and ignoring 180 movements
                        prob= random.random()
                        if prob < weight_vector_normalized[0]:
                            action =0 
                        elif prob < weight_vector_normalized[1]+weight_vector_normalized[0]:
                            action=1
                        else:
                            action =2
                    ############# If the mouse has reached the center once then allowing the mouse to explore the maze more randomly    
                    else:
                        action = random.randrange(0,3)
                    if action == 3:
                        rotation= 90
                        movement=0
                        action_taken =1
                    elif sensors[action]>0:
                        if action == 0:
                            rotation = -90
                            movement =1
                        elif action == 1:
                            rotation = 0
                            movement =1
                        else :
                            rotation= 90
                            movement =1
                        action_taken =1
            return rotation,movement,reached_goal
    
    def location_tracking(self,rotation,movement):
            ### Keeping track of the mouse in the maze by translating its action to a location and direction variable
            if self.heading == 1:
                if rotation == -90:
                    self.heading = 0
                    self.location[0]=self.location[0]-movement
                elif rotation == 0:
                    self.heading = 1
                    self.location[1]=self.location[1]+movement
                elif rotation ==90:
                    self.heading = 2
                    self.location[0]=self.location[0]+movement
            elif self.heading == 2:
                if rotation == -90:
                    self.heading = 1
                    self.location[1]=self.location[1]+movement
                elif rotation == 0:
                    self.heading = 2
                    self.location[0]=self.location[0]+movement
                elif rotation ==90:
                    self.heading = 3
                    self.location[1]=self.location[1]-movement
            elif self.heading == 3:
                if rotation == -90:
                    self.heading = 2
                    self.location[0]=self.location[0]+movement
                elif rotation == 0:
                    self.heading = 3
                    self.location[1]=self.location[1]-movement
                elif rotation ==90:
                    self.heading = 0
                    self.location[0]=self.location[0]-movement
            elif self.heading == 0:
                if rotation == -90:
                    self.heading = 3
                    self.location[1]=self.location[1]-movement
                elif rotation == 0:
                    self.heading = 0
                    self.location[0]=self.location[0]-movement
                elif rotation ==90:
                    self.heading = 1
                    self.location[1]=self.location[1]+movement
            return
    
    def percentage_covered_function(self):
            ### Calculatinng the percentage_covered variable
            count=0
            for i in range(self.maze_dim):
                for y in range(self.maze_dim):
                    state= (i,y)
                    if state in self.Q:
                        count=count+1
            percentage_covered = count*100.0/(self.maze_dim*self.maze_dim)
            return percentage_covered
    def value_iteration_algorithm_initialization(self):
        #### Value iteration algorithm utility initializaiton
                for i in range(self.maze_dim):
                    for y in range(self.maze_dim):
                        state= (i,y)
                        if (i == self.maze_dim/2) or (i == self.maze_dim/2 - 1):
                            if (y == self.maze_dim/2) or (y==self.maze_dim/2 - 1):
                                if state in self.Q:
                                    self.Q[state]['utility']=500
                                else:
                                    if (i == self.maze_dim/2) and (y == self.maze_dim/2):
                                        self.Q[state]={0:1,1:0,2:0,3:1,'utility':500}
                                    if (i == self.maze_dim/2) and (y == self.maze_dim/2 -1):
                                        self.Q[state]={0:1,1:1,2:0,3:0,'utility':500}
                                    if (i == self.maze_dim/2-1) and (y== self.maze_dim/2-1):
                                        self.Q[state]={0:0,1:1,2:1,3:0,'utility':500}
                                    if (i == self.maze_dim/2 -1) and (y==self.maze_dim/2):
                                        self.Q[state]= {0:0,1:0,2:1,3:1,'utility':500}
                                    
                            else:
                                if state in self.Q:
                                    self.Q[state]['utility']=0
                                else:
                                    self.Q[state]={0:0,1:0,2:0,3:0,'utility':0}
                        else:
                            if state in self.Q:
                                self.Q[state]['utility']=0
                            else:
                                self.Q[state]={0:0,1:0,2:0,3:0,'utility':0}
                return
    def value_iteration_algorithm(self):
        ################# Value iteration algorithm in exectution                
        for z in range(self.maze_dim*self.maze_dim):
            for i in range(self.maze_dim):
                for y in range(self.maze_dim):
                    state= (i,y)
                    if state in self.Q:
                        if (self.Q[state][0] > 0):
                            state_next=(i-1,y)
                            if (state_next in self.Q):
                                if (self.Q[state]['utility'] <self.Q[state_next]['utility']-1) and self.Q[state_next]['utility']>0:
                                    self.Q[state]['utility'] =self.Q[state_next]['utility']-1
                        if (self.Q[state][1] > 0):
                            state_next=(i,y+1)
                            if state_next in self.Q:
                                if (self.Q[state]['utility'] <self.Q[state_next]['utility']-1) and self.Q[state_next]['utility']>0:
                                    self.Q[state]['utility'] =self.Q[state_next]['utility']-1
                        if (self.Q[state][2] > 0):
                            state_next=(i+1,y)
                            if state_next in self.Q:
                                if (self.Q[state]['utility'] <self.Q[state_next]['utility']-1) and self.Q[state_next]['utility']>0:
                                    self.Q[state]['utility'] =self.Q[state_next]['utility']-1
                        if (self.Q[state][3] > 0):
                            state_next=(i,y-1)
                            if state_next in self.Q:
                                if (self.Q[state]['utility'] <self.Q[state_next]['utility']-1)  and self.Q[state_next]['utility']>0:
                                    self.Q[state]['utility'] =self.Q[state_next]['utility']-1
        return
