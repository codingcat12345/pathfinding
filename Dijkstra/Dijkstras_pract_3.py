import sys, pygame
import numpy as np
import math

class Circle(pygame.sprite.Sprite):
    #this class define a circle shape sprite
    def __init__(self, color, radius,pos):
        super().__init__()
        self.image = pygame.Surface([2*radius, 2*radius])
        self.image.fill((0,0,0))
        self.image.set_colorkey((0,0,0))
        self.pos=pos
        #When blitting this Surface onto a destination, any pixels that have the same color as 
        # the colorkey will be transparent.
        pygame.draw.circle(self.image, color,(radius,radius),radius)
        # Fetch the rectangle object that has the dimensions of the image
        # Update the position of this object by setting the values of rect.x and rect.y
        self.rect = self.image.get_rect()
        self.rect.centerx = pos[0]
        self.rect.centery = pos[1]

class Line(pygame.sprite.Sprite):
    #this class define a line shape sprite with arg :start point,end point
    def __init__(self,start,end,color=(100,100,244),width=1):
        super().__init__()
        self.start=start
        self.end=end
        self.color=color
        self.rect=pygame.Rect(0,0,0,0)
        self.width=width
    def draw(self,screen):
        #this function will draw a line on the screen
        self.rect=pygame.draw.line(screen,self.color,self.start,self.end,width=self.width)

class Dijkstra:
    def __init__(self, origin_x, origin_y, resolution, screen):
        self.origin_x=origin_x
        self.origin_y=origin_y
        self.resolution=resolution
        self.screen=screen
        self.x_width=screen.get_width()/resolution
        self.y_width=screen.get_height()/resolution
        self.motion=self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index  # index of previous Node

    def planning(self, start_x, start_y, goal_x, goal_y,max_itertime):
        """
        dijkstra path search

        input:
            start_x: start x position [m]
            start_y: start y position [m]
            goal_x: goal x position [m]
            goal_y: goal y position [m]

        output:
            return_x: x position list of the final path
            return_y: y position list of the final path
        """
        #---------------------setting start and end point
        half=self.resolution/2
        ORIGION=Circle((0,0,255),25,np.array([start_x+half,start_y+half]))
        END_POINT=Circle((255,0,0),25,np.array([goal_x+half,goal_y+half]))
        end_point_list = pygame.sprite.Group()
        end_point_list.add(ORIGION)
        end_point_list.add(END_POINT)
        #---------------------
        point_list = pygame.sprite.Group()
        start_node=self.Node(round(start_x/self.resolution),round(start_y/self.resolution),0,-1)
        goal_node=self.Node(round(goal_x/self.resolution),round(goal_y/self.resolution),0,-1)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node
        for i in range(max_itertime):
            current_id = min(open_set, key=lambda o: open_set[o].cost)
            current_node = open_set[current_id]
            # print("current id",current_id)
            self.draw_node(current_node,point_list,end_point_list)
            ''' find goal '''
            if current_node.x == goal_node.x and current_node.y == goal_node.y:
                print("Find goal with cost",current_node.cost)
                goal_node.parent_index = current_node.parent_index
                goal_node.cost = current_node.cost
                break
            ''' Remove the item on open set '''
            del open_set[current_id]
            ''' Add it to the closed set '''
            closed_set[current_id] = current_node
            ''' expand search grid based on motion model'''
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current_node.x + move_x,
                                 current_node.y + move_y,
                                 current_node.cost + move_cost, current_id)
                node_id = self.calc_index(node)
                #visited
                if node_id in closed_set:
                    continue
                if not self.verify_node(node):
                    continue
                if node_id not in open_set:
                    #Discover a new node
                    open_set[node_id] = node
                    node_center=np.array([(node.x+0.5)*self.resolution,(node.y+0.5)*self.resolution])
                    new_circle=Circle((0,255,0),25,node_center)
                    point_list.add(new_circle)
                else:
                    #unvisited
                    if node.cost < open_set[node_id].cost:
                        #the cost and parent be changed while x,y keep the same
                        open_set[node_id] = node
        path = self.calc_final_path(goal_node, closed_set)
        self.draw_path(path,closed_set,goal_node)
        return path

    def calc_final_path(self, goal_node, closed_set):
        path=[]
        parent_index = goal_node.parent_index
        path.append(parent_index)
        while parent_index != -1:
            node = closed_set[parent_index]
            parent_index = node.parent_index
            path.append(parent_index)
        path.pop()
        return path
                

    def calc_index(self, node):
        '''
            return node index by
            1->2->3
            4->5->6
        '''
        return node.y * self.x_width + node.x
    
    def verify_node(self, node):
        if node.x < 0 or node.y < 0:
            return False
        if node.x >= self.x_width or node.y >= self.y_width:
            return False
        # if self.calc_index(node)==2 or self.calc_index(node)==9:
        #     return False
        return True

    @staticmethod
    def get_motion_model():
        ''' motion model for obstacle '''
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        return motion
    
    def draw_node(self,current_node,point_list,end_point_list):
        radius=25
        self.screen.fill((0,0,0))
        point_list.draw(self.screen)
        pygame.time.delay(100)
        current_node_center=np.array([(current_node.x+0.5)*self.resolution,(current_node.y+0.5)*self.resolution])
        pygame.draw.circle(self.screen,(255,255,0),current_node_center,radius)
        end_point_list.draw(self.screen)
        pygame.time.delay(500)
        pygame.display.update()
        whether_quit()

    def draw_path(self,path,closed_set,goal_node):
        node=goal_node
        node_center_1=np.array([(node.x+0.5)*self.resolution,(node.y+0.5)*self.resolution])
        for id in path:
            node=closed_set[id]
            node_center_2=np.array([(node.x+0.5)*self.resolution,(node.y+0.5)*self.resolution])
            new_line=Line(node_center_1,node_center_2,(100,100,224),2)
            new_line.draw(self.screen)
            node_center_1=node_center_2
        pygame.display.update()




def whether_quit():
    #this function use to determine if user push the close bottom of the window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

if __name__ == '__main__': 
    pygame.init()
    pygame.display.set_caption('Dijkstras')
    SCREEN_WIDTH=800
    SCREEN_HIGHT=400
    screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HIGHT))
    running=True
    dijkstra=Dijkstra(0,0,200,screen)
    # n=dijkstra.Node(600/200,200/200,0,-1)
    # id=dijkstra.calc_index(n)
    # path=dijkstra.planning(0,0,600,200,100)
    # print(dijkstra.x_width)
    # print(id)
    idx=0
    while running:
        screen.fill((0,0,0))
        if idx<1:
            path=dijkstra.planning(0,0,600,200,100)
            print(path)
            idx=1   
        whether_quit()