import sys, pygame
import numpy as np
import math
import cv2

class Dijkstra:
    def __init__(self, origin_x, origin_y, screen,img_matrix):
        self.origin_x=origin_x
        self.origin_y=origin_y
        self.screen=screen
        self.x_width=screen.get_width()
        self.y_width=screen.get_height()
        self.img=img_matrix
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
            path:
        """
        start_node=self.Node(start_x,start_y,0,-1)
        goal_node=self.Node(goal_x,goal_y,0,-1)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node
        for i in range(max_itertime):
            current_id = min(open_set, key=lambda o: open_set[o].cost)
            current_node = open_set[current_id]
            # print("current id",current_id)
            ''' find goal '''
            if current_node.x == goal_node.x and current_node.y == goal_node.y:
                print("Find goal with cost",current_node.cost,"after ",i,"th iteration")
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
                    rect=(node.x,node.y,1,1)
                    pygame.draw.rect(screen,(255,0,0),rect)
                    pygame.display.update()
                else:
                    #unvisited
                    if node.cost < open_set[node_id].cost:
                        #the cost and parent be changed while x,y keep the same
                        open_set[node_id] = node
            pygame.draw.circle(screen,(0,0,255),start,2.5)
            whether_quit()
        path = self.calc_final_path(goal_node, closed_set)
        self.draw_path(path,closed_set)
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
        if self.img[node.y][node.x]==0:
            return False
        return True
    
    def draw_path(self,path,closed_set):
        for id in path:
            node=closed_set[id]
            rect=(node.x,node.y,2,2)
            pygame.draw.rect(self.screen,(255,255,0),rect)
        pygame.display.update()

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



def whether_quit():
    #this function use to determine if user push the close bottom of the window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

def image_process(filepath,SCREEN_WIDTH,SCREEN_HIGHT):
    img=cv2.imread(filepath,cv2.IMREAD_GRAYSCALE)
    img=cv2.resize(img, (SCREEN_WIDTH,SCREEN_HIGHT))
    size=img.shape
    for i in range(size[0]):
        for j in range(size[1]):
            if img[i][j]<200:
                img[i][j]=0
    return img

def show_img_OpenCV(img):
    # 顯示圖片，第一個參數表示視窗名稱，第二個參數就是你的圖片。
    cv2.imshow('My Image', img)

    # 按下任意鍵則關閉所有視窗
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__': 
    pygame.init()
    pygame.display.set_caption('dijkstras')
    SCREEN_WIDTH=360
    SCREEN_HIGHT=360
    screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HIGHT))
    screen.fill((0,0,0))
    running=True
    filepath="./obstacle4.png"
    img_matrix=image_process(filepath,SCREEN_WIDTH,SCREEN_HIGHT)
    # show_img_OpenCV(img_matrix)
    background=pygame.surfarray.make_surface(np.transpose(img_matrix))
    dijkstra=Dijkstra(0,0,screen,img_matrix)
    idx=0
    start=np.array([10,10])
    goal=np.array([350,350])
    while running:
        if idx<1:
            screen.blit(background, (0, 0))
            pygame.draw.circle(screen,(0,255,0),goal,2.5)
            pygame.draw.circle(screen,(0,0,255),start,2.5)
            pygame.display.update()
            path=dijkstra.planning(start[0],start[1],goal[0],goal[1],1000000)
            idx=1   
        whether_quit()