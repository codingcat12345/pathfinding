import sys, pygame
import random
import numpy as np
from shapely.geometry import LineString, Polygon


class Block(pygame.sprite.Sprite):
    def __init__(self, width, height,xpos,ypos,color=(255,255,255)):
        super().__init__()#=pygame.sprite.Sprite.__init__(self)
        self.rect=pygame.Rect(xpos,ypos,width,height)
        self.color=color
        self.image = pygame.Surface([width, height])
        self.image.fill(color)
        self.rp=np.array([xpos+width,ypos]) #right up
        self.lp=np.array([xpos,ypos]) 
        self.rd=np.array([xpos+width,ypos+height])#right down
        self.ld=np.array([xpos,ypos+height])
        

class Line(pygame.sprite.Sprite):
    def __init__(self,start,end,color=(100,100,244),width=1):
        super().__init__()
        self.start=start
        self.end=end
        self.color=color
        self.rect=pygame.Rect(0,0,0,0)
        self.width=width
    def draw(self,window):
        self.rect=pygame.draw.line(window,self.color,self.start,self.end,width=self.width)

class Circle(pygame.sprite.Sprite):
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


def if_collide(new_point,block_list):
    bool=False
    for block in block_list:
        if new_point[0] > block.rect.left and  new_point[0] < block.rect.right \
        and new_point[1] < block.rect.bottom and new_point[1] > block.rect.top:
            bool=True
    return bool

def does_point_overscreen(new_point,screen):
    bool=False
    if new_point[0] > screen.get_width() or  new_point[0] < 0 or \
    new_point[1] > screen.get_height() or new_point[1] < 0:
        bool=True
    return bool

def does_line_cross_block(new_point,close_point,block_list):
    bool=False
    line_coords = [new_point,close_point]
    line = LineString(line_coords)
    for block in block_list:
        area_coords = [block.rp,block.lp,block.rd,block.ld]
        area = Polygon(area_coords)
        if line.intersects(area):
            bool=True
    return bool

def find_most_closed_point(point_list,point_in):
    norm_min = max(np.abs(point_list[0][0,:]-point_in))
    point_out = np.array([0,0])
    for pt in point_list:
        ptarray=np.array(pt)
        inf_norm = max(np.abs(ptarray[0,:]-point_in))
        if inf_norm <= norm_min:
            norm_min = inf_norm
            point_out = pt[0,:]
    return point_out

def RRTstar(block,start,end,step_size,max_itertime,screen,point_list):
    path_tree_buffer = []
    path_tree_buffer.append(np.vstack((start,start)))
    b_find=False
    for i in range(max_itertime):
        x_rand = random.randrange(screen.get_width())
        y_rand = random.randrange(screen.get_height())
        p_rand=np.array([x_rand,y_rand])
        p_close = find_most_closed_point(path_tree_buffer,p_rand)
        direction = p_rand-p_close
        unit_vector = direction / np.linalg.norm(direction)
        p_new = p_close+step_size*unit_vector
        #print(p_rand,p_close,p_new)
        b_collide=if_collide(p_new,block)
        b_p2p=does_line_cross_block(p_new,p_close,block_list)
        b_screen=does_point_overscreen(p_new,screen)
        if not (b_collide or b_p2p or b_screen):
            p_new=np.vstack((p_new,p_close))
            # print(p_new)
            path_tree_buffer.append(p_new)
            draw_path(screen,point_list,p_new[0,:],p_close)
            b_p2end=does_line_cross_block(p_new[0,:],end,block_list)
            if max(np.abs(end-p_new[0,:]))<=step_size and not b_p2end:
                b_find=True
                l_new=Line(p_new[0,:],end)
                l_new.draw(screen)
                pygame.display.update()
                break
    if b_find==False:
        print("can't find path during iter time")
        return 0
    else:
        path=backward_path_finding(path_tree_buffer,start)
        redrawpath(screen,path,ORIGION,END_POINT,block_list)
    return path

def draw_path(screen,point_list,p_new,p_close):
    clock = pygame.time.Clock()
    c_new=Circle((0,0,255),5,p_new)
    point_list.add(c_new)
    l_new=Line(p_close,p_new)
    point_list.draw(screen)
    l_new.draw(screen)
    clock.tick(20) #frames per second
    pygame.display.update()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

def backward_path_finding(path_tree_buffer,start):
    i=len(path_tree_buffer)-1
    path=[]
    # path.append(path_tree_buffer[i][0,:])
    while all(path_tree_buffer[i][0,:] != start):
        for j in range(len(path_tree_buffer)):
            if all(path_tree_buffer[j][0,:] == path_tree_buffer[i][1,:]):
                path.append(path_tree_buffer[i][0,:])
                # path_tree_buffer.pop(i)
                i=j
    return path

def redrawpath(screen,path,ORIGION,END_POINT,block_list):
    path_list=pygame.sprite.Group()
    path_list.add(END_POINT)
    line_list=[]
    pos_old=END_POINT.pos
    for pt in path:
        line=Line(pos_old,pt,color=(255,255,0))
        line_list.append(line)
        circle_new=Circle((255,255,0),5,pt)
        path_list.add(circle_new)
        pos_old=pt
    line=Line(pos_old,ORIGION.pos,color=(255,255,0))
    line_list.append(line)
    path_list.add(ORIGION)
    screen.fill((0,0,0))
    block_list.draw(screen)
    path_list.draw(screen)
    for l in line_list:
        l.draw(screen)
    clock = pygame.time.Clock()
    clock.tick(2)
    pygame.display.update()


if __name__ == '__main__': 
    pygame.init()
    pygame.display.set_caption('RRTstar')
    SCREEN_WIDTH=800
    SCREEN_HIGHT=600
    screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HIGHT))
    screen.fill((0,0,0))
    #---------------------setting start and end point
    x0,y0=50,50
    START_P=np.array([x0,y0])
    x_end,y_end=550,550
    END_P=np.array([x_end,y_end])
    ORIGION=Circle((0,255,0),10,START_P)
    END_POINT=Circle((255,0,0),10,END_P)
    point_list = pygame.sprite.Group()
    point_list.add(ORIGION)
    point_list.add(END_POINT)
    #---------------------setting block 
    BLOCK1=Block(10,275,150,0)
    BLOCK2=Block(10,275,150,325)
    BLOCK3=Block(30,300,200,300-300/2)
    block_list = pygame.sprite.Group()
    block_list.add(BLOCK1)
    block_list.add(BLOCK2)
    # block_list.add(BLOCK3)
    #---------------------rrt and display
    running=True
    idx=0
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
        screen.fill((0,0,0))
        block_list.draw(screen)
        if idx<1:
            path=RRTstar(block_list,START_P,END_P,50,1000,screen,point_list)
            # print(len(path))
            # for pt in path:
            #     print(pt)
            idx=1       