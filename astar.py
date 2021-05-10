import math
import numpy as np
import time
from heapq import heappush, heappop
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import cv2

#####################################################################################
################################################################################333
# functions
# Euclidean ditnce between two points
def euclid(p,q):
    sx,sy = p[0],p[1]
    gx,gy = q[0],q[1]
    
    return math.sqrt((gx-sx)**2 + (gy-sy)**2)
   
# generic fuction to convert elements of list into string
def listToString(s):  
        str1 = ""  
        for ele in s:  
             str1 += str(ele)  
        return str1  
   

# Rounding
def rnd(N):
    x,y,t = N[0],N[1],N[2]
    x=round(x,4)
    y=round(y,4)
    t = round(t/th_angle) * th_angle
    n=t//360
    t=t-(360*n)
    t=(t/th_angle)
    return [x,y,int(t)]


# check edges
def border_check(i,j):
    if (i<clear or j>=10-clear or j<clear or i>=10-clear):
        return 0
    else:
        return 1
    
    
# define obstacles
def obstacles(x,y):
    c1 = ((np.square(x-2))+ (np.square(y-2)) <=np.square(1+clear))
    c2 = ((np.square(x-2))+ (np.square(y-8)) <=np.square(1+clear))

    sq=(x>=0) and (x<=1.75+clear) and (y>=4.25-clear) and (y <= 5.75+clear)
    r1=(x>=7.25-clear) and (x<=8.75+clear) and (y>=1-clear) and (y <= 4+clear)
    r2=(x>=3.75-clear) and (x<=6.25+clear) and (y>=4.25-clear) and (y <= 5.75+clear)
    if c1 or c2 or sq or r1 or r2:
        obj_val = 0
    else:
        obj_val = 1
   
    return obj_val

# plot curve generic function 
def plot_curve(Xi,Yi,Thetai,UL,UR,ps,pn):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.9
    Xn=Xi
    Yn=Yi
    Thetan = math.radians(Thetai*th_angle)
    while t<2:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += r * (UL + UR) * math.cos(Thetan) * dt
        Yn += r * (UL + UR) * math.sin(Thetan) * dt
        status=border_check(Xn,Yn)
        flag=obstacles(Xn,Yn)
        if (status!=1) or (flag != 1):
            return None
        Thetan += (r / L) * (UR - UL) * dt
        ps.append((Xs,Ys))
        pn.append((Xn,Yn))
    Thetan = math.degrees(Thetan)
    return [Xn, Yn, Thetan]

# continuation of plot curve
def move_bot(Xi,Yi,Thetai,UL,UR,s,n):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.9
    Xn=Xi
    Yn=Yi
    length=0
    Thetan = math.radians(Thetai*th_angle)
    
# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes

    while t<2:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += r * (UL + UR) * math.cos(Thetan) * dt
        Yn += r * (UL + UR) * math.sin(Thetan) * dt
        length+=euclid([Xs,Ys],[Xn,Yn])
        status=border_check(Xn,Yn)
        flag=obstacles(Xn,Yn)
        if (status!=1) or (flag != 1):
            return None
        Thetan += (r / L) * (UR - UL) * dt
        s.append((Xs,Ys))
        n.append((Xn,Yn))
    Thetan = math.degrees(Thetan)
    
    return [Xn, Yn, Thetan,length]


# Threshhold for nodes near goal
def goal_check(pt,g):
            if ((np.square(pt[0]-g[0])) + (np.square(pt[1]-g[1])) <= np.square(0.3)):
                return 0
            else:
                return 1

def path_find(goal,start):  # finding optimal via backtracking
    path.append(goal)
    GN=parent[listToString(goal[0])]
    path.append(GN)
    while (GN[0]!=start):
        GN=parent[listToString(GN[0])]
        path.append(GN)
        path.reverse()
    return path
        
########################################################################################3
#####################################################################################333

# defining threhhold
th=0.1  #0.01
nd=int(1/th)  #100
sizex=10*nd  #1000

th_angle=10  #10
na=int(360/th_angle)  #36
            
clear=0.1 # clearance

# Max RPM values
rpm1=6
rpm2=8

# start position
x_start = .5
y_start = .5
th_ini = 0
# goal position
x_goal = 9
y_goal = 9
th_final = 0

# action set
actions = [[rpm1,0],[0,rpm1],[rpm1,rpm1],[0,rpm2],[rpm2,0],[rpm2,rpm2],[rpm1,rpm2],[rpm2,rpm1]]

start = rnd([x_start,y_start,th_ini])  # rounding 
c_node = start
goal = rnd([x_goal,y_goal,th_final])
g_node = [goal[0],goal[1],goal[2]]

# Cost matrices
cost=np.array(np.ones((sizex,sizex,na)) * np.inf)
visited=np.array(np.zeros((sizex,sizex,na)))
tot_cost=np.array(np.ones((sizex,sizex,na)) * np.inf)

# node lists
parent = {}
que=[]
visited_list = []

################################################################################################
# put start as zero, nd in queue
heappush(que,(0,start))
cost[int(nd*start[0])][int(nd*start[1])][start[2]]=0
tot_cost[int(nd*start[0])][int(nd*start[1])][start[2]]=0

###################################################################################################
##################################################################################################
# main 
flag=0  # check variable for goal_check

s=[]
n=[]
ps=[]
pn=[]
while goal_check(c_node,g_node) == True:
    
    if flag==1:
        break
    
    c_node=heappop(que)[1]
    # if goal_check(c_node,g_node)==0:  # check if goal is reached
    #     goalfound=[c_node]
    #     print(c_node)
    #     break
    
    for action in actions:
        
        curr_node = move_bot(c_node[0], c_node[1], c_node[2], action[0], action[1],s,n)
        if(curr_node == None):
            continue
        
        orient = curr_node[3]
        curr_node=rnd(curr_node[0:3])

        if goal_check(curr_node,g_node)==0:  # check if goal is reached
            parent[listToString(curr_node)]=[c_node,action]
            goalfound=[curr_node,action]
            print(curr_node)
            flag=1
            break
        
        status=border_check(curr_node[0],curr_node[1])
        flag1=obstacles(curr_node[0],curr_node[1])
        if (status and flag1 == 1):  # if current posiiton not in any obstacle space
          if visited[int(nd*curr_node[0]),int(nd*curr_node[1]),curr_node[2]]==0:
            visited[int(nd*curr_node[0]),int(nd*curr_node[1]),curr_node[2]]=1
            visited_list.append([c_node,action,curr_node])
            parent[listToString(curr_node)]=[c_node,action]
            cost[int(nd*curr_node[0]),int(nd*curr_node[1]),curr_node[2]]=orient+cost[int(nd*c_node[0]),int(nd*c_node[1]),c_node[2]]
            tot_cost[int(nd*curr_node[0]),int(nd*curr_node[1]),curr_node[2]]=cost[int(nd*curr_node[0]),int(nd*curr_node[1]),curr_node[2]] + euclid(curr_node, g_node)
            heappush(que,( tot_cost[int(nd*curr_node[0]),int(nd*curr_node[1]),curr_node[2]] ,curr_node ))
            
          else:
             if tot_cost[int(nd*curr_node[0]),int(nd*curr_node[1]),curr_node[2]]>(tot_cost[int(nd*c_node[0]),int(nd*c_node[1]),c_node[2]]+sum(action)):
                tot_cost[int(nd*curr_node[0]),int(nd*curr_node[1]),curr_node[2]]=(tot_cost[int(nd*c_node[0]),int(nd*c_node[1]),c_node[2]]+sum(action))
                visited_list.append([c_node,action,curr_node])
                parent[listToString(curr_node)]=[c_node,action]


#################################################################################################333
###################################################################################################
# optimal path
path=[]
path=path_find(goalfound,start)

sx=x_start
sy=y_start
sz=th_ini

fig, ax = plt.subplots()
ax.set(xlim=(0, 10), ylim=(0, 10))
    
c1 = plt.Circle((2, 2), 1, fill=None)
c2 = plt.Circle((2, 8), 1, fill=None)

currentAxis = plt.gca()
currentAxis.add_patch(Rectangle((3.75, 4.25), 2.5, 1.5, fill=None, alpha=1))
currentAxis.add_patch(Rectangle((7.25, 2), 1.5, 2, fill=None, alpha=1))
currentAxis.add_patch(Rectangle((.25, 4.25), 1.5, 1.5, fill=None, alpha=1))
ax.add_artist(c1)
ax.add_artist(c2)

ax.set_aspect('equal')


l=0
while(l<len(n)):
      ax.plot([s[l][0],n[l][0]],[s[l][1],n[l][1]], color="red",linewidth=0.7)
      l+=1
 
i=len(path)
for action in path:
    if(i>1):
      x1= plot_curve(action[0][0],action[0][1],action[0][2], action[1][0],action[1][1],ps,pn)
 
    i=i-1
    
# saving frames for backtracking
l=0
while(l<len(pn)):
    ax.plot([ps[l][0],pn[l][0]],[ps[l][1],pn[l][1]], color="green")
    # plt.savefig('frame'+str(l)+'.jpg')
    l+=1


# video 
# img=[]
# for i in range(l-1):
#     img.append(cv2.imread('frame'+str(i)+'.jpg'))

# height,width,layers=img[1].shape

# video=cv2.VideoWriter('video.mp4',-1,5,(width,height))

# for j in range(l-1):
#     video.write(img[j])

# cv2.destroyAllWindows()
# video.release()