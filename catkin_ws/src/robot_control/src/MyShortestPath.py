#!/usr/bin/python

import numpy as np
import yaml

def dijkstras(occupancy_map,x_spacing,y_spacing,start,goal):
    """
    Implements Dijkstra's shortest path algorithm
    Input:
    occupancyshhaa_map - an N by M numpy array of boolean values (represented
        as integers 0 and 1) that represents the locations of the obstacles
        in the world
    x_spacing - parameter representing spacing between adjacent columns
    y_spacing - parameter representing spacing between adjacent rows
    start - a 3 by 1 numpy array of (x,y,theta) for the starting position 
    goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position 
    Output: 
    path: list of the indices of the nodes on the shortest path found
        starting with "start" and ending with "end" (each node is in
        metric coordinates)
    """

    INF = 1000000000
    mymap = np.zeros_like(occupancy_map)
    
    nrows = np.shape(mymap)[0]
    ncols = np.shape(mymap)[1]
    #print(ncols,nrows)
    start_indx = int(np.ceil((start[0]/x_spacing)-0.5))        
    start_indy = int(np.ceil((start[1]/y_spacing)-0.5))    
    dest_indx = int(np.ceil((goal[0]/x_spacing)-0.5))
    dest_indy = int(np.ceil((goal[1]/y_spacing)-0.5))    
    #if ((goal[0][0]/x_spacing-0.5) > (ncols-1)):
    #    dest_indx = ncols-1
    #if (goal[1][0]/y_spacing-0.5) > nrows-1:
    #    dest_indy = nrows - 1        

 
    distanceFromStart = np.zeros_like(occupancy_map)
   
    distanceFromStart[:][:] = INF
  
 
    parent = np.zeros_like(occupancy_map)


    mymap[np.where(occupancy_map == 0)] = 1  # Mark free cells
    mymap[np.where(occupancy_map == 1)] = 2   # Mark obstacle cells


    # Generate linear indices of start and dest nodes
    
    #start_node = np.ravel_multi_index([start_indx, start_indy], np.shape(mymap), order='F')
    #sub2ind(size(map), start_coords(1), start_coords(2));
   
    dest_node = np.ravel_multi_index([dest_indy, dest_indx], np.shape(mymap), order='C')
    #sub2ind(size(map), dest_coords(1),  dest_coords(2));

    mymap[start_indy][start_indx] = 5
    mymap[dest_indy][dest_indx]  = 6
  
     
    distanceFromStart[start_indy][start_indx] = 0
    
    # keep track of number of nodes expanded 
    numExpanded = 0
    

    # Main Loop
    while True:
    
        # Find the node with the minimum distance
        min_dist = np.min(distanceFromStart)
  
        current = np.argmin(distanceFromStart)
        # Compute row, column coordinates of current node
        [i,j] = np.unravel_index(current, np.shape(mymap), order='C')


    
        if (current == dest_node or min_dist == INF):
            break
        
        
        # Update map
        mymap[i][j] = 3         # mark current node as visited
      
        distanceFromStart[i][j] = INF; # remove this node from further consideration
        
 
        numExpanded = numExpanded + 1;
        # Visit each neighbor of the current node and update the map, distances
        # and parent tables appropriately.
        
        if (i>0) and (mymap[i-1][j] != 3) and (mymap[i-1][j] != 5) and (mymap[i-1][j] != 2):
            if (min_dist+1 < distanceFromStart[i-1][j]):
                distanceFromStart[i-1][j] = min_dist+1
                mymap[i-1][j] = 4
                parent[i-1][j] = current
      
        if (i<nrows-1) and (mymap[i+1][j] != 3) and (mymap[i+1][j] != 5) and (mymap[i+1][j] != 2):
            if (min_dist+1 < distanceFromStart[i+1][j]):
                distanceFromStart[i+1][j] = min_dist+1;
                mymap[i+1][j]=4
                parent[i+1][j] = current
     
        if (j>0) and (mymap[i][j-1] != 3) and (mymap[i][j-1] != 5) and (mymap[i][j-1] != 2):
            if (min_dist+1 < distanceFromStart[i][j-1]):
                distanceFromStart[i][j-1] = min_dist+1
                mymap[i][j-1]=4
                parent[i][j-1] = current
      
        if (j<ncols-1) and (mymap[i][j+1] != 3) and (mymap[i][j+1] != 5) and (mymap[i][j+1] != 2):
            if (min_dist+1 < distanceFromStart[i][j+1]):
                distanceFromStart[i][j+1] = min_dist+1;
                mymap[i][j+1]=4
                parent[i][j+1] = current
 
        
        
    # Construct route from start to dest by following the parent links
    if (distanceFromStart[dest_indy][dest_indx] == INF):
        routep = np.array([])
    else:
        routep = np.array([[dest_indx,dest_indy]])
    
        [i,j] = np.unravel_index(dest_node, np.shape(mymap))
        while (parent[i][j] != 0):
            [i,j] = np.unravel_index(parent[i][j], np.shape(mymap))
            routep = np.concatenate((routep, np.array([[j,i]])), axis=0)
            #print(i,j)
        #[i,j] = np.unravel_index(start_node, np.shape(mymap))           
        #routep = np.concatenate((routep, np.array([[i,j]])), axis=0)       
  
    s = len(routep)
 
    
    a = start[0][0]
    b = start[1][0]

    route = np.array([[a,b]])

    for k in range(1,s+1):

        [a,b]= (routep[s-k]+0.5)*[x_spacing, y_spacing]
       
        route = np.concatenate((route, np.array([[a,b]])), axis=0)
 
    a = goal[0][0]
    b = goal[1][0]               
    route = np.concatenate((route, np.array([[a,b]])), axis=0)

            
    return route
    #pass

def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.13
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.3], [0]])
    goal1 = np.array([[0.6], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    true_path1 = np.array([
        [ 0.3  ,  0.3  ],
        [ 0.325,  0.3  ],
        [ 0.325,  0.5  ],
        [ 0.325,  0.7  ],
        [ 0.455,  0.7  ],
        [ 0.455,  0.9  ],
        [ 0.585,  0.9  ],
        [ 0.600,  1.0  ]
        ])
    print(path1)
    if np.array_equal(path1,true_path1):
      print("Path 1 passes")

    test_map2 = np.array([
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.5], [1.0], [1.5707963267948966]])
    goal2 = np.array([[1.1], [0.9], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    print(path2)
    true_path2 = np.array([[ 0.5,  1.0],
                           [ 0.5,  1.1],
                           [ 0.5,  1.3],
                           [ 0.5,  1.5],
                           [ 0.7,  1.5],
                           [ 0.9,  1.5],
                           [ 1.1,  1.5],
                           [ 1.1,  1.3],
                           [ 1.1,  1.1],
                           [ 1.1,  0.9]])
    if np.array_equal(path2,true_path2):
      print("Path 2 passes")

def test_for_grader():
    """
    Function that provides the test paths for submission
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 1, 0, 0, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 0, 0, 1, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 1
    y_spacing1 = 1
    start1 = np.array([[1.5], [1.5], [0]])
    goal1 = np.array([[7.5], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    print(path1)
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)


    test_map2 = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.4], [0.4], [1.5707963267948966]])
    goal2 = np.array([[0.4], [1.8], [-1.5707963267948966]])
    
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    print(path2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 2 length:")
    print(s)



def main():
    # Load parameters from yaml
    param_path = 'params.yaml' # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    # Get params we need
    occupancy_map = np.array(params['occupancy_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']
    path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
    print(path)

if __name__ == '__main__':
    main()

