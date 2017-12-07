# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost):
    # open list elements are of type: [g,x,y] where g is the g-value. It has to come first because
    # we will be sorting the list later to take the one with the lowest g.
    
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1

    x = init[0]
    y = init[1]
    g = 0
    
    open = [[g,x,y]]
    
    found = False # flag that is set when search is complete
    resign = False # flag set if we can't expand
    path = []
    while found is False and resign is False:
        # debug
        #print 'open:'
        #for i in range(len(open)):
        #    print ' ', open[i]
        #print '-----'

        # if we can't expand anymore (no more elements in the open list), give up
        if len(open) == 0:
            resign = True
            print 'Fail'
        else:
            # remove node from list
            open.sort() # sort by g-value
            open.reverse() # so that we can pop which takes the last element 
            next = open.pop()
            
            x = next[1]
            y = next[2]
            g = next[0]
            
            # are we done?
            if x == goal[0] and y == goal[1]:
                found = True
                print next #this is it. no more work to be done
            else:
                #expand winning element and add to new open list
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]): # check we are within the grid
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0: # cell not checked and not occupied (blocked)
                            g2 = g + cost
                            open.append([g2,x2,y2])
                            closed[x2][y2] = 1

    return path
    
search(grid,init,goal,cost)
