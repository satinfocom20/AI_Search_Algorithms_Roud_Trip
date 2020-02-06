#!/usr/bin/env python
'''
(1) Abstraction of the problem
State space: List of all US cities, towns and highways
Successor function: Fetches all the successor cities of current city
Goal state: A path between the origin and destination cities
Vertices: Cities/towns/highways
Edges: Distance / Time
Heuristic function:
Distance:- h(s) = Haversine distance between the current and the goal city/vertex in graph
Time:- h(s) = (Haversine distance between the current and the goal city/vertex in graph) / average speed limit of all roads in US
The haversine calculates the distance from the current city to destination city on earth surface in miles which is a good estimation to know how far we need to go, however we have observed that for some cities the haversine distance is more than the actual road distance so we have multiplied the haversine distance by 0.2 so it will be always lesser than the actual distance to the reach goal, hence the heuristic function is admissible as the heuristic cost is always lesser than the actual cost.

(2) How the search works
Depends on the inputs from the user, the program executes the corressponding function for the routing algorithm and cost function.

BFS: It works with First In First Out concept so we use 'Queue' to always pop the 0th element so the one entered first in the queue will be expanded first. Since we need to find the path of the goal insted of the just the goal state, we need to store the list of path in the queue once all the nodes at a level is proessed breadth wise and pop the one inserted first, this process runs in loop until the queue is empty. We also don't skip the algorithm to just find the 1st goal, instead we expand all nodes to find all possbile paths and store it in a list

DFS: It works with Last In First out concept so we use 'Stack' to always pop the one entered last to be expanded first. Rest of the logic to store the list of paths remains as same as BFS

IDS: It is same as DFS but it uses the advantages of both BFS and DFS, for every iteration it explores the nodes till the depth that we pass. The depth will be increasing one at a time, which means the program explores all nodes at depth 1 to find the goal, if no goal found then it will loop over again from depth 1 to 2 again and so on until it finds the goal. As it uses DFS in iterative fashion, we use 'Stack' to store fringe as we used in DFS

UCS: This works like a BFS but it decides the nodes to be expanded based on the cost we use, in our case it will be either distance / time (segment is not applicable here). We use 'PrioityQueue' which helps to pop an element with least cost(distance / time) and the rest of the logic is same as BFS to store the path

Astar: This works like UCS but it has heuristic function additionally. We calculate the cost till the current node from start node as its done in UCS and add the cost from heuristic which is the haversine distance between the current city and the goal city, if the cost is time we divide the haversine distance by the average speed limit for roads in US from the given inputs

(3) Discussion on problems, assumptions & design decisions
BFS: It doesn't consider the segment, distance and time. It expands the nodes breadwise one level at a time so it is generating the path if one exists with shortest number of segments by its nature. As it doesn't consider the distance and time, we can't use the cost function to find the optimal path based on distance or time, but we designed the logic to return all paths for the goal and find the one with shortest segment, distance and time depends on the user input. It gives best route if the numbers for the shorter number of node traversals, however it takes more time and huge number of memory for cities which are far apart as it has to traverse several nodes

DFS: It doesn't consider the segment, distance and time. It doesn't provide an optimal path as it goes to infinite loop sometimes if it picks a wrong path to search as it keeps exploring a branch and miss the correct branch to be explored. We have run tests for random cities and sometimes the results are very quick, better than BFS but most of the times it took more than 5 mins so we had to stop the program without finding the result.

IDS: It works as same like DFS but it explores only nodes in certain number of depths for an iteration, this helps with the processig time as it doesn't need to explore the other nodes deep in the graph if it finds the goal within fewer iterations, but if the goal is in the leaf or somewhere near the leaf then the proceissing time is very high as it has to repeat the process for every iteration until it reaches the goal node. As said in DFS for some cities the result was generated as fast as DFS but there were some cities it didn't generate the result and we had to cancel the program after 5 mins.

UCS: It considers the distance/time as cost function as it uses either distance or time at any state to pick the child nodes with least distance/time to be expanded. It runs faster than BCS, DFS and IDS as it doesn;t need to explore many nodes as the other alogrithms. As it always expands based on the distance/time it always finds the optimal path for distance and time as cost. It is observed that UCS takes lesser space than BFS for shorter distance but both are same for the longer distance or cities which are far apart

Astar: It considers distance/time as cost function as it uses either distance or time at any state to pick the child nodes with least distance/time inaddition to heuristic cost. It runs faster than UCS as the number of nodes it traverse is lesser than UCS.  It is observed that UCS takes lesser space than BFS and UCS for shorter distance but all are same for the longer distance or cities which are far apart.

Assumptions or improvements:
* There are missing speed limit which I have replaced with default value 30 as the local roads has minimum of 30 miles per hour limit so I am using the least speed limit as possbile
* The highways (JCT*) are not the actual cities and they are missing in the gps file so I calculated the average US road distance with the city road distance given in the input file
* The heuristic function we have may not be consistent as we are not able to find the distance for the highways (JCT*)
* We multiply the haversine distance calculated for heuristic by 0.2 to make sure it is not overestimated
* We have performed around 50+ runs with different cities for all algorithms and compared with google maps and we got close results for 60% of times from BFS, Astar and Uniform.
Astar and Uniform cost depends on the distance and gps co-ordinates given in the file so we belive there may be some variabtion in this data between our dataset and google. Also there
are several highways (JCT) given as cities in the road-segments which are not found in the gps file so our heuristic function was not able to get the good estimate as we subsitute the
default values for the highways(JCT)

References:
1. Course materials, videos in canvas
2. https://pythoninwonderland.wordpress.com/2017/03/18/how-to-implement-breadth-first-search-in-python/
3. https://gist.github.com/rochacbruno/2883505
'''


import sys
from collections import defaultdict
from pprint import pprint
from collections import deque
from Queue import PriorityQueue
import math


class USmap:
    #Intialize the dictionary to store cities, distance, speed limit and latitude and longitude
    def __init__(self):
        self.edges = defaultdict(list)
        self.weights_dist = defaultdict(list)
        self.weights_rate = defaultdict(list)
        self.highway = defaultdict(list)
        self.gpslat = defaultdict(list)
        self.gpslon = defaultdict(list)

    #Load the city connectivity as un directed graph using dictionary by reading the data from txt file
    def load_graph(self):
        file = open('road-segments.txt', 'r')
        lines = file.readlines()
        for x in lines:
            strsplit = x.split(' ')
            self.edges[strsplit[0]].append(strsplit[1])
            self.edges[strsplit[1]].append(strsplit[0])
            self.weights_dist[strsplit[0] + strsplit[1]] = int(strsplit[2])
            self.weights_dist[strsplit[1] + strsplit[0]] = int(strsplit[2])
            if strsplit[3] != '':
                self.weights_rate[strsplit[0] + strsplit[1]] = int(strsplit[3])
                self.weights_rate[strsplit[1] + strsplit[0]] = int(strsplit[3])
            else:
                self.weights_rate[strsplit[0] + strsplit[1]] = 55
                self.weights_rate[strsplit[1] + strsplit[0]] = 55
            self.highway[strsplit[0] + strsplit[1]] = strsplit[4]
            self.highway[strsplit[1] + strsplit[0]] = strsplit[4]
        file.close()

    #Load the city gps using dictionary by reading the data from txt file
    def load_gps(self):
        file = open('city-gps.txt', 'r')
        lines = file.readlines()

        for x in lines:
            strsplit = x.split(' ')
            self.gpslat[strsplit[0]] = float(strsplit[1])
            self.gpslon[strsplit[0]] = float(strsplit[2])
        file.close()

    #Successor function to get all neighbors of any node/city in the graph
    def neighbors(self, node):
        return self.edges[node]

    #Calculate the distance or time for the cost to be used in uniform cost search and A*
    def get_cost(self, from_node, to_node, timedist):
        f = str(from_node)
        t = str(to_node)
        r = f+t
        if timedist == 'distance':
            return self.weights_dist[str(r)]
        elif timedist == 'time':
            dist = self.weights_dist[str(r)]
            rate = self.weights_rate[str(r)]
            if rate == 0:
                rate = 30
            time = float(dist) / rate
            return time
        else:
            return 0

    #Display the loaded graph with city connectivity and the distace cost in dictionary
    def show_graph(self):
        print("Destinations:")
        pprint(dict(self.edges))
        print("Cost:")
        pprint(dict(self.weights_dist))

    #Display the latitude and longitude details of the cities in dictionary
    def show_gps(self):
        print("Latitude:")
        pprint(dict(self.gpslat))
        print("Longitude:")
        pprint(dict(self.gpslon))

    #Calculate average speed limit of US roads with given speed limits from the input. This can be used in heuristic
    #function calculate time as it uses distance and average speed limit

    def avgspeedlimit(self):
        totalspeed = 0
        for city, speed in self.weights_rate.items():
            if speed == '0' or speed == '':
                totalspeed = totalspeed + 30
            else:
                totalspeed = totalspeed + int(speed)
        avgspeed = float(totalspeed) / len(self.weights_rate)
        return avgspeed

    # Calculate average distance of US roads with given distance of all roads from the input. This can be used in
    # heuristic function to if the distance is 0 for any city or the city is not present in gps file
    def averageroaddist(self):
        totaldistance = 0
        for city, distance in self.weights_dist.items():
            if distance == '0' or distance == '':
                totaldistance = totaldistance + 30
            else:
                totaldistance = totaldistance + int(distance)
        avgdistance = float(totaldistance) / len(self.weights_dist)
        return avgdistance

    #Heuristic function for A*star. If the cost is distance it calculates the distance between 2 cities gps coordinates
    #using haversine formula. The claculated distance is multiplied by 0.2 to make sure this function is not
    #overestimating the distance as we noticed for distance between some cities are overestimated. If the cost is time,
    #it divides the calculated distance by average speed limit
    def heuristic(self, origin, destination, timedist, avgdist, avgspdlmt):
        if self.gpslat[origin] and self.gpslon[origin] and self.gpslat[destination] and self.gpslon[destination]\
            and (str(origin[0:3] != 'Jct')):
            lat1 = self.gpslat[origin]
            lon1 = self.gpslon[origin]
            lat2 = self.gpslat[destination]
            lon2 = self.gpslon[destination]

            # radius = 6371 # km
            radius = 3959  # mile

            dlat = math.radians(lat2 - lat1)
            dlon = math.radians(lon2 - lon1)
            a = math.sin(dlat / 2) * math.sin(dlat / 2) + math.cos(math.radians(lat1)) \
                                                         * math.cos(math.radians(lat2)) * math.sin(dlon / 2) * math.sin(
                dlon / 2)
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            d = radius * c
        else:
             d = avgdist

        if timedist == 'distance':
            return float(d * 0.2)
        elif timedist == 'time':
            return float((d * 0.2)) / avgspdlmt

    #Finds the route to destination city with shortest number of segments or turns
    def cost_segment(self, allpaths):
        allpaths.sort(key=len)
        return allpaths[0]

    # Finds the route to destination city with shortest distance
    def cost_distance(self, allpaths):
        distlist = []
        for p in allpaths:
            dist = 0
            i = 0
            while (i < (len(allpaths[0])-1)):
                dist = dist + self.weights_dist[p[i]+p[i+1]]
                i = i + 1
            distlist.append(dist)
        minidx = distlist.index(min(distlist))
        return distlist[minidx], allpaths[minidx]

    # Finds the route to destination city with shortest time
    def cost_time(self, allpaths):
        all_time = []
        for p in allpaths:
            i = 0
            time = 0
            while(i < (len(p)-1)):
                dist = self.weights_dist[p[i]+p[i+1]]
                rate = self.weights_rate[p[i]+p[i+1]]
                if rate == 0:
                    rate = rate + 30
                time = time + float(dist)/rate
                i = i + 1
            all_time.append(time)
        minidx = all_time.index(min(all_time))
        return round(all_time[minidx], 2), allpaths[minidx]

    #Implement BFS
    def bfs(self, start, goal):
        # keep track of explored nodes
        explored = []
        # keep track of all the paths to be checked
        queue = [[start]]
        # keep track of all the paths to the goal
        pathbfs_goal = []
        # return path if start is goal
        if start == goal:
            return "That was easy! Start = goal"

        # keeps looping until all possible paths have been checked
        while queue:
            # pop the first path from the queue
            path = queue.pop(0)
            # get the last node from the path
            node = path[-1]
            if node not in explored:
                neighbours = self.neighbors(node)
                # go through all neighbour nodes, construct a new path and
                # push it into the queue
                for neighbour in neighbours:
                    new_path = list(path)
                    new_path.append(neighbour)
                    queue.append(new_path)
                    # return path if neighbour is goal
                    if neighbour == goal:
                        pathbfs_goal.append(new_path)

                explored.append(node)
        return pathbfs_goal

    # Implement UCS - This is same as BFS but it calculates the cost at each node to decide the next node to expand
    def ucs(self, start, goal, timedist):
        visited = set()
        pathucs_goal = []
        #Priority queue helps to pop an item with least cost (distance / time)
        queue = PriorityQueue()
        queue.put((0, start))
        while queue:
            cost, path = queue.get()
            if cost != 0:
                node = path[-1]
            else:
                node = path
            if node not in visited:
                visited.add(node)

                if node == goal:
                    pathucs_goal.append(list(path))
                    return [list(path)]

                for i in self.neighbors(node):
                    if i not in visited:
                        total_cost = cost + self.get_cost(node, i, timedist)
                        if cost == 0:
                            new_path_ucs = []
                            new_path_ucs.append(path)
                            path_queue = tuple(new_path_ucs)
                        else:
                            path_queue = tuple(path)
                        queue.put(( total_cost, ( path_queue+(i,) ) ))
        return pathucs_goal


    #Implement DFS
    def dfs(self, start, goal):
        visited = set()
        stack = [[start]]
        pathbdfs_goal = []
        while stack:
            pathd = stack.pop()
            noded = pathd[-1]
            if noded not in visited:
                visited.add(noded)

                if noded == goal:
                    pathbdfs_goal.append(new_pathd)
                for neighbor in self.neighbors(noded):
                    if neighbor not in visited:
                        new_pathd = list(pathd)
                        new_pathd.append(neighbor)
                        stack.append(new_pathd)
        return pathbdfs_goal

    #Implement IDS - This uses the advantages of BFS and DFS. It is designed to execute infinte number of times until
    #it finds the goal
    def ids(self, start, goal):
        d = 1
        while d > 0:
            visited = set()
            pathids_goal = []
            stack = [[start]]
            it = 1
            while stack:
                pathd = stack.pop()
                noded = pathd[-1]
                if noded not in visited:
                    visited.add(noded)

                    if noded == goal:
                        pathids_goal.append(pathd)
                        return [pathd]

                    if it <= d:
                        for neighbor in self.neighbors(noded):
                            if neighbor not in visited:
                                new_pathd = list(pathd)
                                new_pathd.append(neighbor)
                                stack.append(new_pathd)
                    it+=1
            d = d + 1

        return pathids_goal

    #Implement A*. It uses heuristic function to calculate the estimated cost from any node to goal
    def astar(self, start, goal, timedist):
        visited = set()
        queue = PriorityQueue()
        queue.put((0, start))
        avgdist = self.averageroaddist()
        avgspdlmt = self.avgspeedlimit()
        while queue:
            cost, path = queue.get()
            if cost != 0:
                node = path[-1]
            else:
                node = path
            if node not in visited:
                visited.add(node)

                if node == goal:
                    return [list(path)]
                for i in self.neighbors(node):
                    if i not in visited:
                        total_cost = cost + self.get_cost(node, i, timedist) + \
                                     self.heuristic(i, goal, timedist, avgdist, avgspdlmt)
                        if cost == 0:
                            new_path_ucs = []
                            new_path_ucs.append(path)
                            path_queue = tuple(new_path_ucs)
                        else:
                            path_queue = tuple(path)
                    queue.put(( total_cost, ( path_queue+(i,) ) ))

    def printroute(self, distance, rt_dist, time, rt_time, finalroute):
        print ('Distance in miles for the route: ', str(distance))
        print ('Time in hours to reach destination: ', str(time))
        print ('Start from ', rt_dist[0])
        print (' ')
        for r in range(0, (len(rt_dist) - 1)):
            h =  str(self.highway[rt_dist[r] + rt_dist[r + 1]])
            s = 'Drive to ' +  rt_dist[r + 1] + ' for ' + \
                str(self.weights_dist[rt_dist[r] + rt_dist[r + 1]]) + ' miles in ' + h
            print (s)
        print ('Your destination is reached')
        print (' ')
        print(optimal + ' ' + str(distance) + ' ' + str(time) + finalroute)

#Call the USmap class
f = USmap()

#Call function to generate a graph with city connectivity
f.load_graph()

#Call function to load cities gps
f.load_gps()

#Below statements can be used to get inputs in IDE
'''startcity = input("Enter the start city: ")
endcity = input("Enter the end city: ")
routingalg = input("Choice of routing algorightm: ")
costfunc = input("Choice of cost function: ") '''

#Receive [Start city] [End city] [Routing algorightm] [Cost function] as input through console
startcity = sys.argv[1]
endcity = sys.argv[2]
routingalg = sys.argv[3]
costfunc = sys.argv[4]

distance = 0
time = 0
rt_dist = []
rt_time = []
#Based on the inputs call the functions to find the route between origin and destination
if routingalg == 'bfs':

    route = f.bfs(startcity, endcity)

    if costfunc == 'segment':
        segment = f.cost_segment(route)
        distance, rt_dist = f.cost_distance([segment])
        time, rt_time = f.cost_time([segment])
        optimal = 'yes'
    elif costfunc == 'distance' or costfunc == 'time':
        distance, rt_dist = f.cost_distance(route)
        time, rt_time = f.cost_time(route)
        optimal = 'no'
    else:
        print("Invalid cost function. Program will not run:")

    finalroute = ' '
    formatroute = ' '

    for r in rt_dist:
        finalroute = finalroute + r + ' '

    for r in rt_dist:
        formatroute = formatroute + r + ' '


    f.printroute(distance, rt_dist, time, rt_time, formatroute)


elif routingalg == 'uniform':
    if costfunc == 'segment':
        optimal = 'no'
    elif costfunc == 'distance':
        optimal = 'yes'
    elif costfunc == 'time':
        optimal = 'yes'
    else:
        optimal = 'Invalid cost function. Program will not run'

    if costfunc == 'segment':
        costfunc = 'distance'
    route = f.ucs(startcity, endcity, costfunc)
    distance, rt_dist = f.cost_distance(route)
    time, rt_time = f.cost_time(route)

    finalroute = ' '
    formatroute = ' '
    for r in rt_dist:
        finalroute = finalroute + r + ' '

    for r in rt_dist:
        formatroute = formatroute + r + ' '

    f.printroute(distance, rt_dist, time, rt_time, formatroute)


elif routingalg == 'dfs':
    if costfunc == 'segment':
        route = f.dfs(startcity, endcity)
        segment = f.cost_segment(route)
        distance, rt_dist = f.cost_distance([segment])
        time, rt_time = f.cost_time([segment])
    elif costfunc == 'distance' or costfunc == 'time':
        route = f.dfs(startcity, endcity)
        distance, rt_dist = f.cost_distance(route)
        time, rt_time = f.cost_time(route)
    else:
        optimal = 'Invalid cost function. Program will not run'
    optimal = 'no'
    finalroute = ' '
    formatroute = ' '
    for r in rt_dist:
        finalroute = finalroute + r + ' '

    for r in rt_dist:
        formatroute = formatroute + r + ' '

    f.printroute(distance, rt_dist, time, rt_time, formatroute)

elif routingalg == 'ids':
    route = f.ids(startcity, endcity)
    segment = f.cost_segment(route)
    distance, rt_dist = f.cost_distance(route)
    time, rt_time = f.cost_time(route)
    finalroute = ' '
    formatroute = ' '
    for r in rt_dist:
        finalroute = finalroute + r + ' '
    if costfunc == 'segment' or costfunc == 'distance' or costfunc == 'time':
        optimal = 'no'
    else:
        optimal = 'Invalid cost function. Program will not run'

    for r in rt_dist:
        formatroute = formatroute + r + ' '

    f.printroute(distance, rt_dist, time, rt_time, formatroute)

elif routingalg == 'astar':
    if costfunc == 'segment':
        optimal = 'no'
    elif costfunc == 'distance':
        optimal = 'yes'
    elif costfunc == 'time':
        optimal = 'yes'
    else:
        optimal = 'Invalid cost function. Program will not run'

    if costfunc == 'segment':
        costfunc = 'distance'
    route = f.astar(startcity, endcity, costfunc)
    distance, rt_dist = f.cost_distance(route)
    time, rt_time = f.cost_time(route)

    finalroute = ' '
    formatroute = ' '
    for r in rt_dist:
        finalroute = finalroute + r + ' '

    for r in rt_dist:
        formatroute = formatroute + r + ' '

    f.printroute(distance, rt_dist, time, rt_time, formatroute)

else:
    print('Invalid routing algorithm')





