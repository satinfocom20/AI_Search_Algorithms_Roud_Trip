# AI_Search_Algorithms_Roud_Trip
Besides baseball, McDonald's, and reality TV, few things are as canonically American as hopping in the car for an old-fashioned road trip. We've a dataset of major highway segments of the United States(and parts of southern Canada and northern Mexico), including highway names, distances, and speed limits; we can visualize this as a graph with nodes as towns and highway segments as edges. We also have a a dataset of cities and towns with corresponding latitude-longitude positions.

Routing algorithms should be:
- bfs uses breadth-first search (which ignores edge weights in the state graph)
- uniform is uniform cost search (the variant of bfs that takes edge weights into consideration)
- dfs uses depth-first search
- ids uses iterative deepening search
- astar uses A* search, with a suitable heuristic function

cost-function is one of:
- segments tries to find a route with the fewest number of "turns" (i.e. edges of the graph)
- distance tries to find a route with the shortest total distance
- time tries to find the fastest route, for a car that always travels at the speed limit

The output of your program should be a nicely-formatted, human-readable list of directions, including travel times, distances, intermediate cities, and highway names, similar to what Google Maps or another site might produce.
