"""
This file consisits of one main method, optimalRoute. 

optimalRoute returns the optimal route for the vertices to travel from the start to the end with 
the factor of picking up the passenger or not.

Last modified: = "27/4/2023"

"""
__author__ = "Tee Zhi Hui"
__version__ = "3"

import math
class Graph():
    """ 
    This class implementation is referred and inspired from https://www.softwaretestinghelp.com/java-graph-tutorial/.

    This class is used to represent a graph. It is an adjacency list implementation. I decided using adjacent list implemention
    as the specification of task1 mentioned I should not assume |R| = theta (|L|^2), |R| is the number of roads, |L| is the number 
    locations so I can't assume it is a dense graph. Using adjacent list will save space as it only requires O(|V| + |E|) space, whereas
    adjacent matrix requires O(|V|^2) space.
    """
    def __init__(self, passengers, roads , isCarpool = False):
        """
        This is the constructor for the Graph class. It takes in a list of passengers, a list of roads and a boolean value to
        determine if the graph is a carpool graph or not.

        :Input:
            passengers: A list of integers representing the vertices that are passengers
            roads: A list of tuples representing the roads. Each tuple contains 4 elements, the first two elements are the
            vertices that the road connects, the third element is the time taken to travel on the road alone (non carpool lane), 
            the fourth element is the time taken to travel on the road with a passenger (carpool lane).
            isCarpool: A boolean value to determine if the graph is a carpool graph or not. Default is False.
        
        :Return:
            None

        :Time complexity: 
            O(E),where E is the number of edges in graph

        :Aux space complexity:
            O(V), where V is the number of element in graph
        """
        # find the max number of vertices
        max_verteces_count = 0
        for road in roads:
            vertex = road[:2]
            if max(vertex) > max_verteces_count:
                max_verteces_count = max(vertex)

        max_verteces_count = max_verteces_count + 1
        self.vertices = [None] * max_verteces_count

        for i in range (max_verteces_count):
            if isCarpool:
                id = (max_verteces_count-1) + i + 1
                self.vertices[i] = Vertex(id)
            else:
                self.vertices[i] = Vertex(i)

        # update the attribute of the vertex which has passenger 
        for pas_location in passengers:
            self.vertices[pas_location].passenger = True
        
        self.generate_graph(roads,isCarpool)


    def generate_graph(self, roads, isCarpool):
        """
        This method is used to generate the graph. 

        :Input:
            roads: A list of tuples representing the roads. Each tuple contains 4 elements, the first two elements are the
            vertices that the road connects, the third element is the time taken to travel on the road alone (non carpool lane),
            the fourth element is the time taken to travel on the road with a passenger (carpool lane).
            isCarpool: A boolean value to determine if the graph is a carpool graph or not. Default is False.

        :Return:
            None

        :Time complexity: 
            O(E),where E is the number of edges in graph

        :Aux space complexity:
            O(1)
        """
        self.add_edges(roads, isCarpool)

            
    def add_edges (self, argv_edges, isCarpool):
        """
        This method is used to add edges to the graph. It takes in a list of edges and a boolean value to determine if the graph
        is a carpool graph or not.

        :Input:
            argv_edges: A list of tuples representing the roads. Each tuple contains 4 elements, the first two elements are the
            vertices that the road connects, the third element is the time taken to travel on the road alone (non carpool lane),
            the fourth element is the time taken to travel on the road with a passenger (carpool lane).
            isCarpool: A boolean value to determine if the graph is a carpool graph or not. Default is False.
        
        :Return:
            None

        :Time complexity: 
            O(E),where E is the number of edges in graph

        :Aux space complexity:
            O(1)
        """
        for edge in argv_edges:
            u = edge[0] 
            v = edge[1]
            time_alone_lane = edge[2]
            time_carpool_lane = edge[3]

            # if it is a carpool graph, the time taken to travel on the road alone (non carpool lane) should be infinity by default
            # else it is a non car pool grpah, the time taken to travel on the road with someone else (carpool lane) should be infinity
            # by default
            if isCarpool:
                current_edge = Edge(self.vertices[u],self.vertices[v], math.inf, time_carpool_lane)
                current_vertex = self.vertices[u]
                current_vertex.add_edge(current_edge)
            else:
                current_edge = Edge(self.vertices[u],self.vertices[v],time_alone_lane, time_carpool_lane)
                current_vertex = self.vertices[u]
                current_vertex.add_edge(current_edge)

    def __str__ (self):
        """
        This method is used to print the graph.

        :Return:
            return_string: A string that represents the graph.

        :Time complexity: 
            O(V),where V is the number of vertices in graph

        :Aux space complexity:
            O(1)
        """
        return_string = ""
        for vertex in self.vertices:
            return_string = return_string + "Vertex " + str(vertex) + "\n"
        return return_string
    
class Vertex():
    """ 
    This class implementation is referred and inspired from the FIT2004 lecture in Malaysia

    This is the Vertex class. It is used to represent a vertex in the graph. 
    """
    def __init__(self, id):
        """
        This is the constructor for the Vertex class. It takes in an integer to represent the id of the vertex.

        :Input:
            id: An integer to represent the id of the vertex.

        :Return:
            None

        :Time complexity: 
            O(1)

        :Aux space complexity:
            O(1)
        """
        self.id = id
        self.edges = []
        self.discovered = False
        self.visited = False
        self.distance = math.inf
        self.passenger = False  

        # backtracking purpose
        self.previous = None
    
    def add_edge(self, edge):
        """
        This method is used to add an edge to the vertex. It takes in an edge object.

        :Input:
            edge: An edge object to be added to connect the vertex.

        :Return:
            None

        :Time complexity: 
            O(1)

        :Aux space complexity:
            O(1)
        """
        self.edges.append(edge)

    def __str__ (self):
        """ 
        This method is used to print the vertex.

        :Return:
            A string representation of the vertex.

        :Time complexity: 
            O(E),where E is the number of edges in graph

        :Aux space complexity:
            O(1)
        """
        return_string_id = str(self.id)
        return_string = return_string_id
        for edge in self.edges:
            if self.previous != None:
                return_string += "\n with edges " + str(edge) + ", disctance: " + str(self.distance)  + ", previous: " + str(self.previous.id) 
            else:
                return_string += "\n with edges " + str(edge) + ", disctance: " + str(self.distance) 
        return return_string

    def added_to_heap(self):
        """ 
        This method is used to mark the vertex as discovered.

        :Return:
            None

        :Time complexity: 
            O(1)

        :Aux space complexity:
            O(1)
        """
        self.discovered = True

    def visited_node(self):
        """ 
        This method is used to mark the vertex as visited. If the vertex is visited means the distance of 
        the vertex is finalised.

        :Return:
            None

        :Time complexity: 
            O(1)

        :Aux space complexity:
            O(1)
        """
        self.visited = True

class Edge():
    """ 
    This class implementation is referred and inspired from https://www.softwaretestinghelp.com/java-graph-tutorial/.

    This is the Edge class. It is used to represent a edge between the vertices in the graph.     
    """
    def __init__(self, u, v, time_alone_lane = math.inf, time_carpool_lane = math.inf):
        """
        This is the constructor for the Edge class. It takes in two vertices and two integers to represent the time taken to travel
        on the road alone and with a passenger. u -> v, both are vertex object

        :Input:
            u: A vertex object to represent the starting vertex of the edge.
            v: A vertex object to represent the ending vertex of the edge.
            time_alone_lane: An integer to represent the time taken to travel on the road alone (non carpool lane). 
                             Default is infinity.
            time_carpool_lane: An integer to represent the time taken to travel on the road with a passenger (carpool lane). 
                               Default is infinity.

        :Return:
            None

        :Time complexity: 
            O(1)

        :Aux space complexity:
            O(1)
        """
        self.u = u
        self.v = v
        self.time_alone_lane = time_alone_lane
        self.time_carpool_lane = time_carpool_lane

    def __str__(self):
        """ 
        This method is used to print the edge.

        :Return:
            A string representation of the edge.

        :Time complexity: 
            O(1)

        :Aux space complexity:
            O(1)
        """
        if self.time_alone_lane != math.inf:
            return str(self.u.id) + ", " + str(self.v.id) + ", alone: " + str(self.time_alone_lane)
        elif self.time_carpool_lane != math.inf:
            return str(self.u.id) + ", " + str(self.v.id) + ", carpool: " + str(self.time_carpool_lane)



class MinHeap():
    """ 
    This class implementation is referred from the FIT2004 course note written by Daniel Anderson (pg33, 34)
    and inspired by the tutorial of week 6.

    Class used to create a MinHeap for the dijkstra algorithm. MinHeap is a data structure which the value of 
    each node is smaller than or equal to its children. This means that the smallest element in the heap is always 
    stored at the root node. It is a fixed heap based on the number of the vertex in the graph,
    the heap is initialised with the tuple of (vertex id, distance and previous vertex id). The heap is used to keep track of and
    update the distance of the vertex that is not visited. 

    array is a list containing all the vertices of the graph
    map_array is a index mapping list. E.g. If I want to know where is vertex 3 in my array list, I can obtain it by doing map_array[3].
    The purpose of having this map_array list is to avoid the cost of tranversing the list. 

    The speciality of this MinHeap is it doesn't have a insert method, as I mentioned it is a fix-size heap based on the number of vertices
    therefore, update method is provided to update any information of any vertex. 
    """
    def __init__(self, graph):
        """
        This method is used to initialise the MinHeap. 
        The MinHeap is initialised with the tuple of (vertex id, distance and previous vertex id).

        :Input:
            graph: a graph object 

        :Return:
            None

        :Time complexity: 
            O(V),where V is the number of element in graph

        :Aux space complexity:
            O(V), where V is the number of element in graph
        """
        self.vertices_count = len(graph.vertices)
        self.array = [None] * (self.vertices_count+1) 
        self.map_array = [None] * self.vertices_count

        # initialise the array with the tuple of (vertex id, distance and previous vertex id)
        # consider dinstance and previous vertex id are unknown to us 
        for i in range (len(self.array)):
            self.array[i] = (graph.vertices[i-1].id, math.inf, math.inf)
            self.map_array[i-1] = i # want to obtain vertex 3 do array[map_array[3]]

        # the first element is set to None to comply rule of obtaining the parent vertex by k // 2, child vertex by 2*k 
        # k is the current index of the node 
        self.array[0] = None
        self.length = len(self.array)

    def serve(self):
        """
        Remove the top element from the heap

        :Return:
            The top element of the heap, a vertex object

        :Time complexity: 
            O(Log V),where V is the number of element in MinHeap

        :Aux space complexity:
            O(1)
        """
        max_element = self.array[1]
        self.swap(1,(self.length-1))
        self.length -= 1
        self.sink(1)
        return max_element[0]

    def rise (self, k):
        """ 
        Move an element up the heap

        :Input:
            k: the index of the node which I wish to rise the position 

        :Return:
            None

        :Time complexity: 
            Best: O(1)
            Worst: O(Log V),where V is the number of element in graph
        
        :Aux space complexity:
            O(1)
        """
        if self.length == 1:
            return 
        parent = k // 2 
        while parent >= 1:
            # ensure the child is smaller than parent 
            if self.array[parent][1] > self.array[k][1]:
                self.swap(parent,k)
                new_k = parent
                self.rise(new_k)
            else:
                break

    def sink (self, k):
        """ 
        Move an element down the heap

        :Input:
            k: the index of the node which I wish to sink the position 

        :Return:
            None

        :Time complexity: 
            Best: O(1)
            Worst: O(Log V),where V is the number of element in graph
        
        :Aux space complexity:
            O(1)
        """
        child = 2 * k 
        while child <= (self.length-1):
            # ensure my child is bigger than left child
            if child < self.length -1  and self.array[child][1] > self.array[child+1][1]:
                child += 1
            # if child is bigger than me ,than swap
            if self.array[k][1] >= self.array[child][1]:
                self.swap(k,child)
                k = child
                child = 2 * k
            else:
                break

    def swap(self, x, y):
        """ 
        Swap two elements in the heap

        :Input:
            x, y: the index of the node which I wish to swap their position 

        :Return:
            None

        :Time complexity: 
            O(1)

        :Aux space complexity:
            O(1)
        """
        # obtain the vertex id as map_array need vertex id to obtain the value
        vertex_x_id = self.array[x][0]
        if y == 0:
            y = 2
        vertex_y_id = self.array[y][0]
        
        self.array[x], self.array[y] = self.array[y], self.array[x]
        self.map_array[vertex_x_id], self.map_array[vertex_y_id] = self.map_array[vertex_y_id], self.map_array[vertex_x_id]

    def update(self, vertex, distance = math.inf, parent = None):
        """ 
        Update the information/value of the node in the heap

        :Input:
            vertex: the vertex object which I wish to update 
            distance: the distance from the source to the vertex
            parent: the previous vertex of the vertex, default is None

        :Return:
            None

        :Time complexity: 
            Best: O(1)
            Worst: O(Log V),where V is the number of element in MinHeap

        :Aux space complexity:
            O(1)
        """
        array_idx = self.map_array[vertex.id] 

        # when the array length <  array_idx, it is the last vertex to update(insert) so doesn't have to do anything
        if array_idx >= self.length:
            return 

        if parent == None:
            parent_id = math.inf
        else:
            parent_id = parent.id
     
        self.array[array_idx] = (vertex.id, distance, parent_id)

        # after updating the information of the vertex in the heap, I ensure it is in the correvt position
        # by rising it 
        self.rise(array_idx)

    def __len__ (self):
        """ 
        This is a magic method to return the length of the heap

        :Return:
            the length of the heap

        :Time complexity: 
            O(1)

        :Aux space complexity:
            O(1)
        """
        return self.length

def dijkstra(source, destination, passengers, graph):
    """
    This implementation is referred and inspired from the FIT2004 course note written by Daniel Anderson (pg73 to pg77).

    Function description:
        This function is used to find the shortest path from source to destination with the given passengers. 
    
    Approach description:
        I will always start my dijkstra from the non-carpool lane, and along the way I go 
        (edge relaxation) , if the vertex (location) has a passenger, I will prioritise those vertex and visit it first 
        to check will the time it took me to the destination be shorter. I prioritise them by setting the distance to 0, therefore 
        if the vertex is in the fixed minHeap, it will always serve the vertex with passenger first. We want to do that as we know the 
        time we spend to drive from source to destination on carpool lane will always be less or equal to the time we have to spend 
        on non carpool lane. Therefore, if there is a chance to drive on carpool lane we must experiment it. After visited all the vertex, 
        I can backtrack my path by using the previous attribute of the destination (vertex). 

    :Input:
        source: a vertex object 
        destination: a vertex object 
        passengers: a list of passenger of locations
        graph: a graph object, run this algorithm on this graph object

    :Return:
        optimal_route, a list containing vertex id which is the optimal route from source to destination

    :Time complexity: 
        Best = Worst: O(E log V), where E is the number of edges, V is the number of vertices 

    :Aux space complexity: 
        O(V+E), where V is the number of vertices and E is the number of edges
    """
    optimal_route = []
    source.distance = 0 
    discovered = MinHeap(graph) 
    discovered.update(source, 0)    

    while len(discovered) > 0:     
        # serve the vertex with minimum distance from heap
        u = discovered.serve() 
        u = graph.vertices[u]
        u.visited_node()    # if u is visited, then the distance of u is finalised
        
        # perform edge relaxation on all adjacent vertices
        for edge in u.edges: 
            v = edge.v 

            # if the vertex hasn't been discovered, the disctance is still infinity 
            if v.discovered == False:            
                v.added_to_heap()             
                # decide which lane
                # update distance from infinity to a new num
                if edge.time_carpool_lane == math.inf:
                    v.distance = u.distance + edge.time_alone_lane 
                else:
                    v.distance = u.distance + edge.time_carpool_lane
                v.previous = u
                discovered.update(v, v.distance, u)

            # it is in heap but not yet finalised 
            elif v.visited == False: 
                # decide which lane
                if edge.time_carpool_lane == math.inf:
                    latest_shortest_distance = u.distance + edge.time_alone_lane  
                else:
                    latest_shortest_distance = u.distance + edge.time_carpool_lane

                # if i find a shorter distance to get to this vertex, i will update the distance
                if v.distance > latest_shortest_distance:
                    v.distance = latest_shortest_distance
                    v.previous = u
                    discovered.update(v, v.distance, u)
                
        # this allow dijkstra to terminate earlier
        if u == destination:
            break

    # check whether the dijkstra end at carpool lane or non carpool lane
    if graph.vertices[destination.id+1+(len(graph.vertices)//2)-1].discovered == True and len(passengers)!=0:
        destination_carpool = graph.vertices[destination.id+1+(len(graph.vertices)//2)-1]
        destination_non_carpool = graph.vertices[destination.id]
        if destination_non_carpool.distance <= destination_carpool.distance:
            destination = destination_non_carpool
        else:
            destination = destination_carpool

    # backtracking
    visited_start_count = 1
    while destination != source and destination != None and visited_start_count <= 2:
        if len(passengers) != 0:
            # the destination is at carpool lane
            if destination.id >= len(graph.vertices)//2 :
                non_carpool_destination = graph.vertices[destination.id - len(graph.vertices)//2]
                # when the previous id of the destination is back to non carpool lane, skip it, to avoid duplication
                if non_carpool_destination.id == destination.previous.id:
                    optimal_route.append(non_carpool_destination.id)
                    destination = destination.previous.previous
                else:
                    optimal_route.append(non_carpool_destination.id)
                    destination = destination.previous
            # # the destination is at non carpool lane   
            else:
                optimal_route.append(destination.id)
                destination = destination.previous
                
        #if there's no passengers
        else:
            optimal_route.append(destination.id)
            destination = destination.previous

    if optimal_route[-1] != source.id:   
        optimal_route.append(source.id)

    # reverse the list of optimal_route
    for i in range(len(optimal_route) // 2):
        optimal_route[i], optimal_route[-i - 1] = optimal_route[-i - 1], optimal_route[i]

    return optimal_route

def optimalRoute(start, end, passengers, roads):
    """
    Function description:
        This function is used to find the optimal route from start to end with the given passengers. It will return a list of 
        vertex id which is the most optimal route from start to end with the decision whether to pick up the passenger.

    Approach description:
        As there's two lane (carpool lane and non-carpool lane). I merge this two lane into one graph before running dijkstra get 
        the shortest path. By doing this, we can diffirentiate which lane we are currently at. We differentiate these two lane by the id 
        of the vertex if the id of the vertex is less than len(graph.vertices) // 2, then the vertex is at the non_carpool lane else
        it is at the carpool lane. The id of the vertex of non carpool lane should always be smaller to be easier to differentiate them, 
        as driver must start at non carpool lane first.
        If there is no passenger in the graph, there's no need to merge the graph, we just have to run dijkstra to find
        the most optimal path as we can only drive on the non carpool lane.

    :Input:
        start: a number (vertex.id) indicate the starting point
        end: a number (vertex.id) indicate the ending point
        passengers: a list of passenger of locations
        roads: a list of road of locations

    :Return:
        optimal_vertex : a list of vertex id which is the most optimal route from start to end with the decision 
        whether to pick up the passenger

    :Time complexity: 
        Best = Worst: O(E log V), where E is the number of edges, V is the number of vertices 

    :Aux space complexity: 
         O(V+E), where V is the number of vertices and E is the number of edges
    """
    non_carpool_graph = Graph(passengers,roads, False)

    # if there's no passenger, run dijkstra 
    if len(passengers) == 0:
        return dijkstra(non_carpool_graph.vertices[start], non_carpool_graph.vertices[end],passengers, non_carpool_graph)

    carpool_graph = Graph(passengers, roads, True)

    # merge the graph of non carpool lane and the graph of carpool lane
    graph = merge_graph(passengers,non_carpool_graph, carpool_graph) 
   
    optimal_vertex = dijkstra(graph.vertices[start], graph.vertices[end],passengers, graph)
  
    return optimal_vertex

def merge_graph(passengers, non_carpool_graph, carpool_graph):
    """ 
        This function is used to merge the two graph into one graph.

        :Input:
            passengers: a list of passenger of locations
            non_carpool_graph: a graph of non carpool lane
            carpool_graph: a graph of carpool lane

        :Return:
            merge_graph: a graph of merge graph (non_carpool_graph and carpool_graph)

        :Time complexity: 
            Best = Worst: O(V+E), where V is the number of vertices, E is the number of edges

        :Aux space complexity:
            O(V), where V is the number of vertices
    """
    roads = []
    # ensure all the edges are included before merging them
    for vertex in non_carpool_graph.vertices:
        for edge in vertex.edges:
            roads.append((vertex.id, edge.v.id, edge.time_alone_lane, math.inf))
    for vertex in carpool_graph.vertices:
        for edge in vertex.edges:
            roads.append((vertex.id, edge.v.id, math.inf, edge.time_carpool_lane))
 
    merge_graph = Graph(passengers, roads, False)

    # add the edges from the carpool lane to the non carpool lane, when there's a passenger
    # therefore both graph is connected
    for i in passengers:
        location_num = i
        merge_graph.add_edges([(i,location_num+1+len(carpool_graph.vertices)-1,0,0)],False)

    return merge_graph

if __name__ == "__main__" :
    start = 0
    end = 4
    # The locations where there are potential passengers
    passengers = [2, 1]
    # The roads 
    roads = [(0, 3, 5, 3), (3, 4, 35, 15), (3, 2, 2, 2), (4, 0, 15, 10), (2, 4, 30, 25), (2, 0, 2, 2), (0, 1, 10, 10), (1, 4, 30, 20)]
    # The locations where there are potential passengers
    passengers = [2, 1]
    the_route = optimalRoute(start,end,passengers,roads)
    print("The most optimal route:", the_route)
