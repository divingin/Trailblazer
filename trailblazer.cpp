/*Candice Murray and Karen Munyan
 * CS 106B Spring 2015
 * Homework 7: Trailblazer
 * Sources: Lecture notes, section 7 handout, Stanford C++ library documentation
 *
 * This file contains the functions implementing the search algorithms for Depth
 * First Search, Breadth First Search, Dijkstra's Algorithm, A* search and the function
 * for making minimal spanning trees to generate mazes using Kruskal's Algorithm.
 */

#include "trailblazer.h"
#include "pqueue.h"
#include "queue.h"


using namespace std;

//Function prototypes
Vector<Vertex*> depthFirstSearchHelper(BasicGraph& graph, Vertex* first, Vertex* end, Vector<Vertex*>& path, bool& foundPath);
Vector<Vertex*> dijkstraAstarHelper(BasicGraph& graph, Vertex* start, Vertex* end, int isAstar);


//Functions
Vector<Vertex*> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    /*Implements the depth first search algorithm by calling a helper function.
     * Accepts a graph to be searched, and a starting and ending point to find the
     * path between. If start and end are the same returns a path with only that point.
     * If a path does not exist returns an empty path.
     */
    graph.resetData();
    Vector<Vertex*> path;
    bool foundPath = false; //bool that gets changed when the end of the path has been reached, stopping the search.
    depthFirstSearchHelper(graph,start,end,path,foundPath);
    if (path[path.size()-1] != end){ //there is no path that connects the two points
        path.clear();
        return path;
    }
    return path;
}

Vector<Vertex*> depthFirstSearchHelper(BasicGraph& graph, Vertex* first, Vertex* end, Vector<Vertex*>& path, bool& foundPath){
    /*Helper function implementing the main search for DFS recursively.  Accepts same parameters
     * as depthFirstSearch, as well as a vector containing the path and a boolean that indicates
     * if a path has been found yet.
     */
    path += first;
    first->visited = true;
    first->setColor(GREEN);
    if (first == end){
        foundPath = true;
        return path; //found the path!
    }else {
        bool neighborsNotAllVisited = false; //bool to check if all of the neighbors have been visited
        for (Vertex* neighbor:graph.getNeighbors(first)){
            if (!neighbor->visited){ //choose
                neighborsNotAllVisited = true;
                depthFirstSearchHelper(graph,neighbor,end,path,foundPath);//explore
                if (foundPath){
                    return path;
                } else{//unchoose
                    path.remove(path.size()-1);//take the neighbor out of the path
                    neighbor->setColor(GRAY);
                }
            }
        }
        if (!neighborsNotAllVisited || graph.getNeighbors(first).isEmpty()){
            //if all of the neighbors of the vertex have been visited or there are no neighbors
            return path;
        }
    }
    return path;
}


Vector<Vertex*> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    /* Implements the Breadth First Search method, which finds the path with the shortest length.
     * Accepts the graph to be searched and the starting and ending points of the desired path.
     * If there is no path connecting the starting and ending points, returns an empty
     * vector. If the start and end are the same, returns a vector containing only the
     * start point.
     */
    graph.resetData(); //clears out any old data
    Vector<Vertex*> path;

    Queue<Vertex*> q;
    q.enqueue(start);
    start -> visited = true; //marks the starting point as visited
    start -> setColor(GREEN);

    if(start == end) //deals with the case where start = end
    {
        path += start;
        return path;
    }

    while(!q.isEmpty())
    {
       Vertex* v = q.dequeue(); //dequeues a vertex v from the queue
       v -> setColor(GREEN);
       if(v == end) //if v is the end point, the shortest length path has been found
       {
           Vertex* temp = v;
           Stack<Vertex*> s;
           s.push(end); //adds the ending point to the reverse path stack
           while(temp -> previous != start) //reconstructs the reverse path in a stack by following the previous back to the start
           {
               s.push(temp -> previous);
               temp = temp -> previous;
           }
           s.push(start); //adds the start to the stack

           while(!s.isEmpty()) //Adds the stack components to the path
           {
               path += s.pop();
           }
           return path;
       }

       for (Vertex* neighbor:graph.getNeighbors(v)) //Goes through each unvisited neighbor of v
       {
            if(neighbor -> visited == false)
            {
                neighbor -> visited = true; //marks the neighbor as visited

                //set n's previous to be v
                neighbor -> previous = v;
                q.enqueue(neighbor); //enqueues the neighbor in the queue
                neighbor -> setColor(YELLOW);
            }
       }
    }

    //if we get here, no path exists
    return path; //returns an empty vector
}


Vector<Vertex*> dijkstraAstarHelper(BasicGraph& graph, Vertex* start, Vertex* end, int isAstar){
   /*Does most of the work for both Dijkstra's Algorithm and A*, since the two algorithms are very
    * similar.  Accepts the graph and start and end points from the actual algorightm call, as well
    * as an int, isAstar, which is 1 if we want to run the A* search algorithm or 0 if we want to
    * call Dijkstra's algorightm.  This int is multiplied by the heuristic function parameters so that
    * they are part of the calculations when running A* but not part of the calculations when running
    * Dijkstra's algorithm. For details of output for each method see comments on dijkstrasAlgorithm
    * and aStar.
    */
   Vector<Vertex*> path;

   if (start == end){
       path += start;
       return path;
   }
   for(Vertex* node:graph){
       node->resetData(); //clear out any old data
       node->cost = POSITIVE_INFINITY; //initialize costs to infinity
   }
   start->cost = 0;
   PriorityQueue<Vertex*> queue;
   queue.enqueue(start, isAstar*heuristicFunction(start, end));
   start->setColor(YELLOW);
   while (!queue.isEmpty()){
       Vertex* current = queue.dequeue();
       current->visited = true;
       current->setColor(GREEN);
       if (current != end){ //if we haven't reached the end yet, cycle through the neighbors
           for (Vertex* neighbor:graph.getNeighbors(current)){
               if (!neighbor->visited){ //if the neighbor hasn't been visited, check the cost
                   double cost = current->cost + graph.getEdge(current,neighbor)->cost;
                   if (cost < neighbor->cost){ //if this path costs less than before, enqueue it or change priority
                       neighbor->cost = cost;
                       neighbor->previous = current;
                       if (neighbor->getColor() == YELLOW){//if the neighbor has been enqueued already
                           queue.changePriority(neighbor, cost + isAstar*heuristicFunction(neighbor, end));
                       } else{ //if the neighbor hasn't been enqueued yet
                           queue.enqueue(neighbor, cost + isAstar*heuristicFunction(neighbor, end));
                           neighbor->setColor(YELLOW);
                       }

                   }
               }
           }
       } else{ //reached the end, stop looking
           break;
       }
   }
   if (end->previous == NULL){ //there is no path that connects the two points
       return path; //return an empty vector
   }
   Vertex* place = end;
   path.insert(0,place);
   while (place != start){ //make the path by following previouses
       place = place->previous;
       path.insert(0,place);
   }
   return path;
}

Vector<Vertex*> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {
    /*Implements Dijkstra's Algorithm, which finds the path of least cost.  Accepts
     * the graph to be searched and the starting and ending points of the desired path.
     * If there is no path connecting the starting and ending points, returns an empty
     * vector. If the start and end are the same, returns a vector containing only the
     * start point.
     */
    return dijkstraAstarHelper(graph,start,end,0);
}


Vector<Vertex*> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {
    /*Implements the A* Algorithm, which uses heuristics to find-tune the
     * order of elements in its priority queue to explore the points that
     * are more likely to be the best first. It is a variation of
     * Dijkstra's Algorithm that finds the path of least cost. It accepts
     * the graph to be searched and the starting and ending points of the desired path.
     * If there is no path connecting the starting and ending points, it returns an empty
     * vector. If the start and end are the same, it returns a vector containing only the
     * start point.
     */
    return dijkstraAstarHelper(graph,start,end,1);
}

Set<Edge*> kruskal(BasicGraph& graph) {
    /*Implements Kruskal's algorithm for building minimum spanning trees, which connects
     * all verticies with minimal number of edges.  Accepts the starting graph for which
     * the minimum spanning tree is to be built.
     */
    Set<Vertex*> vertexSet = graph.getVertexSet();
    Set<Set<Vertex*> > clusters;
    for (Vertex* vertex:vertexSet){ //loop through all vertexes to put them in individual clusters
        Set<Vertex*> newCluster;
        newCluster.add(vertex);
        clusters.add(newCluster);
    }
    Set<Edge*> edgeSet = graph.getEdgeSet();
    PriorityQueue<Edge*> queue;
    for(Edge* edge : edgeSet){
        queue.enqueue(edge, edge->cost);
    }
    Set<Edge*> mst;
    while (clusters.size() > 1){ //while there are still at least 2 separate clusters
        Edge* currentEdge = queue.dequeue();
        Set<Vertex*> set1;
        Set<Vertex*> set2;
        for (Set<Vertex*> subCluster : clusters) {
            if (subCluster.contains(currentEdge->start)){ //find the cluster containing the start of the edge
                set1= subCluster;
            }else if (subCluster.contains(currentEdge->finish)){//find the cluster containing the end of the edge
                set2 = subCluster;
            }
        }
        if (set1 != set2 && (!set1.isEmpty() && !set2.isEmpty())){ //if the ends of the edge aren't in the same set, combine them
            clusters.remove(set1);
            clusters.remove(set2);
            set1 += set2;
            clusters.add(set1);
            mst += currentEdge;
        }
    }
    return mst;
}
