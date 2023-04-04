#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes. Since the inputs are coordinates, not Nodes, you must 	   find the closest nodes to start and end at with FindClosestNode.
  	
  
  	// See route_planner.h... m_Model is an attribute... see https://learn.udacity.com/nanodegrees/nd213/parts/cd0423/lessons/85d80c4a-ecbb-4469-b513-c6a25f36b9de/concepts/455e502c-38fa-4797-8cea-3cbc55f86956 for the notation above for "initializer list"
  
  // start_node and end_node are declared as pointers in route_planner.h
  // Thus in order to assign a value to the object they are pointing to, instead of the address, you have to dereference them with * in front.
	*start_node = m_Model.FindClosestNode(start_x, start_y);
  	*end_node = m_Model.FindClosestNode(end_x, end_y);
  
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  
  // input argument *node is a pointer to the address of the node variable. Need to dereference it to access the object it's pointing to.
  
  return node->distance(*end_node); 
  

}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  
  // FindNeighbors() adds the neighbors to the current nodes neighbors attribute
  current_node -> FindNeighbors();
  
  //neighbors are type: std::vector<Node *> neighbors
  // each element of the vector is a pointer to a node
  for (RouteModel::Node* neighbor: current_node->neighbors){
    // parent is a pointer to current node
    neighbor->parent = current_node;
    neighbor->g_value = current_node->g_value + 1;
    neighbor->h_value = CalculateHValue(neighbor);
    
    // open_list private attribute of RoutePlanner class: std::vector<RouteModel::Node*> open_list
    open_list.push_back(neighbor);
    neighbor->visited = true;
  }
  

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
  
  //std::vector<RouteModel::Node*> open_list
  std::sort(open_list.begin(), open_list.end(), [](const auto & lhs, const auto & rhs) {
      return lhs->get_f_val() < rhs->get_f_val();
   });
  
  // pointer to node with lowest f_val
  RouteModel::Node* node_lowest_f = open_list.front();
  
  // remove from open_list
  open_list.erase(open_list.begin());
  
  return node_lowest_f;

}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
	
  	//RouteModel::Node curr_node = current_node;
  
    // TODO: Implement your solution here.
  	while (current_node->parent != nullptr){
      
      	//add distance
      	distance += current_node->distance(*(*current_node).parent);
          
      	//append the node to the path vector
      	path_found.insert(path_found.begin(), *(*current_node).parent);
      
      	current_node = (*current_node).parent;
          
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
	

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
  
  	// add neighbors of current node
  	RoutePlanner::AddNeighbors(current_node);
  
  	// get the next node
  	RouteModel::Node *next_node = NextNode();
  
  	// loop until we reach the end node
  	while(next_node->distance(*RoutePlanner::end_node) != 0 ){
    
      // add neighbors to next node
      RoutePlanner::AddNeighbors(next_node);
      
      // get the following node
      RouteModel::Node *next_node = NextNode();
    
    }
  
  	// get the final path
  	std::vector<RouteModel::Node> final_path = RoutePlanner::ConstructFinalPath(current_node);
  
  	// store the final path
  	m_Model.path  = final_path;
  	
  

}