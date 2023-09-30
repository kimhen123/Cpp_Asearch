#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    
    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node= &this->m_Model.FindClosestNode(start_x,start_y); //find start node and store its address to this->start_node
    this->end_node= &this->m_Model.FindClosestNode(end_x,end_y);    //find end node and store its address to this->end_node

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h_value=0.0f;                        // Declare h_value variable to store manhattan distance to the end node
    h_value=node->distance(*(this->end_node)); // caculate manhattan distance to the end node
    return h_value;                            // return manhattan distance to the end node
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();                                                              // expand current node's neighbors by using the FindNeighbors() method

    for ( RouteModel::Node* Node_Neighbor : current_node->neighbors) {                          // loop through current node's neighbors
    
        Node_Neighbor->h_value=CalculateHValue(Node_Neighbor);                                  // add h_value by using the CalculateHvalue() method
        Node_Neighbor->g_value=current_node->g_value+current_node->distance(*(Node_Neighbor));  // increase g_value by using the distance() method
        Node_Neighbor->parent=current_node;                                                     // set parent node
        this-> open_list.push_back(Node_Neighbor);                                              // add the neighbor to open_list
        Node_Neighbor->visited=true;                                                            // set the node's visited attribute to true
    }
}

bool compare(const RouteModel::Node *first,const RouteModel::Node *second) // compare the sum of the h value and g value of 2 nodes
{
    float sum1=first->g_value+first->h_value;                              // sum1= g_value of node1 + h_value of node1
    float sum2=second->g_value+second->h_value;                            // sum2= g_value of node2 + h_value of node2
    return sum1 > sum2;
}
void Cell_Sort(std::vector<RouteModel::Node*> *openlist) // sort vector of node address descending order
{
    std::sort(openlist->begin(),openlist->end(),compare); 
}
// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() { // Get the next node
    Cell_Sort(&(this->open_list));
    RouteModel::Node* next_node=this->open_list.back();
    this->open_list.pop_back();
    
    return next_node;
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
    std::vector<RouteModel::Node> path_found_tem; // Create  path_found vector temporary

    // TODO: Implement your solution here.
    
    // Get distance from start node to end node
    while(current_node!=this->start_node){ 
        path_found_tem.push_back(*(current_node));
        distance=distance+current_node->distance(*(current_node->parent));
        current_node= current_node->parent;
    }
    // Store all node to path_found
    path_found.push_back(*(this->start_node));
    while(path_found_tem.size()>0){
        path_found.push_back(path_found_tem.back());
        path_found_tem.pop_back();
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
    // add the start node to open list
    this->start_node->h_value=CalculateHValue(this->end_node);
    this->start_node->g_value=0;
    this->start_node->visited=true;
    this->open_list.push_back(this->start_node);
    
    current_node=this->start_node;  // set current node as start node
    


    // TODO: Implement your solution here.
    while(this->open_list.size()>0){
        current_node=NextNode();                            // find the next node
        
        if(current_node==this->end_node)                    // check if finding the end node
        {
            m_Model.path =ConstructFinalPath(current_node);
            break;
        }
        AddNeighbors(current_node);                         // if not finding the end node, expand search
    }
    if(this->open_list.size()==0)                           // if dont found the path
    {
        std::cout << "No path found !";
    }
    
}