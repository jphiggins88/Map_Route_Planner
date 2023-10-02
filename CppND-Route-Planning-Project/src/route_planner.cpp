#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    std::cout << "RoutePlanner constructor\n";
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    std::cout << "CalculateHValue\n";
    //return (*node).distance(*end_node);    // Distance from the start node to the end node?
    return node->distance(*end_node);    // Distance from the start node to the end node?
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    /* --MY OLD CODE --
    current_node->FindNeighbors();
    //(*current_node).FindNeighbors();

    // Loop through current node's potential neighbors.
    //??? Use RouteModel::Node or Model::Node ? - use RouteModel maybe since CalculateHValue takes in a pointer to a RouteModel::Node
    for (RouteModel::Node * node : current_node->neighbors) {
        // set parent
        // each node we loop though, we set the parent to the current node.
        node->parent = current_node;

        node->g_value += node->distance(*start_node);
        node->h_value = CalculateHValue(node);

        // add neighbor to open list and set node's visited attribute to true.
        open_list.push_back(node);

        node->visited = true;
    }
    */

    current_node->FindNeighbors();
    float current_g = current_node->g_value;
    current_node->visited = true;

    for (int i = 0; i < current_node->neighbors.size(); i++) {
        RouteModel::Node *neighbor = current_node->neighbors[i];
        neighbor->parent = current_node;
        neighbor->g_value = current_g + neighbor->distance(*current_node);  // Can't just do += , it results in a segmentation fault ...?
        neighbor->h_value = CalculateHValue(neighbor);
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::cout << "NextNode\n";
    std::sort(open_list.begin(), open_list.end(), RoutePlanner::CompareSumOfGandH);
    // The vector of Node pointers should now be sorted according to their g+h values (ascending order).
    /* -- MY OLD CODE --
    // return the pointer to the node with the smallest g+h value.(should be the 1st node pointer in the vector)
    //??? Or return the last?
    return open_list.at(0);
    */
    RouteModel::Node* next = open_list.back();

    return next;
}

// Utility function to compare g+h values of 2 nodes. Will be used for sorting the array of node pointers.
bool RoutePlanner::CompareSumOfGandH(RouteModel::Node* a, RouteModel::Node* b) {
    std::cout << "CompareSumOfGandH\n";
    //return (a->g_value + a->h_value)<(b->g_value + b->h_value);
    return (a->g_value + a->h_value)>(b->g_value + b->h_value); // Sorts in descending order
    //??? Why not use < instead and just grab the beginning of the vector?
}


// Complete the ConstructFinalPath method to return the final path found from your A* search.
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    std::cout << "ConstuctFinalPath\n";
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    /* ----MY OLD CODE --
    // TODO: Implement your solution here.
    // ??? could posssibly do while (node->parent != start_node)
    //??? while (current_node->parent != nullptr)
    bool startNodeFound = false;
    while (!startNodeFound)
    {
        path_found.push_back(*current_node);
        //??? Can't just compare nodes. They won't be equal. Must compare coordinates.
        //if(current_node == start_node){
        if(current_node->x == start_node->x && current_node->y == start_node->y){
            // Stop searching once start_node is found and added to the path_found vector.
            startNodeFound = true;     // Not needed with the break below. 
                break;
        }
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    */
    while(current_node->x != this->start_node->x && current_node->y != this->start_node->y){
        
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    
    // We've added nodes to the path_found vector from finish to start. We need to reverse the order.
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// Write the A* Search algorithm here.
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    std::cout << "AStarSearch\n";
    RouteModel::Node *current_node = nullptr;
    open_list.push_back(start_node);

    /*
    //AddNeighbors(current_node);
    AddNeighbors(start_node);

    while(open_list.size() > 0)
    {
        std::cout << "AStarSearch - while\n";
        //??? what is the status of open_list at the beginning of this?
        //??? Isn't open_list empy until FindNeighbor is called. (AddNeighbors calls FindNeighbors)

        current_node = NextNode();
        open_list.pop_back();

        // If goal is found, call construct final path
        //??? check by comparing coordinates or check by seeing if the nodes are equal?
        //??? current_node == end_node ? - this won't work because they are not the same node even if they contain the same values.
        if (current_node->x == end_node->x && current_node->y == end_node->y) {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }

        // Calls Find Neighbors of current node and populates current_node->neighbors vector.
        AddNeighbors(current_node);
    }
    */
    while(this->open_list.size() > 0) {

        current_node = this->NextNode();
        this->open_list.pop_back();

        if (current_node->x == this->end_node->x && current_node->y == this->end_node->y) {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }

        AddNeighbors(current_node);           
    }

}