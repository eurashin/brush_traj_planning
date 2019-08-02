//Uses A* to plan a downwards path for the brush on a given .vtk file
#ifndef PLAN
#define PLAN

#include <iostream>  
#include <set>
#include <vector>
#include "Graph.h"
#include <limits>

using namespace std; 

class AStar {
    private: 
        // Index is always cellID - numberOfPoints
        int start;
        set<int> open_set;
        set<int> closed_set;
        vector<double> g_score; 
        vector<double> f_score;
        vector<int> came_from; 
        double threshold_goal; 
        int cell_goal;
        Graph graph; 

        // Treshold based goal
        bool goal_is_met(Cell current_cell, double goal) {
            return(current_cell.get_coord().y <= goal);    
        }

        // Cell to cell goal
        bool goal_is_met(Cell current_cell, Cell end_cell) {
            return(current_cell.equals(end_cell));
        }

        // Calculate the downwards heuristic for a*
        double h_downwards(Cell cell, Cell goal) {
            Point start_center = cell.get_coord(); 
            Point goal_center = goal.get_coord(); 
            return(abs(goal_center.y - start_center.y));  
        }

        // Calculate the downwards heuristic for a*
        double h_downwards(Cell cell, double threshold) {
            Point start_center = cell.get_coord(); 
            return(abs(threshold - start_center.y));  
        }

        double dist(Point center1, Point center2) {
            return(sqrt(pow(center1.x - center2.x, 2) + pow(center1.y - center2.y, 2) + pow(center1.z - center2.z, 2)));
        }

               
     public:
        AStar(Graph graph, int start) {
            // Initialize A* helpers
            open_set.insert(start); 
            g_score.reserve(graph.size());
            f_score.reserve(graph.size());
            came_from.reserve(graph.size());
            this->start = start;  
            this->graph = graph; 

            // Initialize scoring matrix
            for(int i=0; i<graph.size(); i++) {
                g_score.push_back(INF); 
                f_score.push_back(INF);
            }
            cout << "DONE PUSHING!" << endl; 
        }

        double dist_between(Cell cell1, Cell cell2) {
            Point center1 = cell1.get_coord(); 
            Point center2 = cell2.get_coord(); 

            return(dist(center1, center2));
        }

        void reconstruct_path(int current, vector<int> &path) {
            int daddy = came_from[current]; // Where did the current node descend from?
            
            while(daddy != start) { // While we don't reach the beginning
                path.push_back(daddy);
                daddy = came_from[daddy];
            }

            // Finally, append the start node
            path.push_back(start);
        }
        
        // Returns the index of the end cell
        int run(double goal) {
            cout << f_score.size() << "  " << g_score.size() << endl; 
            // For the first value, the score is completely the heuristic
            f_score[start] = h_downwards(graph.at(start), goal); 
            g_score[start] = 0; 
            int current = start;

            cout << "could access start elements" << endl;  
            while(open_set.size() > 0) {
                // Find cell in open set with lowest f score
                set<int>::iterator it = open_set.begin(); 
                double min_f = f_score[*it];
                current = *it; 
                for(it = open_set.begin(); it != open_set.end(); it++) {
                    if(f_score[*it] < min_f) {
                        current = *it; 
                    } 
                }
                Cell current_cell = graph.at(current); 

                // Goal is met...finish
                if(goal_is_met(current_cell, goal)) {
                    return(current); 
                }
                else { 
                    // Update visited sets
                    open_set.erase(current);
                    closed_set.insert(current);

                    // Find the cell neighbors
                    vector<int> neighbors = current_cell.get_neighbors();       
                
                    for(int i=0; i<neighbors.size(); i++) {
                        int neighbor_index = neighbors[i]; 
                        Cell neighbor_cell = graph.at(neighbor_index);
                        set<int>::iterator find_it_closed = closed_set.find(neighbor_index);

                        // Skip if in closed set
                        if(find_it_closed == closed_set.end()) {
                            // The distance from start to neighbor
                            double tentative_g_score = g_score[current] + dist_between(current_cell, neighbor_cell);

                            set<int>::iterator find_it_open = open_set.find(neighbor_index); 
                            if(find_it_open == open_set.end()) { // Not in open set
                                open_set.insert(neighbor_index);
                            }
                            
                            if(tentative_g_score < g_score[neighbor_index]) { // Found shorter path
                                came_from[neighbor_index] = current; 
                                g_score[neighbor_index] = tentative_g_score; 
                                f_score[neighbor_index] = g_score[neighbor_index] + h_downwards(neighbor_cell, goal);  
                            }
                        }
                    }
                }
            }
            cerr << "No path found!" << endl; 
            return(-1); // No path found 
        } 
        
        int run(Cell goal) {
            // For the first value, the score is completely the heuristic
            f_score[start] = h_downwards(graph.at(start), goal); 
            g_score[start] = 0; 
            int current = start;

            while(open_set.size() > 0) {
                // Find cell in open set with lowest f score
                set<int>::iterator it = open_set.begin(); 
                double min_f = f_score[*it];
                current = *it; 
                for(it = open_set.begin(); it != open_set.end(); it++) {
                    if(f_score[*it] < min_f) {
                        current = *it; 
                    } 
                }
                Cell current_cell = graph.at(current); 

                // Goal is met...finish
                if(goal_is_met(current_cell, goal)) {
                    return(current); 
                }
                else { 
                    // Update visited sets
                    open_set.erase(current);
                    closed_set.insert(current);

                    // Find the cell neighbors
                    vector<int> neighbors = current_cell.get_neighbors();       
                
                    for(int i=0; i<neighbors.size(); i++) {
                        int neighbor_index = neighbors[i]; 
                        Cell neighbor_cell = graph.at(neighbor_index);
                        set<int>::iterator find_it_closed = closed_set.find(neighbor_index);

                        // Skip if in closed set
                        if(find_it_closed == closed_set.end()) {
                            // The distance from start to neighbor
                            double tentative_g_score = g_score[current] + dist_between(current_cell, neighbor_cell);

                            set<int>::iterator find_it_open = open_set.find(neighbor_index); 
                            if(find_it_open == open_set.end()) { // Not in open set
                                open_set.insert(neighbor_index);
                            }
                            
                            if(tentative_g_score < g_score[neighbor_index]) { // Found shorter path
                                came_from[neighbor_index] = current; 
                                g_score[neighbor_index] = tentative_g_score; 
                                f_score[neighbor_index] = g_score[neighbor_index] + h_downwards(neighbor_cell, goal);  
                            }
                        }
                    }
                }
            }

            cerr << "No path found!" << endl; 
            return(-1); // No path found 
        } 
};


    //Input: filename of vtk head model, upper edge threshold, lower edge threshold 
    int find_paths(char* mesh_filename, char* path_filename) 
    { 
        // Read graph 
        Graph graph(mesh_filename);
        cout << graph.size() << endl; 

        // Find starting points, edge points above a certain threshold
        double upper_threshold = graph.y_percent(0.9);   
        vector<int> starts;
        graph.find_edge_points(upper_threshold, 1, starts);

        cout << "Number of start cells: " << starts.size() << endl; 

        // Goal is simply to reach a bottom threshold
        double goal = graph.y_percent(0.3);
        vector<int> ends; 
        vector<double> path_lengths; 
        vector< vector<int > > paths; 

        for(int k=0; k<starts.size(); k++) {
            // Index is always cellID - numberOfPoints
            int start = starts[k]; 

            // Run A*
            AStar a_star(graph, start); 
            cout << "Running for start " << start << endl;  
            int final_cell = a_star.run(goal);

            // Determing whether or not to save path
            if(final_cell != -1) {
                // Has the end cell already been visited?
                vector<int>::iterator endit = find(ends.begin(), ends.end(), final_cell);
                double path_length = a_star.dist_between(graph.at(start), graph.at(final_cell));
                if(endit != ends.end() && paths.size() > 0) { // Already been visited
                    // Is this path shorter? 
                    if(path_length < path_lengths[*endit]) { // Update
                        vector<int> path;
                        a_star.reconstruct_path(final_cell, path);
                        paths[*endit] = path;
                        path_lengths[*endit] = path_length; 
                    }
                }
                else {
                    // Push back new path
                    vector<int> path;
                    a_star.reconstruct_path(final_cell, path);

                    paths.push_back(path);
                    ends.push_back(final_cell);
                    path_lengths.push_back(path_length);
                } 
            }
        }

        // Export all paths
        graph.export_ply_paths(paths, path_filename);
        graph.export_vtk_paths(paths, "vtk_paths");
        graph.export_vtk(paths, "paths.vtk");
        cout << "exported to " << path_filename << endl; 
            cout << endl; 

        return 0; 
    }

#endif
