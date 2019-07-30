//Uses A* to plan a downwards path for the brush on a given .vtk file
#ifndef PLAN
#define PLAN

#include <iostream>  
#include <set>
#include <vector>
#include "Graph.h"
#include <limits>

using namespace std; 
 

//Input: filename of vtk head model, upper edge threshold, lower edge threshold 
int a_star_path(char* mesh_filename, char* path_filename) 
{ 
    // prints hello world 
    Graph graph(mesh_filename);
    graph.at(1).print();
    cout << graph.size() << endl;  

    // Find starting points, edge points above a certain threshold
    double upper_threshold = graph.y_percent(0.9);   
    vector<int> starts;
    graph.find_edge_points(upper_threshold, 1, starts);

    cout << "Number of start cells: " << starts.size() << endl; 


    /*
    // Find ending points
    double lower_threshold = graph.y_percent(0.1);   
    vector<int> ends;
    graph.find_edge_points(lower_threshold, 0, ends);
    
    // Find pairs
    vector<Cell> start_cells;
    
    for(int i=0; i<starts.size(); i++) {
        start_cells.push_back(graph.at(starts[i])); 
    }

    cout << endl; 
    //export_vtk(start_cells, "starting_points.vtk"); 
    */

    // Goal is simply to reach a bottom threshold
    double goal = graph.y_percent(0.3);
    vector<int> ends; 
    vector<double> path_lengths; 
    vector< vector<int > > paths; 
    
    cout << "Upper threshold: " << upper_threshold << endl; 
    cout << "Lower threshold: " << goal << endl; 

    for(int k=0; k<starts.size(); k++) {
        // Index is always cellID - numberOfPoints
        int start = starts[k];
    
        // Initialize A* helpers
        set<int> open_set;
        open_set.insert(start); 
        set<int> closed_set;
        double g_score[graph.size()];
        double f_score[graph.size()];
        vector<int> came_from(graph.size());

        for(int i=0; i<(sizeof(g_score)/sizeof(*g_score)); i++) {
            g_score[i] = INF; 
            f_score[i] = INF;
        }

        // y-value is the heuristic
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

            // Goal is met...finish
            if(graph.at(current).get_coord().y <= goal) {
                cout << "NUMBER OF STORED PATHS: " << paths.size() << endl; 
                // Has the end cell already been visited?
                vector<int>::iterator endit = find(ends.begin(), ends.end(), current);
                if(endit != ends.end() && paths.size() > 0) { // Already been visited
                    // Is this path shorter? 
                    double path_length = dist_between(graph.at(start), graph.at(current));
                    cout << "Found end: " << current << endl; 
                    cout << "Found path distance at: " << path_lengths[*endit] << endl; 
                    
                    if(path_length < path_lengths[*endit]) { // Update
                        vector<int> path;
                        reconstruct_path(came_from, start, current, path);
                        paths[*endit] = path;
                        path_lengths[*endit] = path_length; 
                        cout << "Updated path " << *endit << " with path length " << path.size() << endl; 
                    }
                }
                else {
                    // Push back new path
                    vector<int> path;
                    reconstruct_path(came_from, start, current, path);

                    paths.push_back(path);
                    ends.push_back(current);
                    path_lengths.push_back(dist_between(graph.at(start), graph.at(current)));
                    
                    cout << "Added end: " << ends.back() << endl; 
                    cout << "Added path distance at: " << path_lengths.back() << endl; 
                } 

                break; 
                 

            }
            else { 
                // Update visited sets
                open_set.erase(current);
                closed_set.insert(current);

                // Find the cell neighbors
                Cell current_cell = graph.at(current); 
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
        cout << endl; 
    }

    // Export all paths
    graph.export_ply_paths(paths, path_filename);
    graph.export_vtk_paths(paths, "vtk_paths");
    graph.export_vtk(paths, "paths.vtk");
//    graph.export_ply(paths[0], argv[2]);
    cout << "exported to " << path_filename << endl; 

    return 0; 
}

#endif
