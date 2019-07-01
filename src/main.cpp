#include <iostream>  
#include <set>
#include <vector>
#include "Graph.h"
#include <limits>

using namespace std; 
 

//Input: filename of vtk head model, upper edge threshold, lower edge threshold 
int main(int argc, char** argv) 
{ 
    // prints hello world 
    cout<<"Hello World";
    Graph graph(argv[1]);
    cout << "I finished" << endl;
    cout << graph.size() << endl;  
    // Find starting points, edge points above a certain threshold
    double upper_threshold = graph.y_percent(0.8);   
    vector<int> starts; 
    graph.find_edge_points(upper_threshold, 1, starts);

    // Save the start cells
    cout << "START CELLS: " << endl; 
    vector<Cell> start_cells; 
    for(int i=0; i<starts.size(); i++) {
        cout << starts[i] << "  ";
        //start_cells.push_back(graph.at(starts[i])); 
    }
    cout << endl; 
    //export_vtk(start_cells, "starting_points.vtk"); 

    // Goal is simply to reach a bottom threshold
    double goal = graph.y_percent(0.1);
    cout << "THRESHOLD: " << goal << endl; 
    
    vector< vector<int > > paths; 

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
        f_score[start] = h_downwards(graph.at(start), graph.at(goal)); 
        g_score[start] = 0; 
        int current = start;

        while(open_set.size() > 0 && graph.at(current).get_coord().y > goal) {
            // Find cell in open set with lowest f score
            set<int>::iterator it = open_set.begin(); 
            double min_f = f_score[*it];
            current = *it; 
            for(it = open_set.begin(); it != open_set.end(); it++) {
                if(f_score[*it] < min_f) {
                    current = *it; 
                } 
            }

            /*
            cout << "Visiting node: " << current << endl; 
            cout << "Open set has: " << open_set.size() << endl; 
            cout << "Closed set has: " << closed_set.size() << endl; 
            cout << "Current coord: " << graph.at(current).get_coord().y << endl; 
            cout << "Goal: " << goal << endl << endl; 
            */

            // Goal is met...finish
            if(graph.at(current).get_coord().y <= goal) {
                // Find the path from start to goal
                vector<int> path;
                reconstruct_path(came_from, start, current, path);
                paths.push_back(path);
                cout << "Final path: " << endl; 
                for(int i=0; i<path.size(); i++) {
                    cout << path[i] << "  "; 
                }
                cout << endl; 
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
                            f_score[neighbor_index] = g_score[neighbor_index] + h_downwards(neighbor_cell, graph.at(goal));  
                        }
                    }
                }
            }
        }
    }

    // Export all paths
    graph.export_vtk(paths, "paths.vtk");
    graph.export_ply(paths[0], argv[2]);
    cout << "DIUD I DO IT" << endl; 

    // Export paths as point clouds with normals
    return 0; 
} 
