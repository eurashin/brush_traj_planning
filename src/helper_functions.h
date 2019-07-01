#ifndef HELPER_FUNCTIONS
#define HELPER_FUNCTIONS

#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std; 

// Creates a point cloud to export to VTK for visualization
void export_vtk(vector<Cell> cells, char* filename) {
    ofstream myfile; 
    myfile.open(filename);

    // Set up headers
    myfile << "# vtk DataFile Version 3.0" << endl; 
    myfile << "vtk output" << endl; 
    myfile << "ASCII" << endl; 
    myfile << "DATASET POLYDATA" << endl; 
    myfile << "POINTS " << cells.size() << " float " << endl; 


    for(int i=0; i<cells.size(); i++) {
        Point point = cells[i].get_coord(); 
        myfile << point.x << " " << point.y << " " << point.z << endl; 
    }

    myfile << "LINES " << cells.size() - 1 << " " << (cells.size() -1) * 3 << endl; 
    for(int i=0; i<cells.size() - 1; i++) {
        myfile << "2 " << i << " " << i+1 << endl; 
    }
 
    myfile.close();
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

double dist_between(Cell cell1, Cell cell2) {
    Point center1 = cell1.get_coord(); 
    Point center2 = cell2.get_coord(); 

    return(dist(center1, center2));
}

void reconstruct_path(const vector<int>& came_from, const int start, int current, vector<int> &path) {
    int daddy = came_from[current]; // Where did the current node descend from?
    
    while(daddy != start) { // While we don't reach the beginning
        path.push_back(daddy);
        daddy = came_from[daddy];
    }

    // Finally, append the start node
    path.push_back(start);
}

#endif
