// Graph representing the VTK triangular mesh
#ifndef GRAPH
#define GRAPH

#include "Cell.h"
#include <vector>
#include <set>
#include <algorithm>
#include <vtkPolyData.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <fstream>
#include "helper_functions.h"

using namespace std; 

const int POINTS_PER_CELL = 3; 
const int DIMENSION = 3; 
const double INF = 100000; 

class Graph {
    private: 
        vector<Cell> cells;
        double min_y, max_y; 


        bool is_edge_neighbor(int index1, int index2) {
            vector<int> points1 = cells[index1].get_point_indices(); 
            vector<int> points2 = cells[index2].get_point_indices(); 

            // Neighbors if two points are shared    
            int matches = 0; 
            for(int i=0; i<points1.size(); i++) {
                vector<int>::iterator it = std::find(points2.begin(), points2.end(), points1[i]);
                if(it != points2.end()) {
                    matches++;
                }    
            }
            
            return(matches == 2);
        }

        void initialize_cell_neighbors(int cell_index) {
             // Search downwards and upwards from index
             int increaser = cell_index + 1; 
             int decreaser = cell_index - 1; 
             int neighbors_found = 0; 
             while(increaser < cells.size() && decreaser > 0 && neighbors_found < 3) {
                 if(is_edge_neighbor(cell_index, increaser)) {
                    cells[cell_index].add_neighbor(increaser); 
                    neighbors_found++; 
                 }
                 
                 if(is_edge_neighbor(cell_index, decreaser)) {
                    cells[cell_index].add_neighbor(decreaser); 
                    neighbors_found++; 
                 }
                
                 increaser++; 
                 decreaser--;                 
             }

             if(neighbors_found < 3) { //Haven't found all neighbors yet
                 if(decreaser > 0) { // Need to cover downwards ground
                    while(decreaser > 0  && neighbors_found < 3) {
                        if(is_edge_neighbor(cell_index, decreaser)) {
                            cells[cell_index].add_neighbor(decreaser); 
                            neighbors_found++; 
                        }
                        decreaser--; 
                    }

                 }
                 else { // Need to cover upwards ground
                    while(increaser < cells.size() && neighbors_found < 3) {
                        if(is_edge_neighbor(cell_index, increaser)) {
                            cells[cell_index].add_neighbor(increaser); 
                            neighbors_found++; 
                        }
                        increaser++; 
                    }
                 }
             }

        } 

     
    public: 
	Graph() {
        min_y = INF; 
        max_y = -1 * INF; 	
    }


    void export_vtk_paths(vector< vector<int> > paths, char* dirname) {
        for(int i=0; i<paths.size(); i++) { // Export each path to a separate .PLY file
            // Open a new file in dir
            ofstream myfile;
            string sdirname = dirname; 
            myfile.open(sdirname +"/path" + to_string(i) + ".vtk");
            
            // Set up headers
            myfile << "# vtk DataFile Version 3.0" << endl; 
            myfile << "vtk output" << endl; 
            myfile << "ASCII" << endl; 
            myfile << "DATASET POLYDATA" << endl; 
            myfile << "POINTS " << paths[i].size() << " float " << endl;

            // Create points
            set<int> points; 
            for(int j=0; j<paths[i].size(); j++) {
                int point_index = paths[i].at(j); 
                Point point = cells[point_index].get_coord();
                points.insert(point_index);
                myfile << point.x << " " << point.y << " " << point.z << endl;            
            }

            // Create lines
            int total_lines = paths[i].size() - 1; 
            myfile << "LINES " << total_lines << " " << total_lines * 3 << endl; 
            
            for(int j=0; j<paths[i].size() - 1; j++) {
                std::set<int>::iterator it1 = points.find(paths.at(i).at(j));
                std::set<int>::iterator it2 = points.find(paths.at(i).at(j+1));

                myfile << "2 " << distance(points.begin(), it1)  << " " << distance(points.begin(), it2) << endl;
            }
         
            myfile.close();
        }
    }
    
    void export_ply_paths(vector< vector<int> > paths, char* dirname) {
        for(int i=0; i<paths.size(); i++) { // Export each path to a separate .PLY file
            // Open a new file in dir
            ofstream myfile;
            string sdirname = dirname; 
            myfile.open(sdirname +"/path" + to_string(i) + ".ply");
        
            // Set up headers
            myfile << "ply" << endl; 
            myfile << "format ascii 1.0" << endl;
            myfile << "element vertex " << paths[i].size() << endl; 
            myfile << "property float x" << endl;  
            myfile << "property float y" << endl;  
            myfile << "property float z" << endl;  
            myfile << "property float nx" << endl;  
            myfile << "property float ny" << endl;  
            myfile << "property float nz" << endl;  
            myfile << "end_header" << endl; 

             
            // Create points
            set<int>::iterator it; 
            for(int j=0; j<paths[i].size(); j++ ) {
                int point_index = paths[i].at(j); 
                Point point = cells[point_index].get_coord();
                Point norm = cells[point_index].get_normal(); 
                myfile << point.x << " " << point.y * -1 << " " << point.z * -1<< 
                   " " << norm.x << " " << norm.y << " " << norm.z << endl; 
            }
            myfile.close();
        }
    }


    // Creates a point cloud to export to VTK for visualization
    void export_vtk(vector< vector<int> > paths, char* filename) {
        ofstream myfile; 
        myfile.open(filename);

        set<int> points; 
        int total_lines = 0;  
        for(int i=0; i<paths.size(); i++) {
            for(int j=0; j<paths[i].size(); j++) {
                points.insert(paths.at(i).at(j)); 
            }

            total_lines += (paths.at(i).size() - 1);
        }

        // Set up headers
        myfile << "# vtk DataFile Version 3.0" << endl; 
        myfile << "vtk output" << endl; 
        myfile << "ASCII" << endl; 
        myfile << "DATASET POLYDATA" << endl; 
        myfile << "POINTS " << points.size() << " float " << endl; 


        // Create points
        set<int>::iterator it; 
        for(it = points.begin(); it!=points.end(); it++ ) {
            Point point = cells[*it].get_coord(); 
            myfile << point.x << " " << point.y << " " << point.z << endl;            
        }

        // Create lines
        myfile << "LINES " << total_lines << " " << total_lines * 3 << endl; 
        
        for(int i=0; i<paths.size(); i++) {
            for(int j=0; j<paths.at(i).size() - 1; j++) {
                std::set<int>::iterator it1 = points.find(paths.at(i).at(j));
                std::set<int>::iterator it2 = points.find(paths.at(i).at(j+1));

                myfile << "2 " << distance(points.begin(), it1)  << " " << distance(points.begin(), it2) << endl;
            }
        }
     
        myfile.close();
    }

    void export_ply(vector< vector<int> > paths, char* filename) {
        ofstream myfile; 
        myfile.open(filename);
        
        set<int> points; 
        int total_lines = 0;  
        for(int i=0; i<paths.size(); i++) {
            for(int j=0; j<paths[i].size(); j++) {
                points.insert(paths.at(i).at(j)); 
            }

            total_lines += (paths.at(i).size() - 1);
        }

        // Set up headers
        myfile << "ply" << endl; 
        myfile << "format ascii 1.0" << endl;
        myfile << "element vertex " << points.size() << endl; 
        myfile << "property float x" << endl;  
        myfile << "property float y" << endl;  
        myfile << "property float z" << endl;  
        myfile << "property float nx" << endl;  
        myfile << "property float ny" << endl;  
        myfile << "property float nz" << endl;  
        myfile << "end_header" << endl; 

         
        // Create points
        set<int>::iterator it; 
        for(it = points.begin(); it!=points.end(); it++ ) {
            Point point = cells[*it].get_coord(); 
            Point norm = cells[*it].get_normal(); 
            myfile << point.x << " " << point.y  * -1<< " " << point.z * -1 << 
               " " << norm.x << " " << norm.y << " " << norm.z << endl; 
        }

        myfile.close();
    }
    
    // Find the value of a point that is n percent up the hair
    double y_percent(double n) {
        double range = max_y - min_y; 

        return(min_y + (range * n));
    }

    int size() {
        return(cells.size());
    }

    Cell at(int index) {
        if(index < cells.size())
            return(cells[index]); 
        else
            cerr << "Accessing non-existing index" << endl; 
    } 

    void add_cell(Cell cell) {
        this->cells.push_back(cell); 
        
        // Update min y val
        Point center = cell.get_coord(); 
        if(center.y < min_y) {
            min_y = center.y; 
        }        
        // Update max y val
        if(center.y > max_y) {
            max_y = center.y; 
        }   
    }

	// Initialize from VTK file
	Graph(char* vtk_filename) {
	    min_y = INF; 
        max_y = INF * -1;

	    // Get all data from the file
	    vtkSmartPointer<vtkGenericDataObjectReader> reader =
		vtkSmartPointer<vtkGenericDataObjectReader>::New();
	    reader->SetFileName(vtk_filename);
	    reader->Update();

	    // All of the standard data types can be checked and obtained like this:
	    if(reader->IsFilePolyData()) {
            vtkPolyData* polydata = reader->GetPolyDataOutput();
            polydata->BuildLinks(); 

            // Iterate through the cells in mesh
            vtkSmartPointer<vtkIdList> id_list = vtkSmartPointer<vtkIdList>::New();
            vtkSmartPointer<vtkIdList> neighbor_ids = vtkSmartPointer<vtkIdList>::New();
            vtkCellArray *poly_cells = polydata->GetPolys();
            poly_cells->InitTraversal();
         

            cout << "NUMBER OF POLYDATA CELLS: " << polydata->GetNumberOfPolys() << endl; 
            cout << "NUMBER OF POLYS: " << poly_cells->GetNumberOfCells() << endl; 
            vtkIdType max = 0;  
            while(poly_cells->GetNextCell(id_list)) {
                // Get points in the cell
                vector<Point> points; 
                for(auto i=0; i<POINTS_PER_CELL; i++) { 
                    double p[DIMENSION]; 
                    polydata->GetPoint(id_list->GetId(i), p);
                    points.push_back(Point(p[0], p[1], p[2]));   
                }
                 
                // Add cell object to vector
                add_cell(Cell(points[0], points[1], points[2]));
                this->cells.back().add_point_index((int)id_list->GetId(0));
                this->cells.back().add_point_index((int)id_list->GetId(1));
                this->cells.back().add_point_index((int)id_list->GetId(2));
            }

            // Set the cell neighbors
            for(int i=0; i<cells.size(); i++) {
                initialize_cell_neighbors(i);

            }
            

        } 
	    else {
            cerr << "Not a .VTK polydata file." << endl; 	    	    
	    }
	}


    void find_edge_points(double threshold, int above_or_below, vector<int>& found_cells) {
        for(int i=0; i<cells.size(); i++) {
            Cell cell = cells.at(i); 
            Point center = cell.get_coord(); 
            bool edge = cell.is_edge(); // Is it an edge cell?
             
            if(above_or_below == 1) { // Find points above threshold
                if(edge && center.y >= threshold) {
                    found_cells.push_back(i);
                }
            }
            else { // Find points below threshold
                if(edge && center.y <= threshold) {
                    found_cells.push_back(i);
                }
            }
        }    
    }


};

#endif
