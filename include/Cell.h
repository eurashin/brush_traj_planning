#ifndef CELL
#define CELL

#include <iostream>
#include <math.h>
#include <string>
#include <vector>

const int NUM_DIMENSIONS = 3; 

// 3D Point Class
class Point {
    public: 
        double x, y, z;
        Point() {
            this->x = 0; 
            this->y = 0; 
            this->z = 0; 
        }

        Point(double x, double y, double z) {
            this->x = x; 
            this->y = y; 
            this->z = z; 
        }

        std::string to_string() {
             return("(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")"); 
        }
};


// A triangle in the triangular mesh
class Cell {
    protected: 
        Point coord; //coordinates
        Point normal; //unit normal vector
        std::vector<int> point_indices; 
        std::vector<int> neighbors; //index of neighbors
        double lowest_point_val;
        bool flag; //boundary cells 

    private:
        // Calculate the unit normal of three points 
        void compute_normal(Point p1, Point p2, Point p3) {
            // Calculate two vectors 
            Point v(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);  
            Point u(p3.x - p2.x, p3.y - p2.y, p3.z - p2.z);  

            // Find the cross product
            double x = (u.y * v.z) - (u.z * v.y);
            double y = (u.z * v.x) - (u.x * v.z);
            double z = (u.x * v.y) - (u.y * v.x);
    
            double length = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)); 
            if(length > 0) {
                normal = Point(x/length, y/length, z/length); 
            }
            else { 
                std::cerr << "Error in normal calculation..." << std::endl;  
            }
        } 

    public: 
        Cell() {
            coord.x = 0;
            coord.y = 0; 
            coord.z = 0; 

            normal.x = 0; 
            normal.y = 1; 
            normal.z = 0;

            flag = false;  

        } 

        // Construct Cell "plane" from three verteces
        Cell(Point p1, Point p2, Point p3) {
            coord.x = (p1.x + p2.x + p3.x)/NUM_DIMENSIONS; 
            coord.y = (p1.y + p2.y + p3.y)/NUM_DIMENSIONS; 
            coord.z = (p1.z + p2.z + p3.z)/NUM_DIMENSIONS;
           
            compute_normal(p1, p2, p3); 
            flag = false;
        }

        bool equals(Cell cell) {
            Point coords = cell.get_coord(); 
            return(
                    this->coord.x == coords.x &&
                    this->coord.y == coords.y &&
                    this->coord.z == coords.z
                  ); 
        }
        
        bool is_edge() {
            return(neighbors.size() == 2);
        }

        void set_flag() {
            flag = true; 
        }

        void add_point_index(int n) {
            point_indices.push_back(n); 
        }

        std::vector<int> get_point_indices() {
            return(point_indices);
        }

        void add_neighbor(int n) {
            neighbors.push_back(n);
        }

        void set_lowest_point_val(double val) {
            lowest_point_val = val; 
        }
        
        double get_lowest_point_val() {
            return(lowest_point_val);
        }

        std::vector<int> get_neighbors() {
            return(neighbors);
        }

        Point get_coord() {
            return(coord);
        }

        Point get_normal() {
            return(normal);
        }

        void print() {
            std::cout << "Central coordinate: " << coord.to_string() << std::endl; 
            std::cout << "Normal: " << normal.to_string() << std::endl; 
            std::cout << "Neighbors: " << std::endl; 
            for(int i=0; i<neighbors.size(); i++) {
                std::cout << neighbors[i] << "  "; 
            } 
            std::cout << std::endl;
            
            std::cout << "Point indices: " << std::endl; 
            for(int i=0; i<point_indices.size(); i++) {
                std::cout << point_indices[i] << "  "; 
            } 
            std::cout << std::endl;
       } 

};

#endif
