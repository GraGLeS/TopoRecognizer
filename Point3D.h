
/* 
 * File:   Point3D.h
 * Author: Arbeit
 *
 * Created on 18. Dezember 2016
 */

#ifndef POINT3D_H
#define POINT3D_H

#include <vector>
#include <Eigen/Dense>

using namespace std;

class Point3D {

public:
    Point3D();
    ~Point3D();
     
    
    
    inline void set_CoordinatesXYZ(Eigen::Vector3d* CoordinatesXYZ) {
        (this->CoordinatesXYZ) = (*CoordinatesXYZ);
    }
    
    inline Eigen::Vector3d get_CoordinatesXYZ(){
        return CoordinatesXYZ;
    }
    
    inline double get_CoordinatesX(){
        return (CoordinatesXYZ)(0);
    }
        
    inline void set_CoordinatesX(double x){
        (this->CoordinatesXYZ)(0) = x;
    }

    
    inline double get_CoordinatesY(){
        return (CoordinatesXYZ)(1);
        
    }
        
    inline void set_CoordinatesY(double y){
        (this->CoordinatesXYZ)(1) = y;
    }
    
      
    inline double get_CoordinatesZ(){
        return (CoordinatesXYZ)(2);
        
    }
        
    inline void set_CoordinatesZ(double z){
        (this->CoordinatesXYZ)(2) = z;
    }
    
    
    /*
    inline void set_CoordinatesXYZ(Eigen::Vector3d* CoordinatesXYZ) {
        (*this->CoordinatesXYZ) = (*CoordinatesXYZ);
    }
    
    inline Eigen::Vector3d get_CoordinatesXYZ(){
        return (*CoordinatesXYZ);
    }
    
    inline double get_CoordinatesX(){
        return (*CoordinatesXYZ)(0);
    }
        
    inline void set_CoordinatesX(double x){
        (*this->CoordinatesXYZ)(0) = x;
    }

    
    inline double get_CoordinatesY(){
        return (*CoordinatesXYZ)(1);
        
    }
        
    inline void set_CoordinatesY(double y){
        (*this->CoordinatesXYZ)(1) = y;
    }
    
      
    inline double get_CoordinatesZ(){
        return (*CoordinatesXYZ)(2);
        
    }
        
    inline void set_CoordinatesZ(double z){
        (*this->CoordinatesXYZ)(2) = z;
    }
    
    */
    
    inline int get_GlobalID(){
        return GlobalID;
    }
    
    inline void set_GlobalID(int ID){
        this->GlobalID = ID;
    }
    
    inline int get_LocalID(){
        return LocalID;
    }
    
    inline void set_LocalID(int ID){
        this->LocalID = ID;
    }
    
    inline double get_CubicWeight(){
        return CubicWeight;
    }
    
    inline void set_CubicWeight(double CubicWeight){
        this->CubicWeight = CubicWeight;
    }
    
    inline double get_epsilon(){
        return epsilon;
    }
    
    inline void set_epsilon(double Epsilon){
        this->epsilon = Epsilon;
    }
  
    inline double get_H_last_iteration(){
        return H_last_iteration;
    }
    
    inline void set_H_last_iteration(double H_last_iteration){
        this->H_last_iteration = H_last_iteration;
    }
    
    inline int get_ID_ordered(){
        return ID_ordered;
    }
    
    inline void set_ID_ordered(int ID_ordered){
        this->ID_ordered = ID_ordered;
    }
    
    inline int get_bool_ordered(){
        return ordered;
    }
    
    inline void set_bool_ordered(bool A){
        this->ordered = A;
    }
    
    
    
private:

    
//smoothing:
int GlobalID; //Punkt ID im InputTriplelinePointset
int LocalID; //Punkt ID im LocalPointsetObject
double CubicWeight;
double epsilon; //average approximation error of point


//ordering
double H_last_iteration;
int ID_ordered; //if 0 point is deleted
bool ordered;

//Eigen::Vector3d* pCoordinatesXYZ;
Eigen::Vector3d CoordinatesXYZ;

};

#endif /* POINT3D_H */

