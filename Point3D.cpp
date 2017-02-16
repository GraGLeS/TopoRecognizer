
/*
 * File:   Point3D.cpp
 * Author: Arbeit
 * Created on 18. Dezember 2016
 */

#include <iostream>
//#include <cstdlib>
//#include <math.h>

#include "Point3D.h"

Point3D::Point3D(){

 //CoordinatesXYZ(0)=0;
 //CoordinatesXYZ(1)=0;
 //CoordinatesXYZ(2)=0;
 GlobalID = 0; 
 LocalID = 0;
 CubicWeight = 0.0;
 epsilon = 1;
 
 H_last_iteration = 0;
 ID_ordered = 0; //if 0 point is deleted --> forward +1.+2.... backward -1, -2.....
 ordered=false;
 
 //pCoordinatesXYZ = new Eigen::Vector3d;
    
}


 
Point3D::~Point3D(){

   //delete pCoordinatesXYZ;

}

/*
double Point3D::calc_DistanceToPoint(Point3D& Point){
    double a=0;
    a= CoordinatesXYZ-Point.get_CoordinatesXYZ()
    
    return a;
}
 */