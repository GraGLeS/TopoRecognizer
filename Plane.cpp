
/* 
 * File:   Plane.cpp
 * Author: Arbeit
 * Created on 18. Dezember 2016
 */

#include <iostream>
#include <fstream>
#include <string>

#include "Plane.h"

Plane::Plane() {
    
    A=B=C=D=0;
    P=0;
    X_AxisIntercept = Y_AxisIntercept = Z_AxisIntercept = 0;
    DistanceToOrigin=0;
    unitNormalVector = new Eigen::Vector3d;
    Vector3d_1 = new Eigen::Vector3d;
}

Plane::Plane(const Plane& orig) {
}

Plane::~Plane() {
    delete Vector3d_1;
    delete unitNormalVector;
 
}

void Plane::input_WLSsolution(Eigen::Vector3d* WLS_Solution){
    
    A = (*WLS_Solution)(0);
    B = (*WLS_Solution)(1);      
    D = (*WLS_Solution)(2);
    C = -1;
    calc_HessianNormalForm();
    
}

void Plane::output_ParameterIntoTxtFile(string outFilename){
    
    calc_DistanceToOrigin();
    calc_AxisIntercepts();
    
    ofstream outFile;
    string directory = "D:\\masterarbeit\\NetBeansProjects\\Masterarbeit_FW\\output\\LP\\" + outFilename;
    outFile.open (directory.c_str());
    outFile << "Plane general Parameter(constant, not changed after initialization): " << endl; //Header
    outFile << "A" << "    " << "B" << "   "  << "C" << "   " << "D" << endl; 
    outFile << A << "    " << B << "    " << C << "    " << D << endl;
    outFile << "X_AxisIntercept" << "    " << "Y_AxisIntercept" << "   "  << "Z_AxisIntercept" << endl; 
    outFile << X_AxisIntercept << "    " << Y_AxisIntercept << "    " << Z_AxisIntercept << endl;
    outFile << "DistanceToOrigin: " << endl;
    outFile << P << endl;
    outFile << "Hessian Normal Form: " << endl;
    outFile << "P: " << P << endl;
    outFile << "unitNormalVector: " << endl << (*unitNormalVector);
    
}

void Plane::calc_HessianNormalForm(){
    
    (*unitNormalVector)(0) = A/pow(pow(A,2)+pow(B,2)+pow(C,2),0.5);
    (*unitNormalVector)(1) = B/pow(pow(A,2)+pow(B,2)+pow(C,2),0.5);
    (*unitNormalVector)(2) = C/pow(pow(A,2)+pow(B,2)+pow(C,2),0.5);
    P = D/pow(pow(A,2)+pow(B,2)+pow(C,2),0.5);
    
    /*
    (*unitNormalVector)(0) = A;
    (*unitNormalVector)(1) = B;
    (*unitNormalVector)(2) = C;
    
    (*Vector3d_1) = ((*unitNormalVector).normalized());
    (*unitNormalVector) = (*Vector3d_1);
    */
    
}

void Plane::calc_unitNormalVector(){
    
    (*unitNormalVector)(0) = A;
    (*unitNormalVector)(1) = B;
    (*unitNormalVector)(2) = C;
    
    (*Vector3d_1) = ((*unitNormalVector).normalized());
    (*unitNormalVector) = (*Vector3d_1);
}

void Plane::rotate(Eigen::Matrix3d* RotationMatrix){
    
    (*Vector3d_1) = (*RotationMatrix) * (*unitNormalVector);
    (*unitNormalVector) = (*Vector3d_1);
}

void Plane::calc_DistanceToOrigin(){
    
    DistanceToOrigin = P/(*unitNormalVector).norm();
}

void Plane::calc_AxisIntercepts(){
  
    X_AxisIntercept = -P/(*unitNormalVector)(0);   
    Y_AxisIntercept = -P/(*unitNormalVector)(1);      
    Z_AxisIntercept = -P/(*unitNormalVector)(2); 
 
}