
/* 
 * File:   LinearCurve.cpp
 * Author: Arbeit
 * Created on 18. Dezember 2016
 */
#include <iostream>
#include <fstream>
//#include <string>

#include "LinearCurve.h"

LinearCurve::LinearCurve() {

    A=B=C=0;
    
    normalizedDirection = new Eigen::Vector3d;
    curvePoint = new Eigen::Vector3d;
    
    Vector3d_1 = new Eigen::Vector3d;

}

LinearCurve::LinearCurve(const LinearCurve& orig) {
}

LinearCurve::~LinearCurve() {

    delete normalizedDirection;
    delete curvePoint;
    
    delete Vector3d_1;
    
}

void LinearCurve::input_WLSsolution(Eigen::Vector2d* WLS_Solution){
    
    A = (*WLS_Solution)(0);
    C = (*WLS_Solution)(1);      
    B = -1;
   
    calc_PointSlopeForm3D();
}

void LinearCurve::calc_PointSlopeForm3D(){
    
    //SützPunkt ist Achsenabschnitt Y-achse: (0,C) mit z=0 da Gerade in XY-Ebene liegt
    (*curvePoint)(0)= 0;
    (*curvePoint)(1)= C;       
    (*curvePoint)(2)= 0;
    
    //direction vector (-B,A) z=0;
    (*Vector3d_1)(0)= -B;
    (*Vector3d_1)(1)= A;
    (*Vector3d_1)(2)= 0;
    
    (*normalizedDirection)=(*Vector3d_1).normalized();
}

void LinearCurve::output_ParameterIntoTxtFile(string outFilename){
    
    ofstream outFile;
    string directory = outFilename;
    outFile.open (directory.c_str());
    outFile << "LinearCurve Parameter: " << endl; //Header
    outFile << "A" << "    " << "B" << "   "  << "C" << endl; 
    outFile << A << "    " << B << "    " << C << endl;
    outFile << "normalizedDirection: " << endl;
    outFile << (*normalizedDirection) << endl;
    outFile << "(*curvePoint): " << endl;
    outFile << (*curvePoint);
    
}

void LinearCurve::rotate(Eigen::Matrix3d* RotationMatrix){
    
    (*Vector3d_1) = (*RotationMatrix) * (*normalizedDirection);
    (*normalizedDirection) = (*Vector3d_1);
    
    (*Vector3d_1) = (*RotationMatrix) * (*curvePoint);
    (*curvePoint) = (*Vector3d_1);
}

double LinearCurve::get_y(double x){
    return A*x+C;
}

void LinearCurve::output_FctValues_PointSlopeForm3D_TextFile(double min_t, double max_t, double delta_t, string outFilename){
   
    ofstream outFile;
    string directory = outFilename;
    outFile.open (directory.c_str());
    outFile << "X" << "    " << "Y" << "   "  << "Z";  //Header
    
    double t = min_t;
    while(t<=max_t){
       
        (*Vector3d_1) = (*curvePoint) + t * (*normalizedDirection);
        outFile << endl << (*Vector3d_1)(0) << "  " << (*Vector3d_1)(1) << "    " << (*Vector3d_1)(2);  
        t += delta_t;
    } 
    
    outFile.close();
}

void LinearCurve::output_FctValues_SlopeInterceptForm2D_TextFile(double min_x, double max_x, double delta_x, string outFilename){
    
    ofstream outFile;
    string directory = outFilename;
    outFile.open (directory.c_str());
    outFile << "X" << "    " << "Y";  //Header
    
    double y=0;
    double x = min_x;
    while(x<=max_x){
        
        y = A*x+ C;
        outFile << endl << x << "  " << y;  
        x += delta_x;
    } 
    
    outFile.close();
}
