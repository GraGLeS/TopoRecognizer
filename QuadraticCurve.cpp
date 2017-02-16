
/* 
 * File:   QuadraticCurve.cpp
 * Author: Arbeit 
 * Created on 18. Dezember 2016
 */
#include <iostream>
#include <fstream>
#include <string>

#include "QuadraticCurve.h"

QuadraticCurve::QuadraticCurve() {
   
    A=B=C=D=0;

}

QuadraticCurve::QuadraticCurve(const QuadraticCurve& orig) {
}

QuadraticCurve::~QuadraticCurve() {

}

void QuadraticCurve::input_WLSsolution(Eigen::Vector3d* WLS_Solution){
    
    A = (*WLS_Solution)(0);
    B = (*WLS_Solution)(1);
    D = (*WLS_Solution)(2);
    C = -1;
}
  
void QuadraticCurve::output_ParameterIntoTxtFile(string outFilename){
    
    ofstream outFile;
    string directory = "D:\\masterarbeit\\NetBeansProjects\\Masterarbeit_FW\\output\\LP\\" + outFilename;
    outFile.open (directory.c_str());
    outFile << "QuadraticCurve Parameter: " << endl; //Header
    outFile << "A" << "    " << "B" << "   "  << "C" << "   "  << "D" << endl; 
    outFile << A << "    " << B << "    " << C << " " << D;
}

double QuadraticCurve::get_y(double x){
    
    return (A*pow(x,2)+B*x+D);
}

void QuadraticCurve::output_FctValues_TextFile(double min_x, double max_x, double delta_x, string outFilename){
    
    ofstream outFile;
    string directory = "D:\\masterarbeit\\NetBeansProjects\\Masterarbeit_FW\\output\\LP\\" + outFilename;
    outFile.open (directory.c_str());
    outFile << "X" << "    " << "Y";  //Header
    
    double y=0;
    double x = min_x;
    while(x<=max_x){
        
        y = A*pow(x,2)+ B*x+D;
        outFile << endl << x << "  " << y;  
        x += delta_x;
    } 
    
    outFile.close();
}