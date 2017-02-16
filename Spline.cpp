
/* 
 * File:   Spline.cpp
 * Author: Arbeit
 * Created on 18. Dezember 2016
 */


#include <iostream>
#include <fstream>

#include "Spline.h"
#include "Point3D.h"

Spline::Spline() {




}

Spline::Spline(int N) {
    
    Vector3d_1 = new Eigen::Vector3d;
    
    //Input:
    ordered_Pointset =  new Eigen::MatrixXd(3, N);
    
    //SplineOutput:
    N_Spline = 100; //Menge der aus der B-spline berechneten Punkte für output
    delta_param = 1.0/(N_Spline-1); 
    Spline_Points_output = new Eigen::MatrixXd(3, N_Spline);
}


Spline::~Spline() {
    
    delete ordered_Pointset;
    delete Spline_Points_output;
    
}

void Spline::input_Pointset(vector<Point3D>* Pointset){
    
    int size = (*Pointset).size();
    ordered_Pointset->resize(3, size);
    
    for(int i=0; i<=(size-1); i++){
        
      (*ordered_Pointset)(0,i) = (*Pointset)[i].get_CoordinatesX();
      (*ordered_Pointset)(1,i) = (*Pointset)[i].get_CoordinatesY();
      (*ordered_Pointset)(2,i) = (*Pointset)[i].get_CoordinatesZ();
      
    }
    //output_PointMatrix_TxtFile(ordered_Pointset, "Spline_ordered_Pointset_input");
}

void Spline::output_PointMatrix_TxtFile(Eigen::MatrixXd* EigenMatrixXd, string outFilename){
    
    ofstream outFile;
    string directory = outFilename;
    outFile.open (directory.c_str());
    outFile << "X" << "    " << "Y" << "   "  << "Z" << "   " << "ID" << endl;  //Header
    
    int N = EigenMatrixXd->cols(); //matrixXd(3, N)
    
    for(int i=0; i<= (N-1); i++){
       
        if(i == (N-1)){
        outFile << (*EigenMatrixXd)(0, i) << "  " << (*EigenMatrixXd)(1, i) << " "  << (*EigenMatrixXd)(2, i) << " " << i;
        }else{
        outFile << (*EigenMatrixXd)(0, i) << "  " << (*EigenMatrixXd)(1, i) << " "  << (*EigenMatrixXd)(2, i) << " " << i << endl;
        }
    }
    outFile.close(); 
}

void Spline::output_BSpline_3D_Degree_1_Knots_Ctrls_TxtFile(KnotVectorType3D_deg1& KnotParameters, ControlPointVectorType3D_deg1& ControlPoints, string outFilename){
    
    ofstream outFile;
    string directory =  outFilename;
    outFile.open (directory.c_str());
    outFile << "KnotParameters: " << endl;
    outFile << KnotParameters << endl << endl << endl << endl;
    outFile << "ControlPoints: " << endl;
    outFile << ControlPoints;
    outFile.close(); 
}

void Spline::output_BSpline_3D_Degree_2_Knots_Ctrls_TxtFile(KnotVectorType3D_deg2& KnotParameters, ControlPointVectorType3D_deg2& ControlPoints, string outFilename){
    
    ofstream outFile;
    string directory =   outFilename;
    outFile.open (directory.c_str());
    outFile << "KnotParameters: " << endl;
    outFile << KnotParameters << endl << endl << endl << endl;
    outFile << "ControlPoints: " << endl;
    outFile << ControlPoints;
    outFile.close(); 
}

void Spline::output_BSpline_3D_Degree_3_Knots_Ctrls_TxtFile(KnotVectorType3D_deg3& KnotParameters, ControlPointVectorType3D_deg3& ControlPoints, string outFilename){
    
    ofstream outFile;
    string directory =  outFilename;
    outFile.open (directory.c_str());
    outFile << "KnotParameters: " << endl;
    outFile << KnotParameters << endl << endl << endl << endl;
    outFile << "ControlPoints: " << endl;
    outFile << ControlPoints;
    outFile.close(); 
}

void Spline::interpolate_BSpline_3D_Degree_1_Default(){
    
    KnotVectorType3D_deg1 KnotParameters;
    ControlPointVectorType3D_deg1 ControlPoints;
    PointType3D_deg1 Spline_Point;
    //Eigen::ChordLengths((*ordered_Pointset), KnotParameters); // in diesem Fall alle chordlenght auch knotparameter
    
    const Spline3D_deg1 Spline3D = Eigen::SplineFitting<Spline3D_deg1>::Interpolate((*ordered_Pointset), 1); 
    
    ControlPoints = Spline3D.ctrls();
    KnotParameters = Spline3D.knots();
    //Eigen::DenseIndex Degree = Spline3D.degree();
    output_BSpline_3D_Degree_1_Knots_Ctrls_TxtFile(KnotParameters, ControlPoints, "_Default");
            
    double param=0.0; // u=0 Startknotenwert für ausgabe
    for(int i =0; i<N_Spline; i++){
        
        Spline_Point = Spline3D(param);
        
        if( (abs(Spline_Point(0,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(0,i) = 0.0;     
        }else{
        (*Spline_Points_output)(0,i) = Spline_Point(0,0);     
        }
        
        if( (abs(Spline_Point(1,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(1,i) = 0.0;    
        }else{
        (*Spline_Points_output)(1,i) = Spline_Point(1,0);   
        }
        
        if( (abs(Spline_Point(2,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(2,i) = 0.0;     
        }else{
        (*Spline_Points_output)(2,i) = Spline_Point(2,0);     
        }
        
        param += delta_param;
    }
    
    output_PointMatrix_TxtFile(Spline_Points_output, "BSpline_3D_Degree_1_Points_Default");
}

void Spline::interpolate_BSpline_3D_Degree_2_Default(){
    
    KnotVectorType3D_deg2 KnotParameters;
    ControlPointVectorType3D_deg2 ControlPoints;
    PointType3D_deg2 Spline_Point;
    //Eigen::ChordLengths((*ordered_Pointset), KnotParameters); // in diesem Fall alle chordlenght auch knotparameter
    
    const Spline3D_deg2 Spline3D = Eigen::SplineFitting<Spline3D_deg2>::Interpolate((*ordered_Pointset), 2); 
    
    ControlPoints = Spline3D.ctrls();
    KnotParameters = Spline3D.knots();
    //Eigen::DenseIndex Degree = Spline3D.degree();
    output_BSpline_3D_Degree_2_Knots_Ctrls_TxtFile(KnotParameters, ControlPoints, "_Default");
    
    double param=0.0; // u=0 Startknotenwert für ausgabe
    for(int i =0; i<N_Spline; i++){
        
        Spline_Point = Spline3D(param);
        
        if( (abs(Spline_Point(0,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(0,i) = 0.0;     
        }else{
        (*Spline_Points_output)(0,i) = Spline_Point(0,0);     
        }
        
        if( (abs(Spline_Point(1,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(1,i) = 0.0;    
        }else{
        (*Spline_Points_output)(1,i) = Spline_Point(1,0);   
        }
        
        if( (abs(Spline_Point(2,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(2,i) = 0.0;     
        }else{
        (*Spline_Points_output)(2,i) = Spline_Point(2,0);     
        }
        param += delta_param;
    }
    
    output_PointMatrix_TxtFile(Spline_Points_output, "BSpline_3D_Degree_2_Points_Default");
}

void Spline::interpolate_BSpline_3D_Degree_2_Chordlenghts(){
    
    KnotVectorType3D_deg2 KnotParameters;
    ControlPointVectorType3D_deg2 ControlPoints;
    PointType3D_deg2 Spline_Point;
    Eigen::ChordLengths((*ordered_Pointset), KnotParameters); // in diesem Fall alle chordlenght auch knotparameter
    
    const Spline3D_deg2 Spline3D = Eigen::SplineFitting<Spline3D_deg2>::Interpolate((*ordered_Pointset), 2, KnotParameters); 
    
    ControlPoints = Spline3D.ctrls();
    //Eigen::DenseIndex Degree = Spline3D.degree();
    output_BSpline_3D_Degree_2_Knots_Ctrls_TxtFile(KnotParameters, ControlPoints, "_Chordlenghts");
            
    double param=0.0; // u=0 Startknotenwert für ausgabe
    for(int i =0; i<N_Spline; i++){
        
        Spline_Point = Spline3D(param);
        
        if( (abs(Spline_Point(0,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(0,i) = 0.0;     
        }else{
        (*Spline_Points_output)(0,i) = Spline_Point(0,0);     
        }
        
        if( (abs(Spline_Point(1,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(1,i) = 0.0;    
        }else{
        (*Spline_Points_output)(1,i) = Spline_Point(1,0);   
        }
        
        if( (abs(Spline_Point(2,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(2,i) = 0.0;     
        }else{
        (*Spline_Points_output)(2,i) = Spline_Point(2,0);     
        }
        
        param += delta_param;
    }
    
    output_PointMatrix_TxtFile(Spline_Points_output, "BSpline_3D_Degree_2_Points_Chordlenghts");

}

/*
 * Funktioniert nicht! Kontrollpunkte werde alle =0, Grund unbekannt. Ich denke die Default Funktion rechnet mit gleichen Knotenparametern und gibt die richtigen Kontrollpunkte dazu aus!!!
void Spline::interpolate_BSpline_3D_Degree_2_Chordlenghts_KnotAveraging(){
    
    KnotVectorType3D_deg2 KnotParameters;
    KnotVectorType3D_deg2 KnotParameters_Average;
    ControlPointVectorType3D_deg2 ControlPoints;
    PointType3D_deg2 Spline_Point;
    Eigen::DenseIndex Degree = 2;
    Eigen::ChordLengths((*ordered_Pointset), KnotParameters); // in diesem Fall alle chordlenght auch knotparameter
    Eigen::KnotAveraging(KnotParameters, Degree, KnotParameters_Average);
    
    const Spline3D_deg2 Spline3D = Eigen::SplineFitting<Spline3D_deg2>::Interpolate((*ordered_Pointset), 2, KnotParameters_Average); 
    
    ControlPoints = Spline3D.ctrls();
    
    output_BSpline_3D_Degree_2_Knots_Ctrls_TxtFile(KnotParameters_Average, ControlPoints, "_Chordlenghts_KnotAveraging");
            
    double param=0.0; // u=0 Startknotenwert für ausgabe
    for(int i =0; i<N_Spline; i++){
        
        Spline_Point = Spline3D(param);
        
        if( (abs(Spline_Point(0,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(0,i) = 0.0;     
        }else{
        (*Spline_Points_output)(0,i) = Spline_Point(0,0);     
        }
        
        if( (abs(Spline_Point(1,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(1,i) = 0.0;    
        }else{
        (*Spline_Points_output)(1,i) = Spline_Point(1,0);   
        }
        
        if( (abs(Spline_Point(2,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(2,i) = 0.0;     
        }else{
        (*Spline_Points_output)(2,i) = Spline_Point(2,0);     
        }
        
        param += delta_param;
    }
    
    output_PointMatrix_TxtFile(Spline_Points_output, "BSpline_3D_Degree_2_Points_Chordlenghts_KnotAveraging");
}
*/

void Spline::interpolate_BSpline_3D_Degree_3_Default(){
    
    KnotVectorType3D_deg3 KnotParameters;
    ControlPointVectorType3D_deg3 ControlPoints;
    PointType3D_deg3 Spline_Point;
    //Eigen::ChordLengths((*ordered_Pointset), KnotParameters); // in diesem Fall alle chordlenght auch knotparameter
    
    const Spline3D_deg3 Spline3D = Eigen::SplineFitting<Spline3D_deg3>::Interpolate((*ordered_Pointset), 3); 
    
    ControlPoints = Spline3D.ctrls();
    KnotParameters = Spline3D.knots();
    //Eigen::DenseIndex Degree = Spline3D.degree();
    output_BSpline_3D_Degree_3_Knots_Ctrls_TxtFile(KnotParameters, ControlPoints, "_Default");
            
    double param=0.0; // u=0 Startknotenwert für ausgabe
    for(int i =0; i<N_Spline; i++){
        
        Spline_Point = Spline3D(param);
        
        if( (abs(Spline_Point(0,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(0,i) = 0.0;     
        }else{
        (*Spline_Points_output)(0,i) = Spline_Point(0,0);     
        }
        
        if( (abs(Spline_Point(1,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(1,i) = 0.0;    
        }else{
        (*Spline_Points_output)(1,i) = Spline_Point(1,0);   
        }
        
        if( (abs(Spline_Point(2,0))-0.00001) <0.00001 ){
        (*Spline_Points_output)(2,i) = 0.0;     
        }else{
        (*Spline_Points_output)(2,i) = Spline_Point(2,0);     
        }
        
        param += delta_param;
    }
    
    output_PointMatrix_TxtFile(Spline_Points_output, "BSpline_3D_Degree_3_Points_Default");
}

double Spline::calc_Line_Lenght_Spline(){
    
    int N_cols = Spline_Points_output->cols();
    double lenght =0;
    
    for(int i=0; i<(N_cols-1); i++){
        
        (*Vector3d_1) = Spline_Points_output->col(i) - Spline_Points_output->col(i+1);
        lenght += Vector3d_1->norm();
    }
    return lenght;
}

double Spline::calc_Line_Lenght_ordered_PointInput(){
    
    int N_cols = ordered_Pointset->cols();
    double lenght =0;

    for(int i=0; i<(N_cols-1); i++){
        
        (*Vector3d_1) = ordered_Pointset->col(i) - ordered_Pointset->col(i+1);
        lenght += Vector3d_1->norm();
    }
    
    return lenght;  
}
