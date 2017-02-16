
/* 
 * File:   LocalPointset.cpp
 * Author: Arbeit
 * 
 * Created on 18. Dezember 2016
 */

//#include <cstdio>
#include <iostream>
//#include <cstdlib>
//#include <Eigen/Geometry>
//#include <math.h>
#include <fstream>

#include "Point3D.h"
#include "LocalPointset.h"
#include "TriplelinePointsetClass.h"
#include "Plane.h"
#include "LinearCurve.h"
#include "QuadraticCurve.h"

LocalPointset::LocalPointset(){
}

LocalPointset::LocalPointset(TriplelinePointsetClass* owner): m_origin(owner){
    
    N_LocalPoints = 0;
    maxPointID = 0;
    LP_radius_H = 0;
    allocatedVectorsize = m_origin->get_N_TriplelinePoints();
    GlobalID_OriginPoint = 0 ;
    LocalID_OriginPoint = 0; //loop point wird immer auf platz 0 gesetzt !!!
    LocalPoints = new vector<Point3D>(allocatedVectorsize);
    
    WLS_Plane = new Plane;
    WLS_LinearCurve = new LinearCurve;
    WLS_QuadraticCurve = new QuadraticCurve;
    
    //Rechenmatrizenm
    Vector3d_1 = new Eigen::Vector3d;
    Vector3d_2 = new Eigen::Vector3d;
    Vector3d_3 = new Eigen::Vector3d;
    Vector3d_4 = new Eigen::Vector3d;
    Vector3d_5 = new Eigen::Vector3d;
   
    Matrix3d_1 = new Eigen::Matrix3d;
    Matrix3d_2 = new Eigen::Matrix3d;

    Vector2d_1 = new Eigen::Vector2d;
    Vector2d_2 = new Eigen::Vector2d;
    Vector2d_3 = new Eigen::Vector2d;
    
    Matrix2d_1 = new Eigen::Matrix2d;
    Matrix2d_2 = new Eigen::Matrix2d;
    
    pOutFile_LP = new ofstream;
    
    
    inverse_Rotation1 = new Eigen::Matrix3d;
    Theta1=0;
    Theta2=0;
    normalizedRotationaxis2 = new Eigen::Vector3d;
    stored_Z_AxisIntercept_Plane =0;
    stored_Z_Coordinates_LP = new vector<double>(allocatedVectorsize);
    stored_CoordinatesOriginPoint = new Eigen::Vector3d;
    inverse_Rotation2 = new Eigen::Matrix3d;
    Theta3=0;
    inverse_Rotation3 = new Eigen::Matrix3d;
}

 
LocalPointset::~LocalPointset() {

    delete Vector3d_1;
    delete Vector3d_2;
    delete Vector3d_3;
    delete Vector3d_4;
    delete Vector3d_5;
    
    delete Matrix3d_1;
    delete Matrix3d_2;
    
    delete Vector2d_1;
    delete Vector2d_2;
    delete Vector2d_3;
    
    delete Matrix2d_1;
    delete Matrix2d_2;
    
    delete inverse_Rotation1;
    delete normalizedRotationaxis2;
    delete stored_Z_Coordinates_LP;
    delete stored_CoordinatesOriginPoint;
    delete inverse_Rotation2;
    delete inverse_Rotation3;
    
    delete WLS_Plane;
    delete WLS_LinearCurve;
    delete WLS_QuadraticCurve;
    delete LocalPoints;
    delete pOutFile_LP;
    
}

void LocalPointset::clear(){
    
    (*LocalPoints).resize(0);
    N_LocalPoints = LocalPoints->size();
    GlobalID_OriginPoint = 0 ;
    Theta1=0;
    Theta2=0;
    Theta3=0;
}

void LocalPointset::calc_PointToPointDistances(string outFilename){ //nur als Test Function benutzt!!!
    
    Eigen::MatrixXd* PointToPointDistances;
    PointToPointDistances= new Eigen::MatrixXd(N_LocalPoints, N_LocalPoints);
    
    int u=0;
    int p=0;
    double Distance=0;
    
    for(int i=0; i<=maxPointID; i++){
        p=i;
        u=i+1;
        for(u; u<=maxPointID; u++){
        (*Vector3d_1) = (*LocalPoints)[p].get_CoordinatesXYZ() - (*LocalPoints)[u].get_CoordinatesXYZ();
        Distance = (*Vector3d_1).norm();
        (*PointToPointDistances)(p,u) = (*PointToPointDistances)(u,p) = Distance;
        }
    }  
    //output_EigenMatrixXd(PointToPointDistances, outFilename); 
    delete PointToPointDistances;
}
 
void LocalPointset::set_LocalPoint(int GlobalID_TriplelinePS, vector<Point3D>* Pointset){
    LocalPoints->push_back((*Pointset)[GlobalID_TriplelinePS]);
    N_LocalPoints = LocalPoints->size();
    (*LocalPoints)[N_LocalPoints-1].set_LocalID(N_LocalPoints-1);
    maxPointID = LocalPoints->size()-1;
    }

void LocalPointset::set_OriginPoint(int GlobalID_TriplelinePS, vector<Point3D>* Pointset){
    //LoopPoint hinzufügen ;LoopPoint immer auf Stelle 0 des LocalPoints vectors!!!
    if(LocalPoints->size() == 0){
        LocalPoints->push_back((*Pointset)[GlobalID_TriplelinePS]);
        GlobalID_OriginPoint = GlobalID_TriplelinePS;
        N_LocalPoints = 1;
        (*LocalPoints)[N_LocalPoints-1].set_LocalID(N_LocalPoints-1);
        maxPointID = 0;
    }else{
        (*LocalPoints)[0]= ((*Pointset)[GlobalID_TriplelinePS]);
        N_LocalPoints = LocalPoints->size();
        maxPointID = LocalPoints->size()-1;
    }   
}

string LocalPointset::get_NameDataOutput(string addName){
    
    stringstream n_MLSIterationStep; 
    n_MLSIterationStep << (m_origin->get_n());
    stringstream count_LPIterationStep_NminLoop;
    count_LPIterationStep_NminLoop << (m_origin->get_count());
    stringstream icount_LPIterationStep_rhoLoop;
    icount_LPIterationStep_rhoLoop << (m_origin->get_iterationcount());
    stringstream GlobalID_LP; 
    GlobalID_LP << GlobalID_OriginPoint;
    string seperator = "_";
    string outFilename = n_MLSIterationStep.str() + seperator + "n" +  seperator + GlobalID_LP.str() +  seperator + "ID" + seperator + count_LPIterationStep_NminLoop.str() +  seperator + icount_LPIterationStep_rhoLoop.str() + seperator + addName;
    return outFilename;
}

void LocalPointset::output_DataDebugging_txt(string outFilename){
   
    string directory = outFilename;
    pOutFile_LP->open (directory.c_str());
    
}

double LocalPointset::calc_Correlation_LP(double H){
   
    double rho = 0;
    
    //initialize output_DataDebugging_txt
    output_DataDebugging_txt(get_NameDataOutput("calc_rho"));
    //output_LP_TxtFile(get_NameDataOutput("LocalPSBeforeMLS"));
    (*pOutFile_LP) << "N local: " << N_LocalPoints << endl;
    (*pOutFile_LP) << "H local: " << H << endl;
   
    calc_CubicWeights(H); //cubic weights nach buffer switch der ersten iteration alle = 1 --> grund: es werden immer die orrigin points in TPS_AfterMLS abgesopeichert welche den weight 1 haben. weight wird nicht auf NULL zurück gesetzt!
    //output_LP_TxtFile(get_NameDataOutput("LP_cubicWeights"));
    
    calc_WLS_Plane();
    project_LP_OntoWLSPlane();
    //output_LP_TxtFile(get_NameDataOutput("LP_OntoWLSPlane"));
     
    //TEST1
    //calc_WLS_LinearCurve();
    //(*WLS_LinearCurve).output_FctValues_PointSlopeForm3D_TextFile(2, 10, 0.1, get_NameDataOutput("LinearCurve_Values_TESTTEST"));
    //(*WLS_LinearCurve).output_ParameterIntoTxtFile(get_NameDataOutput("LinearCurve_Param_TESTTEST"));
    //(*WLS_LinearCurve).output_FctValues_SlopeInterceptForm2D_TextFile(-3, 3, 0.1, get_NameDataOutput("WLS_LinearCurve_Values2_AfterCalculation"));
    //calc_WLSLinearCurve3D(); //File Test3D
    //(*pOutFile) << "ERROR_ErstBerechnung " << calc_AverageApproxmationError_LinearCurve() << endl;
    //TEST1
    
    move_OriginOfCoordinateSystem_ToOriginPoint();
    //calc_PointToPointDistances(get_NameDataOutput("Distances_LP_OriginOfCoordinateSystemToOriginPoint"));
    //output_LP_TxtFile(get_NameDataOutput("LP_CoordinateSystemToOriginPoint"));
    
    calc_WLS_LinearCurve();
    //(*WLS_LinearCurve).output_FctValues_PointSlopeForm3D_TextFile(-2, 2, 0.1, get_NameDataOutput("LinearCurve_Values_AfterCalculation"));
    //(*WLS_LinearCurve).output_ParameterIntoTxtFile(get_NameDataOutput("LinearCurve_Param_AfterCalculation"));
    //(*WLS_LinearCurve).output_FctValues_SlopeInterceptForm2D_TextFile(-3, 3, 0.1, get_NameDataOutput("WLS_LinearCurve_Values2_AfterCalculation"));
    //calc_WLSLinearCurve3D(); //File Test3D
    //(*pOutFile_LP) << "calc_AverageApproxmationError_LinearCurve" << calc_AverageApproxmationError_LinearCurve() << endl;
    
    rotate_Pointset_parallelTo_110();
    
    //correlation berechnen
    rho = calc_rho();
    (*pOutFile_LP) << "Rho" << rho;
    pOutFile_LP->close();
   
    return rho;
}

void LocalPointset::rotate_Pointset_parallelTo_110(){
    
    //rotate Localpoints that WLS_LinearCurve = 45° to positiv xaxis of coordinate system 
    //rotate_LocalPointsetParallelTo110Direction();
    (*Vector3d_1) << 1,1,0; //100 positiv x-axis i.e. parallel to 110
    (*Vector3d_2) = (*(*WLS_LinearCurve).get_normalizedDirection());
    (*pOutFile_LP) << "rotate LP and LinearCurve to 45° to positiv xaxis 100 of coordinate system" << endl;
    double dot = (*Vector3d_2).dot((*Vector3d_1));
    if( (dot==1) || (dot==-1)){
        (*pOutFile_LP) << "WLS_LinearCurve already 45° to positiv xaxis 100 of coordinate system" << endl;
        Theta2=0;
    }else{
        Theta2 = calc_AngleTwoVector3d(Vector3d_2, Vector3d_1); //muss für calc_MovedPointAfterMLS() abgespeichert werden!!!
        (*pOutFile_LP) << "Theta2" << endl;
        (*pOutFile_LP) << "angle Theta2 in rad d: " << Theta2 << endl;
        (*pOutFile_LP) << "angle Theta2 in degree: " << Theta2*180/M_PI << endl <<endl;
        
        (*Vector3d_3) = (*calc_NormalizedRotationAxis(Vector3d_2, Vector3d_1));
        (*normalizedRotationaxis2) = (*Vector3d_3); //muss für calc_MovedPointAfterMLS() abgespeichert werden!!!
        (*Matrix3d_1) = (*calc_RotationMatrix(Vector3d_3, Theta2));
           
        (*pOutFile_LP) << "rotationaxis2: " << endl << (*Vector3d_3) << endl;
        (*pOutFile_LP) << "rotationmatrix2: " << endl << (*Matrix3d_1) << endl;
        
        rotate_LocalPoints(Matrix3d_1);
        (*WLS_LinearCurve).rotate(Matrix3d_1);
        //Abspeichern der inversen Rotation
        (*inverse_Rotation2) = (*Matrix3d_1).inverse();
        
    }
    //(*WLS_LinearCurve).output_ParameterIntoTxtFile(get_NameDataOutput("LinearCurve_Param_After45"));
    //(*WLS_LinearCurve).output_FctValues_PointSlopeForm3D_TextFile(-2, 2, 0.1, get_NameDataOutput("LinearCurve_Values_After45"));
    //calc_PointToPointDistances(get_NameDataOutput("Distances_LP_After45"));
    //output_LP_TxtFile(get_NameDataOutput("LP_After45")); 
  
}

double LocalPointset::calc_rho(){
    
    double a, b, sum_x, sum_y, x_mean, y_mean, sum_a2, sum_b2, sum_ab;
    a=b=sum_x=sum_y=x_mean=y_mean=sum_a2=sum_b2=sum_ab=0;
    
    for(int i=0; i<=maxPointID; i++){
       
        sum_x += (*LocalPoints)[i].get_CoordinatesX();
        sum_y += (*LocalPoints)[i].get_CoordinatesY();
    }
     
    x_mean = sum_x/N_LocalPoints;
    y_mean = sum_y/N_LocalPoints;
  
    for(int i=0; i<=maxPointID; i++){
       
        a = (*LocalPoints)[i].get_CoordinatesX() - x_mean;
        b = (*LocalPoints)[i].get_CoordinatesY() - y_mean;
        sum_a2 += pow(a,2);
        sum_b2 += pow(b,2);
        sum_ab += (a*b);
    }
    
    return sum_ab/pow(sum_a2*sum_b2, 0.5);   
}

void LocalPointset::output_LP_TxtFile(string outFilename){
    
    ofstream outFile;
    string directory = outFilename;
    outFile.open (directory.c_str());
    outFile << "X" << "    " << "Y" << "   "  << "Z" << "   " << "GLobalID" << "   " << "LocalID" << "   " << "Distances" << "   " << "CubicWeight" << endl;  //Header
    
    for(int i=0; i<=maxPointID; i++){
         double Distance = (*(m_origin->get_PointToPointDistancesMatrix()))(GlobalID_OriginPoint, (*LocalPoints)[i].get_GlobalID());
         if(i == maxPointID){
         outFile << (*LocalPoints)[i].get_CoordinatesX() << "  " << (*LocalPoints)[i].get_CoordinatesY() << " "  << (*LocalPoints)[i].get_CoordinatesZ()<< " " << (*LocalPoints)[i].get_GlobalID() << " " << (*LocalPoints)[i].get_LocalID() <<  " " << Distance << " " <<  (*LocalPoints)[i].get_CubicWeight();   
         }else{
         outFile << (*LocalPoints)[i].get_CoordinatesX() << "  " << (*LocalPoints)[i].get_CoordinatesY() << " "  << (*LocalPoints)[i].get_CoordinatesZ() << " " << (*LocalPoints)[i].get_GlobalID() << " " << (*LocalPoints)[i].get_LocalID() << "  " << Distance << " " << (*LocalPoints)[i].get_CubicWeight() << endl;
         }
     }
    outFile.close();
}

void LocalPointset::output_EigenMatrixXd(Eigen::MatrixXd* EigenMatrix, string outFilename){
    
    ofstream outFile;
    string directory =  outFilename;
    outFile.open (directory.c_str());
    outFile << (*EigenMatrix);
    outFile.close();  
    
}
  
void LocalPointset::output_EigenMatrix3d(Eigen::Matrix3d* EigenMatrix, string outFilename){
    
    ofstream outFile;
    string directory =  outFilename;
    outFile.open (directory.c_str());
    outFile << (*EigenMatrix);
    outFile.close();  
    
}

void LocalPointset::output_EigenVector3d(Eigen::Vector3d* EigenVector, string outFilename){
    
    ofstream outFile;
    string directory =  outFilename;
    outFile.open (directory.c_str());
    outFile << (*EigenVector);
    outFile.close();  
    
}

void LocalPointset::output_EigenMatrix2d(Eigen::Matrix2d* EigenMatrix, string outFilename){
    
    ofstream outFile;
    string directory =  outFilename;
    outFile.open (directory.c_str());
    outFile << (*EigenMatrix);
    outFile.close();  
    
}

void LocalPointset::output_EigenVector2d(Eigen::Vector2d* EigenVector, string outFilename){
    
    ofstream outFile;
    string directory = outFilename;
    outFile.open (directory.c_str());
    outFile << (*EigenVector);
    outFile.close();  
    
}

void LocalPointset::calc_CubicWeights(double H){
    
    double sqDist = 0; //QuadraticPointToPointDistances
    double weight = 0; 
    for(int i=0; i<=maxPointID; i++){
        sqDist = (*(m_origin->get_PointToPointDistancesMatrix()))(GlobalID_OriginPoint, (*LocalPoints)[i].get_GlobalID());
        weight = 2*pow(sqDist,3)/pow(H,3) - 3*pow(sqDist,2)/pow(H,2) +1;
        (*LocalPoints)[i].set_CubicWeight(weight);    
    } 
}

void LocalPointset::calc_WLS_Plane(){
   
    // WLS:
        // Ansatz matrices: y=X*b+e (e:= error to minimize)
        // b_WLS= arg min(b) sum(e_i)^2  
        // -> calculation: B_WLS=(X_t*W*X)^-1 * X_t*W*y   
        // MEAN CENTERING ERSTMAL NICHT BENUTZT --> evtl später implementieren
    
    double sum_XW, sum_YW, sum_ZW, sum_X2W, sum_Y2W, sum_XYW, sum_XZW, sum_YZW, sum_W;
    sum_XW = sum_YW = sum_ZW = sum_X2W = sum_Y2W = sum_XYW = sum_XZW = sum_YZW = sum_W = 0;
    
    for(int i=0; i<= maxPointID; i++){
        double w = (*LocalPoints)[i].get_CubicWeight();
        sum_W += w;
        sum_XW += (w* (*LocalPoints)[i].get_CoordinatesX());
        sum_YW += (w* (*LocalPoints)[i].get_CoordinatesY());
        sum_ZW += (w* (*LocalPoints)[i].get_CoordinatesZ());
        sum_X2W += (w* pow((*LocalPoints)[i].get_CoordinatesX(),2));
        sum_Y2W += (w* pow((*LocalPoints)[i].get_CoordinatesY(),2));
        sum_XYW += (w* (*LocalPoints)[i].get_CoordinatesX() * (*LocalPoints)[i].get_CoordinatesY());
        sum_XZW += (w* (*LocalPoints)[i].get_CoordinatesX() * (*LocalPoints)[i].get_CoordinatesZ());
        sum_YZW += (w* (*LocalPoints)[i].get_CoordinatesY() * (*LocalPoints)[i].get_CoordinatesZ());
    }
   
    //set matrix 
    (*Matrix3d_1) << sum_X2W, sum_XYW, sum_XW,  sum_XYW, sum_Y2W, sum_YW,   sum_XW, sum_YW, sum_W;
    (*Vector3d_1) << sum_XZW, sum_YZW, sum_ZW;
    //output_EigenMatrix3d(Matrix3d_1, get_NameDataOutput("Plane_WLS_Matrix"));
    //output_EigenVector3d(Vector3d_1, get_NameDataOutput("Plane_WLS_solverY"));
    
    //SOLVE linear equations
    (*Vector3d_2) = (*Matrix3d_1).colPivHouseholderQr().solve((*Vector3d_1));
    (*WLS_Plane).input_WLSsolution(Vector3d_2);
    //(*WLS_Plane).output_ParameterIntoTxtFile(get_NameDataOutput("Plane_WLS_solution"));
    
}

void LocalPointset::calc_LS_Plane(){
   
    double sum_X, sum_Y, sum_Z, sum_X2, sum_Y2, sum_XY, sum_XZ, sum_YZ, sum;
    sum_X = sum_Y = sum_Z = sum_X2 = sum_Y2 = sum_XY = sum_XZ = sum_YZ = sum = 0;
    
    for(int i=0; i<= maxPointID; i++){
        sum;
        sum_X += ((*LocalPoints)[i].get_CoordinatesX());
        sum_Y += ((*LocalPoints)[i].get_CoordinatesY());
        sum_Z += ((*LocalPoints)[i].get_CoordinatesZ());
        sum_X2 += (pow((*LocalPoints)[i].get_CoordinatesX(),2));
        sum_Y2 += (pow((*LocalPoints)[i].get_CoordinatesY(),2));
        sum_XY += ((*LocalPoints)[i].get_CoordinatesX() * (*LocalPoints)[i].get_CoordinatesY());
        sum_XZ += ((*LocalPoints)[i].get_CoordinatesX() * (*LocalPoints)[i].get_CoordinatesZ());
        sum_YZ += ((*LocalPoints)[i].get_CoordinatesY() * (*LocalPoints)[i].get_CoordinatesZ());
    }
   
    //set matrix 
    (*Matrix3d_1) << sum_X2, sum_XY, sum_X,  sum_XY, sum_Y2, sum_Y,   sum_X, sum_Y, sum;
    (*Vector3d_1) << sum_XZ, sum_YZ, sum_Z;
    output_EigenMatrix3d(Matrix3d_1, get_NameDataOutput("Plane_LS_Matrix"));
    output_EigenVector3d(Vector3d_1, get_NameDataOutput("Plane_LS_solverY"));
    
    //SOLVE linear equations
    (*Vector3d_2) = (*Matrix3d_1).colPivHouseholderQr().solve((*Vector3d_1));
    (*WLS_Plane).input_WLSsolution(Vector3d_2);
    (*WLS_Plane).output_ParameterIntoTxtFile(get_NameDataOutput("Plane_LS_solution"));
    
}

void LocalPointset::project_LP_OntoWLSPlane(){
    
    rotate_Pointset_parallelToXY();
    
    //(*WLS_Plane).output_ParameterIntoTxtFile(get_NameDataOutput("Plane_AfterM1"));
    //output_LP_TxtFile(get_NameDataOutput("LP_AfterM1"));
    
    //WLSplane now parallel to XYplane --> use Z axis intercept to project Points onto Plane; all z komponents of LocalPoints - Z axis intercept
    double Z_AxisIntercept = (*WLS_Plane).get_Z_AxisIntercept();
    stored_Z_AxisIntercept_Plane = Z_AxisIntercept;
    
    for(int i=0; i<=maxPointID; i++){
        (*stored_Z_Coordinates_LP)[i] = (*LocalPoints)[i].get_CoordinatesZ();
        (*LocalPoints)[i].set_CoordinatesZ(Z_AxisIntercept);
    }  
}

void LocalPointset::rotate_Pointset_parallelToXY(){
    
    //rotate Localpoints with angle between NormalVector_WLSplane and NormalVector_XYplane 
    (*Vector3d_1) << 0,0,1; //XY plane normalized normal Vector
    (*Vector3d_2) = (*(*WLS_Plane).get_unitNormalVector());
    (*pOutFile_LP) << "rotate_Pointset_parallelToXY: (rotation1)" << endl;
    (*pOutFile_LP) << "unitNormalVector plane: " << (*Vector3d_2) << endl;
    double dot = (*Vector3d_2).dot((*Vector3d_1));
    if( (dot==1) || (dot==-1)){
        (*pOutFile_LP) << "plane already parallel to XY" << endl;
        Theta1=0;
    }else{
        Theta1 = calc_AngleTwoVector3d(Vector3d_2, Vector3d_1);
        (*Vector3d_3) = (*calc_NormalizedRotationAxis(Vector3d_2, Vector3d_1));
        (*Matrix3d_1) = (*calc_RotationMatrix(Vector3d_3, Theta1));
        (*pOutFile_LP) << "Theta1" << endl;
        (*pOutFile_LP) << "angle Theta1 in rad d: " << Theta1 << endl;
        (*pOutFile_LP) << "angle Theta1 in degree: " << Theta1*180/M_PI << endl <<endl;
        (*pOutFile_LP) << "rotationaxis1 " << endl << (*Vector3d_3) << endl;
        (*pOutFile_LP) << "rotaionmatrix1" << endl << (*Matrix3d_1) << endl;
        (*WLS_Plane).rotate(Matrix3d_1);
        rotate_LocalPoints(Matrix3d_1);
        //Abspeichern der inversen Rotation
        (*inverse_Rotation1) = (*Matrix3d_1).inverse();
    }
}

Eigen::Matrix3d* LocalPointset::calc_RotationMatrix(Eigen::Vector3d* normalized_Rotationaxis, double angle){
    
    //Angle Axis Rotation Matrix
    (*Matrix3d_2) = Eigen::AngleAxisd(angle, (*normalized_Rotationaxis));
    return Matrix3d_2;
}

double LocalPointset::calc_AngleTwoVector3d(Eigen::Vector3d* Vector1, Eigen::Vector3d* Vector2){
    
    double a, b, c, d;
    a = (*Vector1).dot((*Vector2)); // dot product nK*nXY 
    if( (a==1) || (a==-1)){ //vektoren schon parallel!
        cout << "ERROR WINKEL" << endl;
    }
    b = (*Vector1).norm() * (*Vector2).norm(); //produkt der beträge der beiden vektoren
    c = a/b; //cos(theta)= (n1*n2)/(|n1|*|n2|)
    d = acos(a/b);
    return d;
}

Eigen::Vector3d* LocalPointset::calc_NormalizedRotationAxis(Eigen::Vector3d* Vector1, Eigen::Vector3d* Vector2){
    
    double a = (*Vector1).dot((*Vector2)); // dot product nK*nXY 
    if( (a==1) || (a==-1)){ //vektoren schon parallel!
         (*pOutFile_LP) << "ERROR ROTATIONSACHSE" << endl;
    }
    (*Vector3d_4) = (*Vector1).cross((*Vector2));
    (*Vector3d_5) = (*Vector3d_4).normalized();
    return Vector3d_5;
}

void LocalPointset::rotate_LocalPoints(Eigen::Matrix3d* rotationmatrix){
    
    for(int i=0; i<=maxPointID; i++){
        (*Vector3d_1) = (*rotationmatrix) * (*LocalPoints)[i].get_CoordinatesXYZ();
        (*LocalPoints)[i].set_CoordinatesXYZ(Vector3d_1);
    }  
}

void LocalPointset::rotate_OriginPoint(Eigen::Matrix3d* rotationmatrix){
    
    (*Vector3d_1) = (*rotationmatrix) * (*LocalPoints)[LocalID_OriginPoint].get_CoordinatesXYZ();
    (*LocalPoints)[LocalID_OriginPoint].set_CoordinatesXYZ(Vector3d_1);
 
}

void LocalPointset::move_OriginOfCoordinateSystem_ToOriginPoint(){
    
    //store Coordinates of Origin Point to reverse this operation later on
    (*stored_CoordinatesOriginPoint) = (*LocalPoints)[LocalID_OriginPoint].get_CoordinatesXYZ();
    (*pOutFile_LP) << "stored_CoordinatesOriginPoint_1:" << endl << (*stored_CoordinatesOriginPoint) << endl;
    for(int i=0; i<=maxPointID; i++){
        (*Vector3d_1) = (*LocalPoints)[i].get_CoordinatesXYZ() - (*stored_CoordinatesOriginPoint);
        (*LocalPoints)[i].set_CoordinatesXYZ(Vector3d_1);
    } 
}

void LocalPointset::calc_WLS_LinearCurve(){
    
    double sum_XW, sum_YW, sum_X2W, sum_XYW, sum_W;
    sum_XW = sum_YW = sum_X2W = sum_XYW = sum_W = 0;
 
    for(int i=0; i<= maxPointID; i++){
        double w = (*LocalPoints)[i].get_CubicWeight();
        sum_W += w;
        sum_XW += (w* (*LocalPoints)[i].get_CoordinatesX());
        sum_YW += (w* (*LocalPoints)[i].get_CoordinatesY());
        sum_X2W += (w* pow((*LocalPoints)[i].get_CoordinatesX(),2));
        sum_XYW += (w* (*LocalPoints)[i].get_CoordinatesX() * (*LocalPoints)[i].get_CoordinatesY());
    }
   
    //set matrix 
    (*Matrix2d_1) << sum_X2W, sum_XW, sum_XW, sum_W;
    (*Vector2d_1) << sum_XYW, sum_YW; 
    //output_EigenMatrix2d(Matrix2d_1, get_NameDataOutput("LinearCurve_WLS_Matrix2D"));
    //output_EigenVector2d(Vector2d_1, get_NameDataOutput("LinearCurve_WLS_solver2D"));
    
    //SOLVE linear equations
    (*Vector2d_2) = (*Matrix2d_1).colPivHouseholderQr().solve((*Vector2d_1));
    (*WLS_LinearCurve).input_WLSsolution(Vector2d_2);
    //(*WLS_LinearCurve).output_ParameterIntoTxtFile(get_NameDataOutput("LinearCurve_WLS_solution2D"));
    
}

void LocalPointset::calc_LS_LinearCurve(){
    
    double sum_X, sum_Y, sum_X2, sum_XY, sum;
    sum_X = sum_Y = sum_X2 = sum_XY = sum = 0;
 
    for(int i=0; i<= maxPointID; i++){
        sum ++;
        sum_X += ((*LocalPoints)[i].get_CoordinatesX());
        sum_Y += ((*LocalPoints)[i].get_CoordinatesY());
        sum_X2 += (pow((*LocalPoints)[i].get_CoordinatesX(),2));
        sum_XY += ((*LocalPoints)[i].get_CoordinatesX() * (*LocalPoints)[i].get_CoordinatesY());
    }
   
    //set matrix 
    (*Matrix2d_1) << sum_X2, sum_X, sum_X, sum;
    (*Vector2d_1) << sum_XY, sum_Y; 
    //output_EigenMatrix2d(Matrix2d_1, get_NameDataOutput("LinearCurve_LS_Matrix2D"));
    //output_EigenVector2d(Vector2d_1, get_NameDataOutput("LinearCurve_LS_solver2D"));
    
    //SOLVE linear equations
    (*Vector2d_2) = (*Matrix2d_1).colPivHouseholderQr().solve((*Vector2d_1));
    (*WLS_LinearCurve).input_WLSsolution(Vector2d_2);
    //(*WLS_LinearCurve).output_ParameterIntoTxtFile(get_NameDataOutput("LinearCurve_LS_solution2D"));
}

void LocalPointset::calc_WLS_LinearCurve3D(){
    
    double sum_XW, sum_YW, sum_X2W, sum_XYW, sum_W;
    sum_XW = sum_YW = sum_X2W = sum_XYW = sum_W = 0;
 
    for(int i=0; i<= maxPointID; i++){
        double w = (*LocalPoints)[i].get_CubicWeight();
        sum_W += w;
        sum_XW += (w* (*LocalPoints)[i].get_CoordinatesX());
        sum_YW += (w* (*LocalPoints)[i].get_CoordinatesY());
        sum_X2W += (w* pow((*LocalPoints)[i].get_CoordinatesX(),2));
        sum_XYW += (w* (*LocalPoints)[i].get_CoordinatesX() * (*LocalPoints)[i].get_CoordinatesY());
    }
   
    //set matrix 
    (*Matrix3d_1) << sum_X2W, sum_XW, 0,  sum_XW, sum_W, 0, 0,0,0;
    (*Vector3d_1) << sum_XYW, sum_YW, 0; 
    //output_EigenMatrix3d(Matrix3d_1, get_NameDataOutput("LinearCurve_WLS_Matrix3D"));
    //output_EigenVector3d(Vector3d_1, get_NameDataOutput("LinearCurve_WLS_solver3D"));
    
    //SOLVE linear equations
    (*Vector3d_2) = (*Matrix3d_1).colPivHouseholderQr().solve((*Vector3d_1));
    //output_EigenVector3d(Vector3d_2, get_NameDataOutput("LinearCurve_WLS_solution3D"));  
}

Point3D LocalPointset::calc_MovedPoint_AfterMLS(){
    
    output_DataDebugging_txt(get_NameDataOutput("calc_MLS"));
    
    //output_LP_TxtFile(get_NameDataOutput("LP_BeforeParallel_ToXaxis"));
    //(*WLS_LinearCurve).output_ParameterIntoTxtFile(get_NameDataOutput("LinearCurve_Param_BeforeParallel_ToXaxis"));
    //(*WLS_LinearCurve).output_FctValues_PointSlopeForm3D_TextFile(-20, 20, 0.1, get_NameDataOutput("LinearCurve_Values_BeforeParallel_ToXaxis"));
    
    //rotate Localpoints that WLS_LinearCurve = 0° to positiv xaxis of coordinate system; WLS_LinearCurve parallel to x-axis;
    (*Vector3d_1) << 1,0,0; //100 positiv x-axis i.e. parallel to 110
    (*Vector3d_2) = (*(*WLS_LinearCurve).get_normalizedDirection());
    (*pOutFile_LP) << "rotate parallel to positiv xaxis 100 of coordinate system (rotation3)" << endl;
    double dot = (*Vector3d_2).dot((*Vector3d_1));
    if( (dot ==1) || (dot==-1)){
        (*pOutFile_LP) << "WLS_LinearCurve already parallel to positiv xaxis 100 of coordinate system" << endl;
        Theta3=0;
    }else{
        Theta3 = calc_AngleTwoVector3d(Vector3d_2, Vector3d_1); //muss für calc_MovedPointAfterMLS() abgespeichert werden!!!
        (*pOutFile_LP) << "Theta3" << endl;
        (*pOutFile_LP) << "angle Theta3 in rad d: " << Theta3 << endl;
        (*pOutFile_LP) << "angle Theta3 in degree: " << Theta3*180/M_PI << endl <<endl;
        (*Vector3d_3) = (*calc_NormalizedRotationAxis(Vector3d_2, Vector3d_1));
        (*normalizedRotationaxis2) = (*Vector3d_3); //muss für calc_MovedPointAfterMLS() abgespeichert werden!!!
        (*Matrix3d_1) = (*calc_RotationMatrix(Vector3d_3, Theta3));  
        (*pOutFile_LP) << "rotationaxis3 " << endl << (*Vector3d_3) << endl;
        (*pOutFile_LP) << "rotationmatrix3 " << endl << (*Matrix3d_1) << endl;
        rotate_LocalPoints(Matrix3d_1);
        (*WLS_LinearCurve).rotate(Matrix3d_1); 
        //Abspeichern der inversen Rotation
        (*inverse_Rotation3) = (*Matrix3d_1).inverse();
    }
    
    //(*WLS_LinearCurve).output_ParameterIntoTxtFile(get_NameDataOutput("LinearCurve_Param_AfterParallel_ToXaxis"));
    //(*WLS_LinearCurve).output_FctValues_PointSlopeForm3D_TextFile(-2, 2, 0.1, get_NameDataOutput("LinearCurve_Values_AfterParallel_ToXaxis"));
    //output_LP_TxtFile(get_NameDataOutput("LP_AfterParallel_ToXaxis"));
      
    //TEST
    //output_LP_TxtFile(get_NameDataOutput("LP_AfterParallel_ToXaxis_TEST"));
    //calc_WLS_LinearCurve();
    //(*WLS_LinearCurve).output_ParameterIntoTxtFile(get_NameDataOutput("LinearCurve_Param_AfterParallel_ToXaxis_TEST"));
    //(*WLS_LinearCurve).output_FctValues_PointSlopeForm3D_TextFile(-2, 2, 0.1, get_NameDataOutput("LinearCurve_Values_AfterParallel_ToXaxis_TEST"));
    //(*pOutFile) << "ERROR_nachNEUberechnung_parallelToXaxis " << calc_AverageApproxmationError_LinearCurve() << endl;
    //(*WLS_LinearCurve).output_ParameterIntoTxtFile(get_NameDataOutput("LinearCurve_Param_AfterParallel_ToXaxis_TEST"));
    //TESTENDE
    
    calc_WLS_QuadraticCurve();
    (*LocalPoints)[LocalID_OriginPoint].set_epsilon(calc_AverageApproxmationError_QuadraticCurve()); //set epsilon for possible next iteration loop
    (*pOutFile_LP) << "AverageApproxmationError for ID: " << (*LocalPoints)[LocalID_OriginPoint].get_GlobalID() << endl;
    (*pOutFile_LP) << (*LocalPoints)[LocalID_OriginPoint].get_epsilon() << endl;
    //(*WLS_QuadraticCurve).output_FctValues_TextFile(-2,8,0.1, get_NameDataOutput("QuadraticCurve_Values_AfterCalculation"));
    //(*WLS_QuadraticCurve).output_ParameterIntoTxtFile(get_NameDataOutput("QuadraticCurve_Param_AfterCalculation"));
    project_OriginPoint_OntoQuadraticCurve();
    
    calc_movedOriginPoint_InDefaultCoordinateSystem();
    
    pOutFile_LP->close();
    return (*LocalPoints)[LocalID_OriginPoint];
}

void LocalPointset::calc_WLS_QuadraticCurve(){
    
    double sum_X4W, sum_X3W, sum_X2W, sum_XW, sum_X2YW, sum_XYW, sum_YW, sum_W;
    sum_X4W = sum_X3W = sum_X2W = sum_XW = sum_X2YW = sum_XYW = sum_YW = sum_W;
    
    for(int i=0; i<= maxPointID; i++){
        double w = (*LocalPoints)[i].get_CubicWeight();
        sum_W += w;
        sum_X4W += (w* pow((*LocalPoints)[i].get_CoordinatesX(),4));
        sum_X3W += (w* pow((*LocalPoints)[i].get_CoordinatesX(),3));
        sum_X2W += (w* pow((*LocalPoints)[i].get_CoordinatesX(),2));
        sum_XW += (w* (*LocalPoints)[i].get_CoordinatesX());   
        sum_X2YW += (w* pow((*LocalPoints)[i].get_CoordinatesX(),2) * (*LocalPoints)[i].get_CoordinatesY());
        sum_XYW += (w* (*LocalPoints)[i].get_CoordinatesX() * (*LocalPoints)[i].get_CoordinatesY());
        sum_YW += (w* (*LocalPoints)[i].get_CoordinatesY());
    }
   
    //set matrix 
    (*Matrix3d_1) << sum_X4W, sum_X3W, sum_X2W,  sum_X3W, sum_X2W, sum_XW,   sum_X2W, sum_XW, sum_W;
    (*Vector3d_1) << sum_X2YW, sum_XYW, sum_YW;
    //output_EigenMatrix3d(Matrix3d_1, get_NameDataOutput("QuadraticCurve_WLS_Matrix2D"));
    //output_EigenVector3d(Vector3d_1, get_NameDataOutput("QuadraticCurve_WLS_solverY2D"));
    
    //SOLVE linear equations
    (*Vector3d_2) = (*Matrix3d_1).colPivHouseholderQr().solve((*Vector3d_1));
    (*WLS_QuadraticCurve).input_WLSsolution(Vector3d_2);
    //(*WLS_QuadraticCurve).output_ParameterIntoTxtFile(get_NameDataOutput("QuadraticCurve_WLS_solution"));  
}

void LocalPointset::calc_LS_QuadraticCurve(){
    
    double sum_X4, sum_X3, sum_X2, sum_X, sum_X2Y, sum_XY, sum_Y, sum;
    sum_X4 = sum_X3 = sum_X2 = sum_X = sum_X2Y = sum_XY = sum_Y = sum=0;
    
    for(int i=0; i<= maxPointID; i++){
        sum ++;
        sum_X4 += (pow((*LocalPoints)[i].get_CoordinatesX(),4));
        sum_X3 += (pow((*LocalPoints)[i].get_CoordinatesX(),3));
        sum_X2 += (pow((*LocalPoints)[i].get_CoordinatesX(),2));
        sum_X += ((*LocalPoints)[i].get_CoordinatesX());   
        sum_X2Y += (pow((*LocalPoints)[i].get_CoordinatesX(),2) * (*LocalPoints)[i].get_CoordinatesY());
        sum_XY += ((*LocalPoints)[i].get_CoordinatesX() * (*LocalPoints)[i].get_CoordinatesY());
        sum_Y += ((*LocalPoints)[i].get_CoordinatesY());
    }
   
    //set matrix 
    (*Matrix3d_1) << sum_X4, sum_X3, sum_X2,  sum_X3, sum_X2, sum_X,   sum_X2, sum_X, sum;
    (*Vector3d_1) << sum_X2Y, sum_XY, sum_Y;
    //output_EigenMatrix3d(Matrix3d_1, get_NameDataOutput("QuadraticCurve_LS_Matrix2D"));
    //output_EigenVector3d(Vector3d_1, get_NameDataOutput("QuadraticCurve_LS_solverY2D"));
    
    //SOLVE linear equations
    (*Vector3d_2) = (*Matrix3d_1).colPivHouseholderQr().solve((*Vector3d_1));
    (*WLS_QuadraticCurve).input_WLSsolution(Vector3d_2);
    //(*WLS_QuadraticCurve).output_ParameterIntoTxtFile(get_NameDataOutput("QuadraticCurve_LS_solution"));
}

void LocalPointset::project_OriginPoint_OntoQuadraticCurve(){
    
    //project point onto (0,D) of quadratic curve
    double u = 0;
    (*LocalPoints)[LocalID_OriginPoint].set_CoordinatesX(0);
    u = (*WLS_QuadraticCurve).get_D();
    (*LocalPoints)[LocalID_OriginPoint].set_CoordinatesY(u);
    //(*LocalPoints)[LocalID_OriginPoint].get_CoordinatesZ() //sollte schon 0 sein
}

double LocalPointset::calc_AverageApproxmationError_LinearCurve(){ //kann nicht nach rotationen verwendet werden. general eq. of line wird nicht rotiert
    
    double sum_PointErrors = 0;
    for(int i=0; i<=maxPointID; i++){
        sum_PointErrors += abs((*LocalPoints)[i].get_CoordinatesY() - (*WLS_LinearCurve).get_y((*LocalPoints)[i].get_CoordinatesX()));
    }
   return sum_PointErrors/N_LocalPoints;
    
}

double LocalPointset::calc_AverageApproxmationError_QuadraticCurve(){  //kann nicht nach rotationen verwendet werden. general eq. of Qline wird nicht rotiert
    double sum_PointErrors = 0;
    for(int i=0; i<=maxPointID; i++){
        sum_PointErrors += abs((*LocalPoints)[i].get_CoordinatesY() - (*WLS_QuadraticCurve).get_y((*LocalPoints)[i].get_CoordinatesX()));
    }
   return sum_PointErrors/N_LocalPoints;
    
}

void LocalPointset::calc_movedOriginPoint_InDefaultCoordinateSystem(){
    
    (*pOutFile_LP) << "rotate LP back to Theta2" << endl;
    if(Theta3 != 0){
    rotate_OriginPoint(inverse_Rotation3);
    }
    if(Theta2 != 0){
    rotate_OriginPoint(inverse_Rotation2);
    }
   
    //+coordinates origon point --> zurück verschieben
    (*pOutFile_LP) << "LP_reverseOriginOfCoordinateSyste" << endl;
    (*Vector3d_1) = (*LocalPoints)[LocalID_OriginPoint].get_CoordinatesXYZ() + (*stored_CoordinatesOriginPoint);
    (*LocalPoints)[LocalID_OriginPoint].set_CoordinatesXYZ(Vector3d_1);
   
    if(Theta1 != 0){
    (*pOutFile_LP) << "inverse_Rotation1" << endl;
    rotate_OriginPoint(inverse_Rotation1);
    }
}


void LocalPointset::calc_LP_InDefaultCoordinateSystem(){
   
    (*pOutFile_LP) << "rotate LP back to Theta2" << endl;
    if(Theta3 != 0){
    rotate_LocalPoints(inverse_Rotation3);
    }
    if(Theta2 != 0){
    rotate_LocalPoints(inverse_Rotation2);
    }
    //output_LP_TxtFile(get_NameDataOutput("LP_reverseToTheta2"));
    
    //+coordinates origon point --> zurück verschieben
    (*pOutFile_LP) << "LP_reverseOriginOfCoordinateSyste" << endl;
    for(int i=0; i<=maxPointID; i++){
        
        (*Vector3d_1) = (*LocalPoints)[i].get_CoordinatesXYZ() + (*stored_CoordinatesOriginPoint);
        (*LocalPoints)[i].set_CoordinatesXYZ(Vector3d_1);
    } 
    //output_LP_TxtFile(get_NameDataOutput("LP_reverseOriginOfCoordinateSyste"));
    
    //z axis intercept
    (*pOutFile_LP) << "LP_reverseZAxisDistances" << endl;
    for(int i=0; i<=maxPointID; i++){
        (*LocalPoints)[i].set_CoordinatesZ((*stored_Z_Coordinates_LP)[i]);  
    }
    //output_LP_TxtFile(get_NameDataOutput("LP_reverseZAxisDistances"));
    
    if(Theta1 != 0){
    (*pOutFile_LP) << "inverse_Rotation1" << endl;
    rotate_LocalPoints(inverse_Rotation1);
    }
    
}

//
//
//
//
//
//functions for ordering:


int LocalPointset::get_ID_first_forwardPoint(double weight_order, double max_gap, double min_gap){
    
    int ID_first_forwardPoint =-1;
    int localID_first_forwardPoint =-1;
    double weight=0;
    double delta_w=0;
    double delta_w_min =1;
    double weight_max=0;
    double dist=0;
    
    output_DataDebugging_txt(get_NameDataOutput("get_ID_first_forwardPoint"));
    
    (*pOutFile_LP) << "LP_radius_H: " << LP_radius_H << endl;
    (*pOutFile_LP) << "calc_CubicWeights: " << LP_radius_H << endl;
    calc_CubicWeights(LP_radius_H);
    //calc_LP_InDefaultCoordinateSystem();
    
    //TEST
    if((m_origin->get_n()) ==3 ){
        
        output_LP_TxtFile(get_NameDataOutput("TPS_check"));
        
    }
    //TEST
    
    (*pOutFile_LP) << "coordinates origin point: " << (*LocalPoints)[LocalID_OriginPoint].get_GlobalID() << "  " << (*(m_origin->get_TriplelinePointsetAfterMLS()))[(*LocalPoints)[LocalID_OriginPoint].get_GlobalID()].get_CoordinatesXYZ() << endl;
    
    (*pOutFile_LP) << "BEGIN FOR LOOP" << endl;
    for(int i=1; i<= maxPointID; i++){
        
        (*pOutFile_LP) << "localID i: " << i << endl;
        (*pOutFile_LP) << "(*LocalPoints)[i].get_GlobalID(): " << (*LocalPoints)[i].get_GlobalID() << endl;
        weight = (*LocalPoints)[i].get_CubicWeight();
        (*pOutFile_LP) << "weight: " << weight << endl;
        (*pOutFile_LP) << "weight2: " << (*LocalPoints)[i].get_CubicWeight() << endl; 
        delta_w = abs(weight-weight_order);
        (*pOutFile_LP) << "delta_w: " << delta_w << endl;
        dist = ((*(m_origin->get_PointToPointDistancesMatrix()))((*LocalPoints)[i].get_GlobalID(), (*LocalPoints)[0].get_GlobalID()));
        
        if( (delta_w < delta_w_min) && (max_gap > dist > min_gap)){
           
            delta_w_min = delta_w;
            (*pOutFile_LP) << "delta_w_min: " << delta_w_min << endl;
            localID_first_forwardPoint = i;
            (*pOutFile_LP) << "localID_first_forwardPoint: " << localID_first_forwardPoint << endl;
            ID_first_forwardPoint = (*LocalPoints)[i].get_GlobalID();   
            (*pOutFile_LP) << "ID_first_forwardPoint: " << ID_first_forwardPoint << endl;
        }
        (*pOutFile_LP) << endl << endl;
    }
    (*pOutFile_LP) << "END FOR LOOP" << endl;
    
    (*pOutFile_LP) << "final ID_first_forwardPoint: " << ID_first_forwardPoint << endl;
    
    if(ID_first_forwardPoint <= 0){
        
        cout << "ERROR final ID_first_forwardPoint" << endl;
    }
    
    if(ID_first_forwardPoint < 0){
        cout << "ERROR get_next_orderedPoint; no ID_first_forwardPoint!!!" << endl;
    }
    
    pOutFile_LP->close();
    return ID_first_forwardPoint;
}

int LocalPointset::get_next_orderedPoint(double weight_order, double max_gap, double min_gap, int ID_2last_orderedPoint){
    
    int ID_next_orderedPoint=-1;
    int localID_next_orderedPoint=0;
    double weight =0;
    double weight_max=0;
    double delta_w=0;
    double delta_w_min =1;
    double dist=0;
    double dist2=0;
   
    if(ID_2last_orderedPoint < 0){
        cout << "ERROR get_next_orderedPoint; no ID_2öast_orderedPoint!!!" << endl;
    }
    
    output_DataDebugging_txt(get_NameDataOutput("get_next_orderedPoint"));
    
    calc_CubicWeights(LP_radius_H);
    (*pOutFile_LP) << "LP_radius_H: " << LP_radius_H << endl;
    (*pOutFile_LP) << "calc_CubicWeights: " << LP_radius_H << endl;
    (*pOutFile_LP) << "N_LocalPoints: " << N_LocalPoints << endl;
    (*pOutFile_LP) << "ID_2last_orderedPoint: " << ID_2last_orderedPoint << endl;
    //calc_LP_InDefaultCoordinateSystem();
    
    if((m_origin->get_n()) ==3 ){
        
        output_LP_TxtFile(get_NameDataOutput("get_next_orderedPoint_LP"));
        
    }
    
    (*pOutFile_LP) << "coordinates origin point: " << (*LocalPoints)[LocalID_OriginPoint].get_GlobalID() << "  " << (*(m_origin->get_TriplelinePointsetAfterMLS()))[(*LocalPoints)[LocalID_OriginPoint].get_GlobalID()].get_CoordinatesXYZ() << endl;
    
    (*pOutFile_LP) << "BEGIN FOR LOOP" << endl << endl;
    for(int i=1; i<= maxPointID; i++){
        
        (*pOutFile_LP) << "localID i: " << i << endl;
        (*pOutFile_LP) << "(*LocalPoints)[i].get_GlobalID(): " << (*LocalPoints)[i].get_GlobalID() << endl;
        (*pOutFile_LP) << "ID_ordered(): " << (*LocalPoints)[i].get_ID_ordered() << endl;
         
        if((*LocalPoints)[i].get_bool_ordered() == false){
            
            weight = (*LocalPoints)[i].get_CubicWeight();
            (*pOutFile_LP) << "weight: " << weight << endl;
            (*pOutFile_LP) << "weight2: " << (*LocalPoints)[i].get_CubicWeight() << endl; 
            delta_w = abs(weight-weight_order);
            (*pOutFile_LP) << "delta_w: " << delta_w << endl;
            dist = ((*(m_origin->get_PointToPointDistancesMatrix()))((*LocalPoints)[i].get_GlobalID(), (*LocalPoints)[LocalID_OriginPoint].get_GlobalID()));
            dist2 = (((*m_origin->get_PointToPointDistancesMatrix()))((*LocalPoints)[i].get_GlobalID(), ID_2last_orderedPoint));
            (*pOutFile_LP) << "dist: " << dist << endl;
            (*pOutFile_LP) << "dist2: " << dist2 << endl;
            
            if( ((delta_w < delta_w_min) && (max_gap > dist > min_gap)) && (check_direction_dotProduct(i, ID_2last_orderedPoint) == true) && (dist2 > dist)){
            
                delta_w_min = delta_w;
                (*pOutFile_LP) << "delta_w_min: " << delta_w_min << endl;
                localID_next_orderedPoint = i;
                (*pOutFile_LP) << "localID_next_orderedPoint: " << localID_next_orderedPoint << endl;
                ID_next_orderedPoint = (*LocalPoints)[i].get_GlobalID();   
                (*pOutFile_LP) << "ID_next_orderedPoint: " << ID_next_orderedPoint << endl;
            }
            (*pOutFile_LP) << endl << endl;
        }
    }
   
    (*pOutFile_LP) << "final ID_next_orderedPoint: " <<  ID_next_orderedPoint << endl << endl;
   
    pOutFile_LP->close();
    return ID_next_orderedPoint;
}

bool LocalPointset::check_direction_dotProduct(int ID_newP, int ID_2last_lastP){ //spitzer winkel skalarprodukt > 0; stumpfer winkel < 0
    
    (*pOutFile_LP) << "check_direction_dotProduct ID_newP: " << ID_newP << endl;
    
    bool check = false;
    double dot = 0;
    
    (*pOutFile_LP) << "(*LocalPoints)[ID_newP].get_CoordinatesXYZ(): " << (*LocalPoints)[ID_newP].get_CoordinatesXYZ() << endl;
            
    (*pOutFile_LP) << "(*LocalPoints)[LocalID_OriginPoint].get_CoordinatesXYZ(): " << (*LocalPoints)[LocalID_OriginPoint].get_CoordinatesXYZ() << endl;
    (*pOutFile_LP) << "ID_2last_lastP: " << ID_2last_lastP << endl;
    (*pOutFile_LP) << "(*(m_origin->get_TriplelinePointsetAfterMLS()))[ID_2last_lastP].get_CoordinatesXYZ(): " << (*(m_origin->get_TriplelinePointsetAfterMLS()))[ID_2last_lastP].get_CoordinatesXYZ() << endl;
    
    
    (*Vector3d_1) = (*LocalPoints)[LocalID_OriginPoint].get_CoordinatesXYZ() - (*(m_origin->get_TriplelinePointsetAfterMLS()))[ID_2last_lastP].get_CoordinatesXYZ(); 
    (*Vector3d_2) = (*LocalPoints)[ID_newP].get_CoordinatesXYZ() - (*LocalPoints)[LocalID_OriginPoint].get_CoordinatesXYZ();
    
    (*pOutFile_LP) << "(*Vector3d_1): " << (*Vector3d_1) << endl;
    (*pOutFile_LP) << "(*Vector3d_2): " << (*Vector3d_2) << endl;
    
    dot = (*Vector3d_1).dot((*Vector3d_2));
    
    (*pOutFile_LP) << "dot: " << dot << endl;
    
    if(dot >= 0){
        
        check = true;   
    }
    
    (*pOutFile_LP) << "return check: " << check << endl; 
    return check;
}

void LocalPointset::test_function(){
    
    /*
    
    //angle rotation tests:
    
    (*Vector3d_1) << 1,0,0;
    (*Vector3d_2) << 0.434569 ,-0.90063 ,0;
    cout << "V1:    " << endl << (*Vector3d_1) << endl;
    cout << "V2:    " << endl << (*Vector3d_2) << endl;
    
    double angle = calc_AngleTwoVector3d(Vector3d_1, Vector3d_2);
    (*Vector3d_3) = (*calc_NormalizedRotationAxis(Vector3d_1, Vector3d_2));
    cout << "Ra:    " << endl << (*Vector3d_3) << endl;
    
    (*Matrix3d_1) = (*calc_RotationMatrix(Vector3d_3, angle));
    cout << "M:    " << endl << (*Matrix3d_1) << endl;
    
    (*Vector3d_4) = (*Matrix3d_1)*(*Vector3d_1);
    (*Vector3d_5) = (*Matrix3d_1)*(*Vector3d_2);
    
    cout << "V1rot+angle:    " << endl << (*Vector3d_4) << endl;
    cout << "V2rot+angle:    " << endl << (*Vector3d_5) << endl;
*/
     
 }
