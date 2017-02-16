
/* 
 * File:   PointCloudGenerator.cpp
 * Author: Arbeit
 * 
 * Created on 21. Januar 2017
 */

#include <iostream>
#include <fstream>
//#include <cstdio>
//#include <string>
//#include <cstdlib>
//#include <math.h>

#include "Point3D.h"
#include "PointCloudGenerator.h"

PointCloudGenerator::PointCloudGenerator() {

    /*
//a)
    //TriplelinePointset generator config:
        //i)intern function:
            
            N_Fct = 800;
            maxFctPointID = 0;
            maxPointID = 0;
            FunctionPoints = new vector<Point3D>(N_Fct);
            x_min = -2;
            x_max = 4;
        //ii)
          //input_FunctionValues_OutOfTxtFile(string inFilename)  
          // N_Fct 
            
       
        //RandomPoint spawn box borders:
        x_min_box=-3;; // max zwei nachkomma stellen wegen RND funktion die nur für integer gilt !!! wird durch 100 geteilt !!!!
        x_max_box=6;
        y_min_box=-1;
        y_max_box=20;
        z_min_box=0;
        z_max_box=0;
        
        allowed_maxDistance_toFunction = 20;  
        allowed_minDistance_toOtherRandomPoints = 0.1;
        
        Vector3d_1 = new Eigen::Vector3d;
        Vector3d_2 = new Eigen::Vector3d;
        
*/
}

PointCloudGenerator::PointCloudGenerator(vector<Point3D>* store_PointCloud) {

    PointCloud = store_PointCloud;
    
    //TriplelinePointset generator config:
        //i)intern function:
            N_Fct = 800;
            maxFctPointID = N_Fct-1;
            maxPointID = (*PointCloud).size()-1;
            FunctionPoints = new vector<Point3D>(N_Fct);
            x_min = -2;
            x_max = 4;
        //ii)
          //input_FunctionValues_OutOfTxtFile(string inFilename)  
          // N_Fct 
            
       
        //RandomPoint spawn box borders:
        x_min_box=-5;; // max zwei nachkomma stellen wegen RND funktion die nur für integer gilt !!! wird durch 100 geteilt !!!!
        x_max_box=8;
        y_min_box=-5;
        y_max_box=25;
        z_min_box=-5;
        z_max_box=-8;
        
        allowed_maxDistance_toFunction = 0.5;  
        allowed_minDistance_toOtherRandomPoints = 0.5;
        
        Vector3d_1 = new Eigen::Vector3d;
        Vector3d_2 = new Eigen::Vector3d;

}

PointCloudGenerator::~PointCloudGenerator() {

    delete Vector3d_1;
    delete Vector3d_2;

    //delete [] FunctionPoints;
}

void PointCloudGenerator::generate_PointCloud(){
    
    calc_FunctionPoints();
    //input_FunctionValues_OutOfTxtFile(string inFilename)
    output_FunctionPointsIntoTxtFile();
    
    calc_RandomPointCloud();
    
}

void PointCloudGenerator::output_FunctionPointsIntoTxtFile(){
    
    double x, y, z;
    ofstream outFile;
    string directory = "D:\\masterarbeit\\NetBeansProjects\\Masterarbeit_FW\\output\\PointCloudGenerator_FunctionPoints";
    outFile.open (directory.c_str());
    outFile << "N_Fct: " << N_Fct << endl;
    outFile << "function: y=a*x^2+b; z=c*x+d" << endl;
    outFile << "X" << "    " << "Y" << "   "  << "Z" << "   " << "GLobalID" << endl;  //Header
      
    for(int i=0; i<=maxFctPointID; i++){
         if(i == maxFctPointID){
         outFile << (*FunctionPoints)[i].get_CoordinatesX() << "  " << (*FunctionPoints)[i].get_CoordinatesY() << " "  << (*FunctionPoints)[i].get_CoordinatesZ()<< " " << i;   
         }else{
         outFile << (*FunctionPoints)[i].get_CoordinatesX() << "  " << (*FunctionPoints)[i].get_CoordinatesY() << " "  << (*FunctionPoints)[i].get_CoordinatesZ() << " " << i << endl;
         }
     }
     outFile.close();  
}

void PointCloudGenerator::calc_FunctionPoints(){
    
    double a,b,c,d; //y=ax^2+b; z = c*x +d
    a=1;
    b=0;
    c=1;
    d=0;
    double x = x_min;
    double delta_x;
    delta_x = (x_max-x_min)/(N_Fct-1);
    
    for(int i=0; i<= maxFctPointID; i++){
        (*FunctionPoints)[i].set_CoordinatesX(x); 
        (*FunctionPoints)[i].set_CoordinatesY(a * pow(x, 2) + b); 
        (*FunctionPoints)[i].set_CoordinatesZ(0); //c * x + d
        x += delta_x;
    }
    //output_internFunctionIntoTxtFile();
}

void PointCloudGenerator::input_FunctionPoints_OutOfTxtFile(string inFilename){
    
    
    
}

void PointCloudGenerator::calc_RandomPointCloud(){

    int count = 0;
    int loopcount = 0;
    int loopMAX=1000;
    double Distance_toFunction = allowed_maxDistance_toFunction+1;
    double Distance_toOtherRandomPoints = allowed_minDistance_toOtherRandomPoints -1;
    
    while(count <= maxPointID){
        
        (*Vector3d_1)(0) = (double)(rand() % ((int)(fabs(x_max_box - x_min_box))*100+1))/100+(x_min_box);
        (*Vector3d_1)(1) = (double)(rand() % ((int)(fabs(y_max_box - y_min_box))*100+1))/100+(y_min_box);
        (*Vector3d_1)(2) = 0;//(double)(rand() % ((int)(fabs(z_max_box - z_min_box))*100+1))/100+(z_min_box);
        
        Distance_toFunction = check_DistancesToFunction(Vector3d_1);
        Distance_toOtherRandomPoints = calc_minDistanceToOtherRandomPoints(Vector3d_1, count);
        if((Distance_toFunction < allowed_maxDistance_toFunction) && (Distance_toOtherRandomPoints > allowed_minDistance_toOtherRandomPoints)){
            (*PointCloud)[count].set_CoordinatesXYZ(Vector3d_1);
            (*PointCloud)[count].set_GlobalID(count);
            count ++;
        }
        loopcount++;
        if(loopcount > loopMAX){
            loopMAX +=1000;
            allowed_minDistance_toOtherRandomPoints -= 0.05;
            cout << "allowed_minDistance_toOtherRandomPoints: " << allowed_minDistance_toOtherRandomPoints << endl;
        }    
    }
    cout << "loopcount: " << loopcount << endl;
}

double PointCloudGenerator::check_DistancesToFunction(Eigen::Vector3d* RandomPoint){
 
    double d;
    for(int i=0; i<=maxFctPointID; i++){
    (*Vector3d_2) = (*RandomPoint) - (*FunctionPoints)[i].get_CoordinatesXYZ();
    d = (*Vector3d_2).norm();
        if(d < allowed_maxDistance_toFunction){
           break;
        }  
    }
    return d;
}
    
double PointCloudGenerator::calc_minDistanceToOtherRandomPoints(Eigen::Vector3d* RandomPoint, int N_generatedRandomPoints){
    
    double d;
    if(N_generatedRandomPoints==0){
        d=allowed_minDistance_toOtherRandomPoints+1;
    }else{
        for(int i=0; i<=(N_generatedRandomPoints-1); i++){
            (*Vector3d_2) = (*RandomPoint) - (*PointCloud)[i].get_CoordinatesXYZ();
            d = (*Vector3d_2).norm();
            if(d < allowed_minDistance_toOtherRandomPoints){
                break;
            }  
        }    
    }
    return d;
}

