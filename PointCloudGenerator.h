
/* 
 * File:   PointCloudGenerator.h
 * Author: Arbeit
 *
 * Created on 21. Januar 2017
 */

#ifndef POINTCLOUDGENERATOR_H
#define POINTCLOUDGENERATOR_H

#include <vector>
#include <Eigen/Dense>

class Point3D;

using namespace std;

class PointCloudGenerator {

public:
    PointCloudGenerator();
    PointCloudGenerator(vector<Point3D>*);
    virtual ~PointCloudGenerator();
    
    void generate_PointCloud();
 
    
    void calc_FunctionPoints();
    void input_FunctionPoints_OutOfTxtFile(string inFilename);
    void output_FunctionPointsIntoTxtFile();
    void calc_RandomPointCloud();
    
    double check_DistancesToFunction(Eigen::Vector3d*);
    double calc_minDistanceToOtherRandomPoints(Eigen::Vector3d*, int);
    
private:

    
    //PointCloudGenerator()
    vector<Point3D>* FunctionPoints;
    vector<Point3D>* PointCloud;
    int N_Fct;
    int maxFctPointID;
    int maxPointID;
    double x_min, x_max;
    double x_min_box, x_max_box, y_min_box, y_max_box, z_min_box, z_max_box;// RandomPoint spawn box borders; max zwei nachkomma stellen wegen RND funktion die nur für integer gilt !!! wird durch 100 geteilt !!!!
    double allowed_maxDistance_toFunction,  allowed_minDistance_toOtherRandomPoints;
    
    //Rechenmatrizen
    Eigen::Vector3d* Vector3d_1;
    Eigen::Vector3d* Vector3d_2;
    
};

#endif /* POINTCLOUDGENERATOR_H */

