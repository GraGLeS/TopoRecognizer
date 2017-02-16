
/* 
 * File:   Spline.h
 * Author: Arbeit
 * Created on 18. Dezember 2016
 * https://eigen.tuxfamily.org/dox/unsupported/classEigen_1_1Spline.html
 * https://eigen.tuxfamily.org/dox/unsupported/structEigen_1_1SplineFitting.html
 */

#ifndef SPLINE_H
#define SPLINE_H

#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

using namespace std;

typedef Eigen::Spline<double, 3, 1> Spline3D_deg1;
typedef Spline3D_deg1::PointType PointType3D_deg1;
typedef Spline3D_deg1::KnotVectorType KnotVectorType3D_deg1;
typedef Spline3D_deg1::ControlPointVectorType ControlPointVectorType3D_deg1;
    
typedef Eigen::Spline<double, 3, 2> Spline3D_deg2;
typedef Spline3D_deg2::PointType PointType3D_deg2;
typedef Spline3D_deg2::KnotVectorType KnotVectorType3D_deg2;
typedef Spline3D_deg2::ControlPointVectorType ControlPointVectorType3D_deg2;
    
typedef Eigen::Spline<double, 3, 3> Spline3D_deg3;
typedef Spline3D_deg3::PointType PointType3D_deg3;
typedef Spline3D_deg3::KnotVectorType KnotVectorType3D_deg3;
typedef Spline3D_deg3::ControlPointVectorType ControlPointVectorType3D_deg3;   

class Point3D;

class Spline {

public:
    Spline();
    Spline(int);
    ~Spline();
    
    void input_Pointset(vector<Point3D>*);
    
    void interpolate_BSpline_3D_Degree_1_Default();
    void interpolate_BSpline_3D_Degree_2_Default();
    void interpolate_BSpline_3D_Degree_2_Chordlenghts();
    //void interpolate_BSpline_3D_Degree_2_Chordlenghts_KnotAveraging();
    void interpolate_BSpline_3D_Degree_3_Default();
 
    double calc_Line_Lenght_ordered_PointInput();
    double calc_Line_Lenght_Spline();
    
private:

    void output_PointMatrix_TxtFile(Eigen::MatrixXd*, string);
    void output_BSpline_3D_Degree_1_Knots_Ctrls_TxtFile(KnotVectorType3D_deg1&, ControlPointVectorType3D_deg1&, string);
    void output_BSpline_3D_Degree_2_Knots_Ctrls_TxtFile(KnotVectorType3D_deg2&, ControlPointVectorType3D_deg2&, string);
    void output_BSpline_3D_Degree_3_Knots_Ctrls_TxtFile(KnotVectorType3D_deg3&, ControlPointVectorType3D_deg3&, string);
    
    Eigen::MatrixXd* ordered_Pointset;
    
    
    //spline output
    Eigen::MatrixXd* Spline_Points_output;
    double delta_param;
    int N_Spline;
    
    //rechnenmatrizen:
    Eigen::Vector3d* Vector3d_1;
};

#endif /* SPLINE_H */

