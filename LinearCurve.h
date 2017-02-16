
/* 
 * File:   LinearCurve.h
 * Author: Arbeit
 * Created on 18. Dezember 2016
 */

#ifndef LINEARCURVE_H
#define LINEARCURVE_H

#include <Eigen/Dense>

using namespace std;

class LinearCurve {
public:
    LinearCurve();
    LinearCurve(const LinearCurve& orig);
    virtual ~LinearCurve();
    void input_WLSsolution(Eigen::Vector2d*);
    void output_ParameterIntoTxtFile(string);
    void rotate(Eigen::Matrix3d*);
    void output_FctValues_PointSlopeForm3D_TextFile(double, double, double, string); //Punkt-Richtungsform; ~Point-Direction-Form ; Point-Slope-Form; parameterform; 3D; 
    void output_FctValues_SlopeInterceptForm2D_TextFile(double, double, double, string); //y=Ax+B // kann nicht nach ROTATION in 3D benutzt werden !!!
    double get_y(double);
    
    inline Eigen::Vector3d* get_normalizedDirection(){
        return normalizedDirection;
    }
    
     
private:
    // general equation of a line 2D: Ax+By+C=0; WLS B=-1 --> y=Ax+C 
    double A,B,C;
    
    
    //Punkt-Richtungsform; ~Point-Direction-Form ; Point-Slope-Form; parameterform; vec_P2 = Vec_CurvePoint + t* Vec_Direction
    void calc_PointSlopeForm3D();
    Eigen::Vector3d* normalizedDirection;
    Eigen::Vector3d* curvePoint;
    
    Eigen::Vector3d* Vector3d_1;

};

#endif /* LINEARCURVE_H */

