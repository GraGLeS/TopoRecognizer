
/* 
 * File:   Plane.h
 * Author: Arbeit
 * Created on 18. Dezember 2016
 */

#ifndef PLANE_H
#define PLANE_H

#include <Eigen/Dense>

using namespace std;


class Plane {
public:
    Plane();
    Plane(const Plane& orig);
    virtual ~Plane();
    
    void input_WLSsolution(Eigen::Vector3d*);
    void output_ParameterIntoTxtFile(string);
    void calc_unitNormalVector();
    void rotate(Eigen::Matrix3d*);
    
    
    inline double get_A(){
        return A;
    }
    
    inline double get_B(){
        return B;
    }
    
    inline double get_C(){
        return C;
    }
    
    inline double get_D(){
        return D;
    }
    
    inline Eigen::Vector3d* get_unitNormalVector(){
        return unitNormalVector;
    }
    
    inline double get_P(){
        return P;
    }
    
    inline double get_X_AxisIntercept(){
        X_AxisIntercept = -P/(*unitNormalVector)(0);   
        return X_AxisIntercept;
    }
    
    inline double get_Y_AxisIntercept(){
        Y_AxisIntercept = -P/(*unitNormalVector)(1);      
        return Y_AxisIntercept;
    }
    
    inline double get_Z_AxisIntercept(){
        Z_AxisIntercept = -P/(*unitNormalVector)(2); 
        return Z_AxisIntercept;
    }
    
    inline double get_DistanceToOrigin(){
        DistanceToOrigin = P/(*unitNormalVector).norm(); 
        return DistanceToOrigin;
    }
    
private:

 
    // general equation of a plane: Ax+By+Cz+D=0
    double A,B,C,D;
    
    //Hessian Normal Form: n*x=-p
    void calc_HessianNormalForm();
    void calc_DistanceToOrigin();
    void calc_AxisIntercepts();
    Eigen::Vector3d* unitNormalVector;
    double P;
    
    double X_AxisIntercept, Y_AxisIntercept, Z_AxisIntercept;
    double DistanceToOrigin;
    
    Eigen::Vector3d* Vector3d_1;
};

#endif /* PLANE_H */

