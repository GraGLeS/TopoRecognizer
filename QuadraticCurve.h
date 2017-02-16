
/* 
 * File:   QuadraticCurve.h
 * Author: Arbeit
 * Created on 18. Dezember 201
 */

#ifndef QUADRATICCURVE_H
#define QUADRATICCURVE_H

#include <Eigen/Dense>

using namespace std;

class QuadraticCurve {

public:
    QuadraticCurve();
    QuadraticCurve(const QuadraticCurve& orig);
    virtual ~QuadraticCurve();
    void output_ParameterIntoTxtFile(string);
    void input_WLSsolution(Eigen::Vector3d*);
    double get_y(double); // get y(x) value; y= Ax^2+Bx+D
    void output_FctValues_TextFile(double, double, double, string);
    
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
    
    
private:

    // general equation of a QuadraticCurve 2D: Ax^2+Bx+Cy+D=0; WLS C=-1 --> y=Ax^2+Bx+D
    double A,B,C,D;
};

#endif /* QUADRATICCURVE_H */

