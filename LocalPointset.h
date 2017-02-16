
/* 
 * File:   LocalPointset.h
 * Author: Arbeit
 * Created on 18. Dezember 2016
 */

#ifndef LOCALPOINTSET_H
#define LOCALPOINTSET_H

#include <vector>
#include <Eigen/Dense>

class Point3D;
class TriplelinePointsetClass;
class Plane;
class LinearCurve;
class QuadraticCurve;

using namespace std;

class LocalPointset{

public:
    LocalPointset();
    LocalPointset(TriplelinePointsetClass*);
    ~LocalPointset();
    void clear();
    double calc_Correlation_LP(double);
    Point3D calc_MovedPoint_AfterMLS();
    
    //functions for ordering:
        
        int get_ID_first_forwardPoint(double, double, double);
        int get_next_orderedPoint(double, double, double, int);
        
    void test_function();
    
    
    void output_LP_TxtFile(string);
    void output_EigenMatrixXd(Eigen::MatrixXd*, string);
    void output_EigenMatrix3d(Eigen::Matrix3d*, string);
    void output_EigenVector3d(Eigen::Vector3d*, string);
    void output_EigenMatrix2d(Eigen::Matrix2d*, string);
    void output_EigenVector2d(Eigen::Vector2d*, string);
    
    void set_OriginPoint(int, vector<Point3D>*); // on vector position 0
    void set_LocalPoint(int, vector<Point3D>*);
    
    /*
    inline Point3D get_LocalPoint(int localID){
        return (*LocalPoints)[localID];
    }  
    */
      
    inline vector<Point3D>* get_LocalPoints(){
        return LocalPoints;
    }
    
    inline int get_N_LocalPoints(){
        N_LocalPoints = (*LocalPoints).size();
        return N_LocalPoints;
    }
    
    inline int get_allocatedVectorsize(){
        return allocatedVectorsize;
    }
    
    inline void set_N_LocalPoints(int N_LocalPoints){
        this->N_LocalPoints = N_LocalPoints;
    }
 
    inline int get_GlobalID_OriginPoint(){
        return GlobalID_OriginPoint;
    }
    
    inline void set_LP_radius_H(double LP_radius_H){
        this->LP_radius_H = LP_radius_H;
    }
 
    inline double get_LP_radius_H(){
        return LP_radius_H;
    }
    
    
private:
        
    void calc_PointToPointDistances(string); //nur zu tests
    
        
    //functions for smoothing    
        //calc_CorrelationOfLocalPointset:
        void calc_CubicWeights(double);
        void calc_WLS_Plane();
        void calc_LS_Plane();
        void project_LP_OntoWLSPlane();
            void rotate_Pointset_parallelToXY();
            double calc_AngleTwoVector3d(Eigen::Vector3d*, Eigen::Vector3d*);
            Eigen::Vector3d* calc_NormalizedRotationAxis(Eigen::Vector3d*, Eigen::Vector3d*);
            Eigen::Matrix3d* calc_RotationMatrix(Eigen::Vector3d*, double);
            void rotate_LocalPoints(Eigen::Matrix3d*);
        double calc_rho();
        void move_OriginOfCoordinateSystem_ToOriginPoint();
        void calc_WLS_LinearCurve();
        void calc_LS_LinearCurve();
        void calc_WLS_LinearCurve3D();
        void rotate_Pointset_parallelTo_110();
            //EVTL orthogonal Regression line ???
            
        //calc_MovedPointAfterMLS:
        void calc_WLS_QuadraticCurve();
        void calc_LS_QuadraticCurve();
        void project_OriginPoint_OntoQuadraticCurve();
        double calc_AverageApproxmationError_LinearCurve();
        double calc_AverageApproxmationError_QuadraticCurve();
        
        //reverse operations:
        void calc_movedOriginPoint_InDefaultCoordinateSystem();
        void rotate_OriginPoint(Eigen::Matrix3d*);
        void calc_LP_InDefaultCoordinateSystem();
    
    
    //functions for ordering:
        bool check_direction_dotProduct(int, int);
        
    
    //class objects and vectors:
    vector<Point3D>* LocalPoints; //more cache efficient than fetching from global vector; punkte kopieren anstatt aus globalem set immer wieder auszulesen
    Plane* WLS_Plane;
    LinearCurve* WLS_LinearCurve;
    QuadraticCurve* WLS_QuadraticCurve;
    
    TriplelinePointsetClass* m_origin; //Pointer auf Überklasse TripleLinePoinset
    
    //global variables for txt file output name declarations:
    void output_DataDebugging_txt(string);
    string get_NameDataOutput(string);
    ofstream* pOutFile_LP;
    
    
    
    //global variables:
    int allocatedVectorsize; //allocated vectorsize of LocalPoints; in constructor!
    int GlobalID_OriginPoint;
    int LocalID_OriginPoint;
    int N_LocalPoints; //current number of Local Points
    int maxPointID; //current ID of the last LocalPoint in LocalPointsVector used as a loop count
    double LP_radius_H;
    
    
    //Rechenmatrizen
    Eigen::Vector3d* Vector3d_1  = NULL;
    Eigen::Vector3d* Vector3d_2  = NULL;
    Eigen::Vector3d* Vector3d_3  = NULL;
    Eigen::Vector3d* Vector3d_4  = NULL;
    Eigen::Vector3d* Vector3d_5  = NULL;
    
    Eigen::Matrix3d* Matrix3d_1  = NULL;
    Eigen::Matrix3d* Matrix3d_2  = NULL;
    
    Eigen::Vector2d* Vector2d_1  = NULL;
    Eigen::Vector2d* Vector2d_2  = NULL;
    Eigen::Vector2d* Vector2d_3  = NULL;
    
    Eigen::Matrix2d* Matrix2d_1  = NULL;
    Eigen::Matrix2d* Matrix2d_2  = NULL;
    
    
    //stored informations to reverse certain operations later on: 
    Eigen::Matrix3d* inverse_Rotation1; //in function void project_LocalPointsetOntoPlane())
    double stored_Z_AxisIntercept_Plane; //from project_LocalPointsetOntoWLSPlane()
    vector<double>* stored_Z_Coordinates_LP;
    Eigen::Vector3d* stored_CoordinatesOriginPoint; //stored Coordinates of Origin Point to reverse move_OriginOfCoordinateSystemToOriginPoint() later on
    double Theta1,Theta2,Theta3;
    Eigen::Vector3d* normalizedRotationaxis2; // from function: calc_CorrelationOfLocalPointset(double H); //muss für calc_MovedPointAfterMLS() abgespeichert werden!!!
    Eigen::Matrix3d* inverse_Rotation2;
    Eigen::Matrix3d* inverse_Rotation3;
    
     
};

#endif /* LOCALPOINTSET_H */

