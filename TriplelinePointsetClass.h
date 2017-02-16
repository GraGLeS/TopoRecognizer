
/* 
 * File:   TriplelinePointsetClass.h
 * Author: Arbeit
 *
 * Created on 18. Dezember 2016
 */

#ifndef TRIPLELINEPOINTSETCLASS_H
#define TRIPLELINEPOINTSETCLASS_H

#include <vector>
#include <Eigen/Dense>

class Point3D;
class LocalPointset;
class Spline;

using namespace std;

class  TriplelinePointsetClass{
public:
    
    
    TriplelinePointsetClass();
    TriplelinePointsetClass(const Eigen::MatrixXd*, int);
    TriplelinePointsetClass(const vector<Point3D>*);
    ~TriplelinePointsetClass();
  
    void use_PointCloudGenerator(int);
   
    void set_Parameter();
    void process_TriplelinePointset(); // const oder nicht const... leiber neue matrizen erstellen ??
    
    
    inline Eigen::MatrixXd* get_PointToPointDistancesMatrix(){
        return PointToPointDistances;
    }
   
    inline int get_N_TriplelinePoints(){
        return N;
    }

    inline vector<Point3D>* get_TriplelinePointsetBeforeMLS(){
        return TPS_BeforeMLS;
    }
   
    inline vector<Point3D>* get_TriplelinePointsetAfterMLS(){
        return TPS_AfterMLS;
    }
    
    
    inline int get_n(){
        return n;
    }
    
  
    inline int get_count(){
        return count;
    }
    
    inline int get_iterationcount(){
        return iterationscount;
    }
    
    
private:
    
    //output functions:
    void input_TPS_OutOfTxtFile(string);
    void output_internFunctionIntoTxtFile();
    void output_TPS_IntoTxtFile(vector<Point3D>*, string, int);
    void output_EigenMatrixXd(Eigen::MatrixXd*, string);
    void output_LP_Object_IntoTxtFile(double, double);
    void output_DataDebugging_txt(string);
    
    //process_TriplelinePointset():
    //
    //MLS:
    void calc_PointToPointDistances(vector<Point3D>*);
    void smoothe_TPS();
    void calc_MLS_Iteration();
    void calc_Sufficient_LP_forMLS(int);
    void calc_NextNeighborSet_RadiusH(double, vector<Point3D>*);
    void switch_Buffer(); //nach einer iteration über das komplette Punktset. output des vorherigen schrittes als input des nächsten setzen
    
    //Ordering:
    void orderAndReduce_TPS();
        void ordering_dynamicDistances();
        int select_startPoint(int);
        int calc_RandomID(int);
        void calc_Sufficient_LP_forOrdering(int);
        int calc_Next_Neighbor_ID(int);
        double calc_Next_Neighbor_Distance(int);
        void order_Neighborhood_method1(int);
            void order_direction_forward(int);
            int check_existing_neighbor(int);
            bool check_direction_dotProduct(int, int);
            void order_direction_backward(int);
            void copy_Buffer_ReducedAndOrdered();
    
    //Spline:
    void calc_BSpline_ProcessedTPS();  
    
    
    //variables+containers:
    //MovingLeastSquares:= MLS
    vector<Point3D>* TPS_BeforeMLS;
    vector<Point3D>* TPS_AfterMLS;
    vector<Point3D>* TPS_Ordered_Forward_Buffer;
    vector<Point3D>* TPS_Ordered_Backward_Buffer;
    vector<Point3D>* TPS_processed;
    
    
    Eigen::MatrixXd* BoolListPointsMovedInMLSIteration; //Information dass Punkt gemoved wurde speichern!!!! //vermerken ob punkt überhaupt berechnet wurde
    Eigen::MatrixXd* AverageApproxmationError;
    Eigen::MatrixXd* PointToPointDistances; //distances r=|deltaP|
    
    LocalPointset* LP_Object; //statisch alloc mit vorgegebener Vectorgröße
    Spline* Spline_Object;
    
    //global variables for txt file output name declarations:
    ofstream* pOutFile;
    int count; //set in calc_SufficientLocalPointsetForLoopPoint
    int iterationscount; //set in calc_SufficientLocalPointsetForLoopPoint
  
    int n0; //vorgesehene Anzahl Schleifendurchläufe über das gesamte Triplelinepunktset
    int n; //aktueller Schleifendurchlauf
    double H0;
    int N; //Punktanzahl im TriplelinePointsetInput
    int maxPointID;
    int Nmin; //mindestanzahl Punkte in lokaler regression bzw local point set A
    double dH; // increment H
    double rho0;
    double epsilon0; //prescribed  average approximation error to quatratic regression line during moving least squares 
    
    //global variables ordering:
    double rho_order;
    double weight_order;
    double min_gap; 
    double max_gap;
    int ID_starting_point;
    int ID_last_point;
    int ID_ordered_forward;
    int ID_ordered_backward;
    
    //global variables spline:
    double lenght_generatorFct;
    double lenght_spline;
   
    //Rechenmatrizenm
    Eigen::Vector3d* Vector3d_1;
    Eigen::Vector3d* Vector3d_2;
    
 
};

#endif /* NEWCLASS_H */

