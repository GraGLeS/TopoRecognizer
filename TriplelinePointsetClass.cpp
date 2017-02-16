
/* 
 * File:   TriplelinePointsetClass.cpp
 * Author: Arbeit
 * Created on 18. Dezember 2016
 */

//#include <cstdio>
#include <iostream>
#include <fstream>
//#include <string>
//#include <cstdlib>
//#include <math.h>
//#include <Eigen/Geometry>

#include "TriplelinePointsetClass.h"
#include "Spline.h"
#include "PointCloudGenerator.h"
#include "LocalPointset.h"
#include "Point3D.h"

TriplelinePointsetClass::TriplelinePointsetClass() {
    
    
    //input:
    use_PointCloudGenerator(100);
    
    //b)
    //N=1000 
    //input TriplelinePointset from file
    //input_PointsetOutOfTxtFile("PC1");
    
    //TriplelinePointset process parameter:
    //Smoothing:
    n0=5; 
    n=0;
    H0=2; // muss mindestens >= NN radius sein!!! rho kann bei zwei punkten = 1 sein; wenn diese schon vor drehung und projektion in einer ebene liegen !!!
    Nmin=3; //mindestanzahl Punkte in lokaler regression bzw im LocalPointsetForLoopPointID
    dH=0.02;
    rho0=0.4;
    epsilon0=0.15; //prescribed local average approximation error 
    count = 0;
    iterationscount = 0;
    
    TPS_AfterMLS = new vector<Point3D>(N);
    TPS_Ordered_Forward_Buffer = new vector<Point3D>(N);
    TPS_Ordered_Backward_Buffer = new vector<Point3D>(N);
    TPS_processed = new vector<Point3D>(N);
    
    PointToPointDistances = new Eigen::MatrixXd(N, N);
    BoolListPointsMovedInMLSIteration = new Eigen::MatrixXd(N,n0);
    AverageApproxmationError = new Eigen::MatrixXd(N,n0);
    
    LP_Object = new LocalPointset(this); 
    Spline_Object = new Spline(N);
    
    
    //Ordering:
    rho_order = 0.9;
    weight_order = 0.7;
    max_gap = H0/2;
    min_gap= 0.3; 
    ID_starting_point=0;
    ID_last_point=0;
    ID_ordered_forward=0;
    ID_ordered_backward=0;
    
    
    //Rechenmatrizen:
    Vector3d_1 = new Eigen::Vector3d;
    Vector3d_2 = new Eigen::Vector3d;
    
}

TriplelinePointsetClass::TriplelinePointsetClass(const Eigen::MatrixXd* RawTriplelinePointsetInput, int a){
    
    /*
    //N_TriplelinePoints = RawTriplelinePointsetInput->cols();
    maxPointID = N-1; //vector size da ID bei 0 beginnt
    n0=1;
    n=0;
    H0=0.5; 
    Nmin=3; //mindestanzahl Punkte in lokaler regression bzw im LocalPointsetForLoopPointID
    dH=0.001;
    rho0=0.8;
    epsilon0=0; //prescribed local average approximation error 
    N = RawTriplelinePointsetInput->cols(); //Anzahl Punkte: N+1
    
    TriplelinePointsetBeforeMovingLeastSquaresIteration = new vector<Point3D>(RawTriplelinePointsetInput->cols());
    TransformRawTriplelinePointsetInputFromEigenMatrixXdTypeIntoGlobalPoint3DVectorType();
    InitializePointIDsOfTriplelinePointsetVector();
    TriplelinePointsetAfterMovingLeastSquaresIteration = new vector<Point3D>(N);
    SmoothedTriplelinePointset = new vector<Point3D>(N);
    SmoothedReducedAndOrderedTriplelinePointset = new vector<Point3D>(N);
    QuadraticPointToPointDistancesMatrix = new Eigen::MatrixXd(N, N);
    LocalPointsetObject = new LocalPointset(this); //Größe LocalPointset grob schätzen 
    BoolListIfPointsWereMovedOrNotInMLSIteration = new Eigen::MatrixXd(n,N);
    WLScalculator = new WeightedLeastSquares;
     */
}

TriplelinePointsetClass::TriplelinePointsetClass(const vector<Point3D>* RawTriplelinePointsetInput) {

    /*
    //N_TriplelinePoints = RawTriplelinePointsetInput->size(),
    maxPointID = N-1; //vector size da ID bei 0 beginnt
    n0=1;
    n=0;
    H0=0.5; 
    Nmin=3; //mindestanzahl Punkte in lokaler regression bzw im LocalPointsetForLoopPointID
    dH=0.001;
    rho0=0.8;
    epsilon0=0; //prescribed local average approximation error 
    N = RawTriplelinePointsetInput->size();
    
    TriplelinePointsetBeforeMovingLeastSquaresIteration = new vector<Point3D>(RawTriplelinePointsetInput->size());
    *TriplelinePointsetBeforeMovingLeastSquaresIteration = *RawTriplelinePointsetInput;
    InitializePointIDsOfTriplelinePointsetVector();
    TriplelinePointsetAfterMovingLeastSquaresIteration = new vector<Point3D>(RawTriplelinePointsetInput->size());
    SmoothedTriplelinePointset = new vector<Point3D>(RawTriplelinePointsetInput->size());
    SmoothedReducedAndOrderedTriplelinePointset = new vector<Point3D>(RawTriplelinePointsetInput->size());
    QuadraticPointToPointDistancesMatrix = new Eigen::MatrixXd(N, N);
    LocalPointsetObject = new LocalPointset(this); //Größe LocalPointset grob schätzen
    BoolListIfPointsWereMovedOrNotInMLSIteration = new Eigen::MatrixXd(n,N);
    WLScalculator = new WeightedLeastSquares;
    
    
     */
}

TriplelinePointsetClass::~TriplelinePointsetClass() {

    delete TPS_BeforeMLS;
    delete TPS_AfterMLS;
    delete TPS_Ordered_Forward_Buffer;
    delete TPS_Ordered_Backward_Buffer;
    
    delete LP_Object;
    delete Spline_Object;
    
    delete PointToPointDistances;
   
    delete BoolListPointsMovedInMLSIteration;
    
    //delete pOutFile;
    delete Vector3d_1;
    delete Vector3d_2;
  
}

void TriplelinePointsetClass::use_PointCloudGenerator(int N_Points){
    
    N = N_Points;
    maxPointID = N-1; //vector size da ID bei 0 beginnt
    TPS_BeforeMLS = new vector<Point3D>(N);
    PointCloudGenerator PC1(TPS_BeforeMLS);
    PC1.generate_PointCloud(); //stored in TriplelinePointsetBeforeMLS
    output_TPS_IntoTxtFile(TPS_BeforeMLS, "generatedPC", maxPointID);
    
}

void TriplelinePointsetClass::input_TPS_OutOfTxtFile(string inFilename){
    
    
    double x, y, z;
    string X,Y,Z;
    string unused;
    int linecount = 0;
    
    ifstream inFile;   
    //inFile.open("bla");
    string directory = "D:\\masterarbeit\\NetBeansProjects\\Masterarbeit_FW\\input\\" + inFilename;
    inFile.open(directory.c_str());
    //inFile.open(inFilename.c_str()); //, ios::binary
    
    if(inFile.fail()){
       cout << "Error" << endl;
       exit(1);
    }
  
    
    /*
    // get length of file: http://www.cplusplus.com/reference/istream/istream/seekg/
    cout << "size file:" << endl;
    streampos p = inFile.tellg();
    cout << p <<endl;
    inFile.seekg (0, inFile.end);
    p = inFile.tellg();
    cout << p <<endl;
    int length = inFile.tellg();
    cout << length << endl;
    inFile.seekg (0, inFile.beg);
    p = inFile.tellg();
    cout << p <<endl << endl;
    */
    
    //streampos pA = inFile.tellg();
    //streampos pE;
    //cout << "pA: " << pA <<endl;
    
    /*
    while ( std::getline(inFile, unused) ){ //count lines of input
    linecount++;
    } 
    
    
    //LINECOUNT
      
    cout << "linecount: " << linecount << endl;
    N = linecount -1; //Wegen HeaderSpalte !!!!
    cout << "N points: " << N << endl;
    //TriplelinePointsetBeforeMovingLeastSquaresIteration = new vector<Point3D>(N);
    
    linecount=0; //input point data 
    cout << "linecount reset: " << linecount << endl;
    streampos reset = 0;
    inFile.seekg(reset, ios::beg);
    pE = inFile.tellg();
    cout << "pE: " << pE <<endl;
   
    TriplelinePointsetBeforeMovingLeastSquaresIteration = new vector<Point3D>(801);
    (*TriplelinePointsetBeforeMovingLeastSquaresIteration)[0].set_CoordinatesX(2);
    cout << (*TriplelinePointsetBeforeMovingLeastSquaresIteration)[0].get_CoordinatesX() << endl;
    */
   
   
    
    
    /*
    {
    string first_line; 
    getline(inFile, first_line); // waste the first line ("X Y Z")
    }
    */
    
    
    
    //pA = inFile.tellg();
    //cout << pA <<endl;
    
    Point3D Point;
    TPS_BeforeMLS = new vector<Point3D>;
    TPS_BeforeMLS->reserve(800);
    inFile >> X >> Y >> Z ;
    //cout << X << ";" << Y << ";"  << Z << endl;
    linecount +=1;
    int pointID = 0;
    while(!inFile.eof()){  
    inFile >> x >> y >> z ;
    //cout << "input file: " << x << "    " << y << " " << z << endl;
    Point.set_CoordinatesX(x);
    Point.set_CoordinatesY(y);
    Point.set_CoordinatesZ(z);
    Point.set_GlobalID(pointID);
    Point.set_epsilon(1); //so wird am anfang jeder Punkt gemoved 
    TPS_BeforeMLS->push_back(Point);    
    linecount +=1;
    pointID++;
    //pE = inFile.tellg();
    //cout << pE <<endl;
    } 
    cout << endl; 
    //cout << "linecount nach auslesen punkte mit header: " << linecount << endl;
    //cout << "pointID nach auslesen punkte mit header: " << pointID << endl;
    //cout << "Vectorsize nach auslesen punkte mit header: " << TriplelinePointsetBeforeMLS->size() << endl;
    N = pointID;
    maxPointID = N-1; //vector size da ID bei 0 beginnt
    //cout << "N: " << N << endl; 
    
   
    /*
     while(!inFile.eof()){
       
       inFile >> a >> k >> b >> k >> c ;
       cout << a << ";" << b << ";"  << c << endl;
   } 
   */
   
   
   inFile.close();
    
   output_TPS_IntoTxtFile(TPS_BeforeMLS, "OutputTest", maxPointID);
}

void TriplelinePointsetClass::output_DataDebugging_txt(string outFilename){
    
    pOutFile = new ofstream;
    string directory = outFilename;
    pOutFile->open (directory.c_str());
}

void TriplelinePointsetClass::output_TPS_IntoTxtFile(vector<Point3D>* Pointset, string outFilename, int maxID){
    
    double x, y, z;
    ofstream outFile;
    string directory = outFilename;
    outFile.open (directory.c_str());
    outFile << "X" << "    " << "Y" << "   "  << "Z" << "   " << "GLobalID" << "    " << "OrderedID" << "   " << "epsilon" << endl;  //Header
      
    for(int i=0; i<=maxID; i++){
         if(i == maxID){
         outFile << (*Pointset)[i].get_CoordinatesX() << "  " << (*Pointset)[i].get_CoordinatesY() << " "  << (*Pointset)[i].get_CoordinatesZ()<< " " << (*Pointset)[i].get_GlobalID() << " " << (*Pointset)[i].get_ID_ordered() << "   " << (*Pointset)[i].get_epsilon();   
         }else{
         outFile << (*Pointset)[i].get_CoordinatesX() << "  " << (*Pointset)[i].get_CoordinatesY() << " "  << (*Pointset)[i].get_CoordinatesZ() << " " << (*Pointset)[i].get_GlobalID() << " " << (*Pointset)[i].get_ID_ordered() << "  " << (*Pointset)[i].get_epsilon() << endl;
         }
     }
     outFile.close();  
}

void TriplelinePointsetClass::output_EigenMatrixXd(Eigen::MatrixXd* EigenMatrix, string outFilename){
    
    ofstream outFile;
    string directory = outFilename;
    outFile.open (directory.c_str());
    outFile << (*EigenMatrix);
    outFile.close();  
    
}

void TriplelinePointsetClass::set_Parameter(){
   
   // set....... Textdatei einlesen der Aufrufparameter
    
    
    
    
}

void TriplelinePointsetClass::process_TriplelinePointset(){ // input matrix mit spalntezahl = punktzahl also 3xN matrix
         
    //set_Parameter();
    smoothe_TPS();
    
    orderAndReduce_TPS();
    
    calc_BSpline_ProcessedTPS();
  
}    

void TriplelinePointsetClass::smoothe_TPS(){
    
    for(int i=0; i<n0; i++){ //n ist die vorher festgelegte Anzahl der Moving Least Squares Iteration Schritte über das komplette Pointset
        
        n++;
            //FileOutput
            stringstream n_MLSIterationStep; 
            n_MLSIterationStep << n;
            string seperator = "_";
            string outFilename = n_MLSIterationStep.str() + seperator + "n" +  seperator + "TriplelinePointsetBeforeMLS";
        output_TPS_IntoTxtFile(TPS_BeforeMLS, outFilename, maxPointID);
        
        calc_PointToPointDistances(TPS_BeforeMLS);
            //output_EigenMatrixXd(PointToPointDistances, "PointToPointDistances");
            //output_EigenMatrixXd(QuadraticPointToPointDistances, "QuadraticPointToPointDistances"); 
        //IF überpfüfen ob überhaupt noch punkte verschoben werden müssen !!!!! mit errorwert vector
        calc_MLS_Iteration();
            outFilename = n_MLSIterationStep.str() + seperator + "nE" +  seperator + "TriplelinePointsetAfterMLS";
            cout << "check " << endl;
        output_TPS_IntoTxtFile(TPS_AfterMLS, "TriplelinePointsetAfterMLS", maxPointID);
        
        if(i<n0-1){
            switch_Buffer();
        }
    }
    output_EigenMatrixXd(BoolListPointsMovedInMLSIteration, "BoolListPointsMovedInMLSIteration"); 
    output_EigenMatrixXd(AverageApproxmationError, "AverageApproxmationError");
}

void TriplelinePointsetClass::calc_PointToPointDistances(vector<Point3D>* TPS){
    
    int u=0;
    int p=0;
    double Distance=0;
    double squaredDistance=0;
    
    for(int i=0; i<=maxPointID; i++){
        p=i;
        for(u=i+1; u<=maxPointID; u++){
        (*Vector3d_1) = (*TPS)[p].get_CoordinatesXYZ() - (*TPS)[u].get_CoordinatesXYZ();
        Distance = (*Vector3d_1).norm();
        (*PointToPointDistances)(p,u) = Distance;
        (*PointToPointDistances)(u,p) = Distance;
        }
    }  
}

void TriplelinePointsetClass::calc_MLS_Iteration(){
    
    for(int i=0; i<=maxPointID; i++){
        
        if((*TPS_BeforeMLS)[i].get_epsilon() >= epsilon0){     
            calc_Sufficient_LP_forMLS(i);
            (*TPS_AfterMLS)[i]= (*LP_Object).calc_MovedPoint_AfterMLS();
            //Information dass Punkt gemoved wurde speichern!!!! 
            (*BoolListPointsMovedInMLSIteration)(i,n-1) = 1; 
            (*AverageApproxmationError)(i,n-1) = (*TPS_AfterMLS)[i].get_epsilon();
        }else{
            //Information dass Punkt NICHT gemoved wurde speichern!!!! 
            (*TPS_AfterMLS)[i]= (*TPS_BeforeMLS)[i];
            (*BoolListPointsMovedInMLSIteration)(i,n-1) = 0;
            (*AverageApproxmationError)(i,n-1) = 0;
        }  
    }      
}

void TriplelinePointsetClass::calc_Sufficient_LP_forMLS(int GlobalID_LoopPoint){
    
    //cout << "GlobalID_LoopPoint " << GlobalID_LoopPoint << endl;
    
    //FileOutput iteration history:
    stringstream n_MLSIterationStep; 
    n_MLSIterationStep << n;
    stringstream GlobalID_LP; 
    GlobalID_LP << GlobalID_LoopPoint;
    string seperator = "_";
    string outFilename = n_MLSIterationStep.str() + seperator + "n" +  seperator + GlobalID_LP.str() + seperator + "ID" + seperator + "calc_SufficientLocalPointset";
    ofstream outFile;
    string directory =  outFilename;
    outFile.open (directory.c_str());
  
    (*LP_Object).clear();
    int N_LocalPoints =0;
    double rho =0;
    double H=H0;
    outFile << "GlobalID_LoopPoint: " << GlobalID_LoopPoint << endl;
    count = 0;

    /*
    //Initialize Local Pointset with Nmin
    outFile << "Initialize Local Pointset with Nmin" << endl;
    outFile << "Nmin: " << Nmin << endl << endl;
    while(N_LocalPoints < Nmin){
        count++;
        outFile << "count: " << count <<endl;
        (*(*LP_Object).get_LocalPoints()).resize(0);
        (*LP_Object).set_OriginPoint(GlobalID_LoopPoint, TPS_BeforeMLS);
        outFile << "Origin: " << endl << (*(*LP_Object).get_LocalPoints())[0].get_CoordinatesXYZ() << endl;
        outFile << "SIZE: " << (*(*LP_Object).get_LocalPoints()).size() << endl;    
        calc_NextNeighborPS_RadiusH(H, TPS_BeforeMLS);
        N_LocalPoints = (*LP_Object).get_N_LocalPoints();
        outFile << "N: " << N_LocalPoints << endl;
        outFile << "H: " << H << endl;
            if(N_LocalPoints < Nmin){
                H +=  dH;
            } 
    }
  
    rho = abs((*LP_Object).calc_Correlation_LP(H));
    outFile << endl << "END of Nmin Loop: " << endl;
    outFile << "rho: " << rho << endl << endl << endl << endl << endl << endl;
    */
      
    //Nmin muss nicht überprüft werden, da rho nicht berechnet wird wenn Nmin < 3.
    //Grund: keine quadratische curve aus weniger als drei punkten berechenbar.Nicht genug Gleichungssysteme. 
    //Folge: rho wird wie in default =0 zurück gegeben und Punktset automatisch vergrößert.
    //ABER: (rho==1) kann bei zwei punkten vorkommen!!!! siehe constructor erklärung
    outFile << "Expand Local Pointset with rho: " << endl << endl;
    iterationscount = 0;
    //Expand Local Pointset with rho
    while( (rho < rho0) || (rho>=1) ){  
        iterationscount++;
        (*(*LP_Object).get_LocalPoints()).resize(0);
        (*LP_Object).set_OriginPoint(GlobalID_LoopPoint, TPS_BeforeMLS);
        calc_NextNeighborSet_RadiusH(H, TPS_BeforeMLS);
        if( ((*LP_Object).get_N_LocalPoints() > N_LocalPoints)){ 
            rho = abs((*LP_Object).calc_Correlation_LP(H));
            N_LocalPoints = (*LP_Object).get_N_LocalPoints();
        }  
        outFile << "count: " << count << endl;
        outFile << "interationscount: " << iterationscount << endl;
        outFile << "N: " << (*LP_Object).get_N_LocalPoints() << endl;
        outFile << "H: " << H << endl;
        outFile << "rho: " << rho << endl;
        if((rho < rho0) || (rho>=1)){
            H += dH; //region expansion
        }
        /*
        if(N_LocalPoints == N){// falls kein rho > rho0 gefunden werden kann
            
            H0 *= 0.9;
            H=H0;
            if(H0 < dH){
                cout << "ERROR KEIN LOKALES PS FÜR ID: " << GlobalID_LoopPoint << endl;
            } 
        }
         */
        if(N_LocalPoints==N){
            cout << "ERROR LP FOR WLS" << endl;
        }
     }
    (*(LP_Object->get_LocalPoints()))[0].set_H_last_iteration(H); // 0 ist origin point im local Pset
    outFile.close();
}

void TriplelinePointsetClass::calc_NextNeighborSet_RadiusH(double H, vector<Point3D>* Pointset){
  
    int GlobalID_LoopPoint = LP_Object->get_GlobalID_OriginPoint();
    (*LP_Object).set_LP_radius_H(H);
    for(int i=0; i<=maxPointID; i++){
        if(((*PointToPointDistances)(GlobalID_LoopPoint, i) < H) && (i != GlobalID_LoopPoint) ){
        (*LP_Object).set_LocalPoint(i, Pointset);
        }
    }    
}

void TriplelinePointsetClass::output_LP_Object_IntoTxtFile(double H, double rho){ //LP wird eigentlich in der Klasse LocalPointset ausgegeben; 
   
    //FileOutput
    stringstream n_MLSIterationStep; 
    n_MLSIterationStep << n;
    stringstream GlobalID_LP;
    GlobalID_LP << (*LP_Object).get_GlobalID_OriginPoint();
    stringstream Radius;
    Radius << H;
    stringstream Rho;
    Rho << rho;
    string seperator = "_";
    string outFilename = n_MLSIterationStep.str() + seperator + "n" + seperator + GlobalID_LP.str() + seperator + "ID" + seperator + Radius.str() + seperator + "H" + seperator + Rho.str() + seperator + "Rho" + seperator + "LocalPS";
    (*LP_Object).output_LP_TxtFile(outFilename);

}

void TriplelinePointsetClass::switch_Buffer(){ //Pointer switchen
    
    vector<Point3D>* temp;
    temp = TPS_BeforeMLS;
    TPS_BeforeMLS = TPS_AfterMLS;
    TPS_AfterMLS = temp;
    
}

//
//
//
//ORDER

void TriplelinePointsetClass::orderAndReduce_TPS(){
    
    n=n0+1; // nur für output nummerierung
    calc_PointToPointDistances(TPS_AfterMLS);
   
    (*LP_Object).clear();
    output_DataDebugging_txt("TPS_ordered_debugg");
    
    (*pOutFile) << "ordering_dynamicDistances:" << endl;
    ordering_dynamicDistances();
    (*pOutFile) << "output_ordered_TPS_IntoTxtFile" << endl;
    
    (*pOutFile) << "ID_ordered_forward: " <<  ID_ordered_forward << endl;
    (*pOutFile) << "ID_ordered_backward: " << ID_ordered_backward << endl;
    
    int maxID_ordered_forward = TPS_Ordered_Forward_Buffer->size()-1;
    output_TPS_IntoTxtFile(TPS_Ordered_Forward_Buffer, "TPS_Ordered_Forward_Buffer", maxID_ordered_forward);
    int maxID_ordered_backward = TPS_Ordered_Backward_Buffer->size()-1;
    output_TPS_IntoTxtFile(TPS_Ordered_Backward_Buffer, "TPS_Ordered_Backward_Buffer", maxID_ordered_backward);
    copy_Buffer_ReducedAndOrdered();
    int maxID_ordered = TPS_processed->size()-1;
    output_TPS_IntoTxtFile(TPS_processed, "TPS_processed", maxID_ordered);
    output_TPS_IntoTxtFile(TPS_AfterMLS, "TPS_complete_AfterOrdering", maxPointID);
    
    pOutFile->close();
    delete pOutFile;
    
}

void TriplelinePointsetClass::ordering_dynamicDistances(){
    
    TPS_Ordered_Forward_Buffer->resize(0);
    TPS_Ordered_Backward_Buffer->resize(0);
    ID_ordered_forward = 0;
    ID_ordered_backward = 0;
    
    (*pOutFile) << "select starting point; = first ordered point ordered ID =1" << endl;
    ID_starting_point = calc_RandomID(N);
    (*pOutFile) << "RandomID: " << ID_starting_point << endl;
    (*TPS_AfterMLS)[ID_starting_point].set_ID_ordered(ID_ordered_forward);
    (*TPS_AfterMLS)[ID_starting_point].set_bool_ordered(true);
    TPS_Ordered_Forward_Buffer->push_back((*TPS_AfterMLS)[ID_starting_point]);
    
    (*pOutFile) << "order_Neighborhood_method1" << endl;
    order_Neighborhood_method1(ID_starting_point);
}

int TriplelinePointsetClass::calc_RandomID(int N){
    
    return (rand() % N);
}

void TriplelinePointsetClass::order_Neighborhood_method1(int ID_start){
    
    int ID_first_forwardPoint = 0;
    int ID_first_backwardPoint = 0;
    
    (*pOutFile) << "calc_Sufficient_LP_forOrdering(ID_starting_point)" << endl;
    calc_Sufficient_LP_forOrdering(ID_starting_point);
    (*pOutFile) << "ID_starting_point: " << ID_starting_point << endl;
  
    ID_first_forwardPoint = (*LP_Object).get_ID_first_forwardPoint(weight_order, max_gap, min_gap);
    (*pOutFile) << "ID_first_forwardPoint: " << ID_first_forwardPoint << endl;
    ID_ordered_forward++;
    (*TPS_AfterMLS)[ID_first_forwardPoint].set_ID_ordered(ID_ordered_forward);
    (*TPS_AfterMLS)[ID_first_forwardPoint].set_bool_ordered(true);
    TPS_Ordered_Forward_Buffer->push_back((*TPS_AfterMLS)[ID_first_forwardPoint]);
    
    ID_first_backwardPoint = (*LP_Object).get_next_orderedPoint(weight_order, max_gap, min_gap, ID_first_forwardPoint);
    (*pOutFile) << "ID_first_backwardPoint: " << ID_first_backwardPoint << endl;
    ID_ordered_backward--;
    (*TPS_AfterMLS)[ID_first_backwardPoint].set_ID_ordered(ID_ordered_backward);
    (*TPS_AfterMLS)[ID_first_backwardPoint].set_bool_ordered(true);
    TPS_Ordered_Backward_Buffer->push_back((*TPS_AfterMLS)[ID_first_backwardPoint]);
    
    (*pOutFile) << "order_direction_forward" << endl;
    order_direction_forward(ID_first_forwardPoint);
    
    (*pOutFile) << "order_direction_backward" << endl;
    order_direction_backward(ID_first_backwardPoint);
}

void TriplelinePointsetClass::order_direction_forward(int ID_first_forwardPoint){
    
    double ID_new_orderedPoint=0;
    int ID_last_orderedPoint = ID_first_forwardPoint;
    int ID_2last_orderedPoint = ID_starting_point;
    
    (*pOutFile) << "init order_direction_forward:" << endl;
    (*pOutFile) << "ID_new_orderedPoint: " << ID_new_orderedPoint << endl;
    (*pOutFile) << "ID_ordered_forward: " << ID_ordered_forward << endl;
    (*pOutFile) << "ID_last_orderedPoint: " << ID_last_orderedPoint << endl;
    (*pOutFile) << "ID_2last_orderedPoint: " << ID_2last_orderedPoint << endl;
    
    (*pOutFile) << "while loop order_direction_forward:" << endl << endl;
    while(ID_new_orderedPoint >= 0){
        
        (*pOutFile) << "calc_Sufficient_LP_forOrdering with ID_last_orderedPoint: " << ID_last_orderedPoint << endl;
        calc_Sufficient_LP_forOrdering(ID_last_orderedPoint);
        
        (*pOutFile) << "get_next_orderedPoint" << endl;
        ID_new_orderedPoint = (*LP_Object).get_next_orderedPoint(weight_order, max_gap, min_gap, ID_2last_orderedPoint);
        
        if(ID_new_orderedPoint<0){
                
            (*pOutFile) << "IF ID_new_orderedPoint<0" << endl;
            (*pOutFile) << "check_existing_neighbor(ID_2last_orderedPoint): " << ID_2last_orderedPoint << endl << endl;
            ID_new_orderedPoint = check_existing_neighbor(ID_2last_orderedPoint);
            (*pOutFile) << "new ordered point CHECK NN: " << ID_new_orderedPoint << endl;
        }
        
        if(ID_new_orderedPoint >= 0){
            
            (*pOutFile) << "IF ID_new_orderedPoint >= 0" << endl;
            (*TPS_AfterMLS)[ID_new_orderedPoint].set_bool_ordered(true);
            ID_ordered_forward++;
            (*TPS_AfterMLS)[ID_new_orderedPoint].set_ID_ordered(ID_ordered_forward);
            TPS_Ordered_Forward_Buffer->push_back((*TPS_AfterMLS)[ID_new_orderedPoint]);
            ID_2last_orderedPoint = ID_last_orderedPoint;
            (*pOutFile) << "new ID_2last_orderedPoint: " << ID_2last_orderedPoint << endl;
            ID_last_orderedPoint = ID_new_orderedPoint;
            (*pOutFile) << "new last_orderedPoint: " << ID_last_orderedPoint << endl << endl;
            (*pOutFile) << "new ordered point: " << ID_new_orderedPoint << endl;
        }
        (*pOutFile) << endl << endl;
    } 
    (*pOutFile) << "endLoop" << endl;
    (*pOutFile) << "max_forward_ID: " << ID_ordered_forward << endl;
}

int TriplelinePointsetClass::check_existing_neighbor(int ID_2last_orderedPoint){ 
    
    double dist = 0;
    double dist2=0;
    double d_min = max_gap;
    int ID_neighbor = -1;
    
    for(int i=0; i<= maxPointID; i++){
        
        (*pOutFile) << "i: " << i << endl;
        if(((*TPS_AfterMLS)[i].get_ID_ordered() == 0)){
            
            dist = (*PointToPointDistances)(i, LP_Object->get_GlobalID_OriginPoint()); 
            dist2 = (*PointToPointDistances)(i, ID_2last_orderedPoint); 
            (*pOutFile) << "dist: " << dist << endl;
            if( (dist < max_gap) && (dist < d_min) && (dist2>dist)){ 
                
                (*pOutFile) << "dist < max_gap && dist < d_min" << endl;
                if( (check_direction_dotProduct(i, ID_2last_orderedPoint)==true) ){
               
                (*pOutFile) << "(*TPS_AfterMLS)[i].get_ID_ordered() == 0 && check_direction_dotProduct(i, ID_2last_orderedPoint)==true " << endl;    
                ID_neighbor = i;
                (*pOutFile) << "ID_neighbor: " << ID_neighbor << endl;
                d_min = dist;
                (*pOutFile) << "d_min: " << d_min << endl;
                }
            }
        }
        (*pOutFile) << endl << endl << endl;
    }   
    
    (*pOutFile) << "final ID_neighbor: " << ID_neighbor << endl;
    return ID_neighbor;
}

bool TriplelinePointsetClass::check_direction_dotProduct(int ID_newP, int ID_2last_orderedPoint){ //spitzer winkel skalarprodukt > 0; stumpfer winkel < 0
    
    bool check = false;
    double dot = 0;
    
    (*Vector3d_1) = (*TPS_AfterMLS)[ID_newP].get_CoordinatesXYZ() - (*TPS_AfterMLS)[ID_2last_orderedPoint].get_CoordinatesXYZ(); 
    (*Vector3d_2) = (*TPS_AfterMLS)[ID_newP].get_CoordinatesXYZ() - (*LP_Object->get_LocalPoints())[0].get_CoordinatesXYZ();
    
    (*pOutFile) << "(*TPS_AfterMLS)[ID_newP].get_CoordinatesXYZ(): " << (*TPS_AfterMLS)[40].get_CoordinatesXYZ() << endl;
    
    (*pOutFile) << "(*LP_Object->get_LocalPoints())[0].get_CoordinatesXYZ(): " << (*LP_Object->get_LocalPoints())[0].get_CoordinatesXYZ() << endl;
    
    (*pOutFile) << "Vector3d_1: " << (*Vector3d_1) << endl;
    (*pOutFile) << "Vector3d_2: " << (*Vector3d_2) << endl;
           
    dot = (*Vector3d_1).dot((*Vector3d_2));
    (*pOutFile) << "dot: " << dot << endl;
    
    if(dot >= 0){
        
        check = true;   
    }
    
    (*pOutFile) << "return check: " << check << endl;
    return check;
}

void TriplelinePointsetClass::order_direction_backward(int ID_first_backwardPoint){
    
    double ID_new_orderedPoint=0;
    int ID_last_orderedPoint = ID_first_backwardPoint;
    int ID_2last_orderedPoint = ID_starting_point;
    
    (*pOutFile) << "init order_direction_backward:" << endl;
    (*pOutFile) << "ID_new_orderedPoint: " << ID_new_orderedPoint << endl;
    (*pOutFile) << "ID_ordered_backward: " << ID_ordered_backward << endl;
    (*pOutFile) << "ID_last_orderedPoint: " << ID_last_orderedPoint << endl;
    (*pOutFile) << "ID_2last_orderedPoint: " << ID_2last_orderedPoint << endl;
    
    (*pOutFile) << "while loop order_direction_backward:" << endl << endl;
    while(ID_new_orderedPoint >= 0){
        
        (*pOutFile) << "calc_Sufficient_LP_forOrdering with ID_last_orderedPoint: " << ID_last_orderedPoint << endl;
        calc_Sufficient_LP_forOrdering(ID_last_orderedPoint);
        
        (*pOutFile) << "get_next_orderedPoint" << endl;
        ID_new_orderedPoint = (*LP_Object).get_next_orderedPoint(weight_order, max_gap, min_gap, ID_2last_orderedPoint);
        
        if(ID_new_orderedPoint<0){
                
            (*pOutFile) << "IF ID_new_orderedPoint<0" << endl;
            (*pOutFile) << "check_existing_neighbor(ID_2last_orderedPoint): " << ID_2last_orderedPoint << endl << endl;
            ID_new_orderedPoint = check_existing_neighbor(ID_2last_orderedPoint);
            (*pOutFile) << "new ordered point CHECK NN: " << ID_new_orderedPoint << endl;
        }
        
        if(ID_new_orderedPoint >= 0){
            
            (*pOutFile) << "IF ID_new_orderedPoint >= 0" << endl;
            ID_ordered_backward--;
            (*TPS_AfterMLS)[ID_new_orderedPoint].set_ID_ordered(ID_ordered_backward);
            TPS_Ordered_Backward_Buffer->push_back((*TPS_AfterMLS)[ID_new_orderedPoint]);
            ID_2last_orderedPoint = ID_last_orderedPoint;
            (*pOutFile) << "new ID_2last_orderedPoint: " << ID_2last_orderedPoint << endl;
            ID_last_orderedPoint = ID_new_orderedPoint;
            (*pOutFile) << "new last_orderedPoint: " << ID_last_orderedPoint << endl << endl;
            (*pOutFile) << "new ordered point: " << ID_new_orderedPoint << endl;
        }
        
        (*pOutFile) << endl << endl;
    } 
    (*pOutFile) << "endLoop" << endl;

    (*pOutFile) << "max_backward_ID: " << ID_ordered_backward << endl;
 
}

void TriplelinePointsetClass::calc_Sufficient_LP_forOrdering(int GlobalID_Point){
    
    int ID = calc_Next_Neighbor_ID(GlobalID_Point);
    //(*pOutFile) << "test dist: " << (*PointToPointDistances)(GlobalID_Point, ID)  << endl;
    int N_LocalPoints=0;
    double rho = 1;
    double rho_end=0;
    double H_start = calc_Next_Neighbor_Distance(GlobalID_Point)+ dH/100; // damit NN Punkt innerhalb < H liegt
    double H = H_start;
    double H_end = H_start;
    
    //(*pOutFile) << "H_start: " << H_start << endl;
    
    //(*pOutFile) << "begin (rho>=1 ||(rho<=0) loop" << endl;
    while((rho>=1 || (rho<=0)) || (N<=Nmin)){
        //(*pOutFile) << "begin" << endl;
        (*(*LP_Object).get_LocalPoints()).resize(0);
        (*LP_Object).set_OriginPoint(GlobalID_Point, TPS_AfterMLS);
        //(*pOutFile) << "calc_NextNeighborPS_RadiusH" << endl;
        calc_NextNeighborSet_RadiusH(H, TPS_AfterMLS);
        //(*pOutFile) << "N_LocalPoints: " << (*LP_Object).get_N_LocalPoints() << endl;
        if((*LP_Object).get_N_LocalPoints() > N_LocalPoints){ 
            rho = abs((*LP_Object).calc_Correlation_LP(H));
            (*pOutFile) << "rho: " << rho << endl;
            N_LocalPoints = (*LP_Object).get_N_LocalPoints();
            //(*pOutFile) << "N_LocalPoints: " << N_LocalPoints << endl;
        }   
        if((rho>=1) || (rho<=0)){
            H += dH;    
        }
        //(*pOutFile) << "H: " << H << endl;
        if(N_LocalPoints==N){
            cout << "ERROR LP FOR ORDERING" << endl;
        }
    }
    //(*pOutFile) << "H_end: " << H_end << endl;
    //(*pOutFile) << "end (rho>=1 ||(rho<=0) loop" << endl; 
    
    //(*pOutFile) << "begin (rho >= rho_order) loop" << endl; 
    while((rho >= rho_order)){  //MindestPunktzahl muss erfüllt sein  
        //(*pOutFile) << "begin" << endl;
        (*(*LP_Object).get_LocalPoints()).resize(0);
        (*LP_Object).set_OriginPoint(GlobalID_Point, TPS_AfterMLS);
        //(*pOutFile) << "calc_NextNeighborPS_RadiusH" << endl;
        calc_NextNeighborSet_RadiusH(H, TPS_AfterMLS);
        //(*pOutFile) << "N_LocalPoints: " << (*LP_Object).get_N_LocalPoints() << endl;
        if((*LP_Object).get_N_LocalPoints() > N_LocalPoints){ 
            rho = abs((*LP_Object).calc_Correlation_LP(H));
            (*pOutFile) << "rho: " << rho << endl;
            N_LocalPoints = (*LP_Object).get_N_LocalPoints();
            //(*pOutFile) << "N_LocalPoints: " << N_LocalPoints << endl;
        } 
        if(H > (*TPS_AfterMLS)[GlobalID_Point].get_H_last_iteration()){
            H_end = H;
            rho_end = rho;
            break;
        }
        if((rho >= rho_order)){
            H_end = H;
            rho_end = rho;
            H += dH;
        }
        //(*pOutFile) << "H: " << H << endl;
        if(N_LocalPoints==N){
            cout << "ERROR LP FOR ORDERING" << endl;
        }
    }
    //(*pOutFile) << "end (rho >= rho_order) loop" << endl; 
    (*pOutFile) << "rho_end: " << rho_end << endl;
    (*pOutFile) << "H_end: " << H_end << endl;
    
    (*(*LP_Object).get_LocalPoints()).resize(0);
    (*LP_Object).set_OriginPoint(GlobalID_Point, TPS_AfterMLS);
    calc_NextNeighborSet_RadiusH(H_end, TPS_AfterMLS);
    (*pOutFile) << "N_local: " << (*LP_Object).get_N_LocalPoints() << endl;
    output_LP_Object_IntoTxtFile(H_end, rho_end);
} 

int TriplelinePointsetClass::calc_Next_Neighbor_ID(int ID_Point){
    
    double dist=0;
    double dist_min=0;
    int ID_NN =0;
    
    for(int i=0; i<= maxPointID; i++){
        if(i!=ID_Point){
            dist = (*PointToPointDistances)(ID_Point, i);
            if(dist_min==0){
                ID_NN=i;
                dist_min = dist;  
            }
            if(dist < dist_min){
               ID_NN=i;
               dist_min=dist;
            }  
        }  
    }
    //(*pOutFile) << "ID_NN: " << ID_NN << endl;
    return ID_NN; 
}

double TriplelinePointsetClass::calc_Next_Neighbor_Distance(int ID_Point){
    
    double dist=0;
    double dist_min=0;
  
    for(int i=0; i<= maxPointID; i++){
        if(i!=ID_Point){
            dist = (*PointToPointDistances)(ID_Point, i);
            if(dist_min==0){
            dist_min = dist;
            }
            if(dist < dist_min){
               dist_min=dist;
            }  
        }  
    }
    //(*pOutFile) << "dist_min_NN: " << dist_min << endl;
    return dist_min;
}

void TriplelinePointsetClass::copy_Buffer_ReducedAndOrdered(){
    
    TPS_processed->resize(0);
    int ID_max_forward = TPS_Ordered_Forward_Buffer->size()-1;
    int ID_max_backward = TPS_Ordered_Backward_Buffer->size()-1;
    
    for(int i=ID_max_backward; i>=0; i--){
        
       TPS_processed->push_back((*TPS_Ordered_Backward_Buffer)[i]); 
    }
    
    for(int i=0; i<=ID_max_forward; i++){
        
        TPS_processed->push_back((*TPS_Ordered_Forward_Buffer)[i]);   
    } 
}
 
//
//
//
//SPLINE:

void TriplelinePointsetClass::calc_BSpline_ProcessedTPS(){
    
    Spline_Object->input_Pointset(TPS_processed);
    Spline_Object->interpolate_BSpline_3D_Degree_2_Default();
    lenght_spline = Spline_Object->calc_Line_Lenght_Spline();
    cout << "lenght_spline: " << lenght_spline << endl;
    cout << "lenght_input: " << Spline_Object->calc_Line_Lenght_ordered_PointInput() << endl;
}

