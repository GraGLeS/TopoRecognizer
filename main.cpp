
/* 
 * File:   main.cpp
 * Author: Fabrice Wagner
 * Created on 18. Dezember 2016
 */

#include "main.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include "TriplelinePointsetClass.h"
#include "Point3D.h"

using namespace std;


int main() { //int argc, char** argv
    
   //testFileOutput(); 
   //testFileInput();
    
    //Point3D testpoint;
    //testpoint.~Point3D();
    
    
    TriplelinePointsetClass Test;
    Test.process_TriplelinePointset();
    //txtInputTest.~TriplelinePointsetClass();
    //TriplelinePointsetClass.setParametersForTriplelinePointsetProcessing(ParameterList);
    
    
    
    
    
    return 0;
}


void testFileOutput(){
    
    double a,b,c;
    a=1.01;
    b=2.01;
    c=3.01;
    
     ofstream myfile;
     myfile.open ("example.txt");
     myfile << a << " " << b << " "  << c << endl; 
     myfile << a << " " << b << " "  << c << endl; 
     myfile << a << " " << b << " "  << c << endl; 
     myfile << a << " " << b << " "  << c << endl; 
     myfile << a << " " << b << " "  << c << endl; 
     myfile << a << " " << b << " "  << c << endl; 
     myfile << a << " " << b << " "  << c << endl; 
     
     myfile << a << " " << b << " "  << c;
     
     /*
     myfile << a << ";" << b << ";"  << c << endl; 
     myfile << a << ";" << b << ";"  << c << endl; 
     myfile << a << ";" << b << ";"  << c << endl; 
     myfile << a << ";" << b << ";"  << c << endl; 
     myfile << a << ";" << b << ";"  << c << endl; 
     myfile << a << ";" << b << ";"  << c << endl; 
     myfile << a << ";" << b << ";"  << c << endl; 
     myfile << a << ";" << b << ";"  << c << endl; 
     myfile << a << ";" << b << ";"  << c << endl; 
     myfile << a << ";" << b << ";"  << c << endl;
     */     
     myfile.close();
    
    
}
void testFileInput(){
    
   ifstream inFile;
   inFile.open ("testFct.txt");
   double a,b,c;
   string x,y,z;
   int linecount = 0;
   
   if(inFile.fail()){
       cout << "Error" << endl;
       exit(1);
   }
    
   
   streampos pA = inFile.tellg();
   cout << pA <<endl;
   
   /*
   {
   string first_line; 
   getline(inFile, first_line); // waste the first line ("X Y Z")
   }
   */
     
   inFile >> x >> y >> z ;
   cout << x << ";" << y << ";"  << z << endl;
   linecount +=1;
   
   streampos pE =0;
   
   while(!inFile.eof()){
       
       inFile >> a >> b >> c ;
       cout << a << ";" << b << ";"  << c << endl;
       linecount +=1;
       pE = inFile.tellg();
       cout << pE <<endl;
   } 
   cout << linecount << endl;
   
   
   /*
     while(!inFile.eof()){
       
       inFile >> a >> k >> b >> k >> c ;
       cout << a << ";" << b << ";"  << c << endl;
   } 
   */
   
   
   inFile.close();
}

/*
 
  
 
 NAMEN zusammenfügen
 cout << endl;
    string a = "naise";
    cout << "a:" << a << endl;
    string b;
    int x = 2;
    cout << "x: " << x << endl; 
    stringstream c;
    c << x;
    b = c.str();
    cout << "b: " <<  b << endl;
    string d ="_";
    
    string name = b + d + a;
    cout << "name: " << name << endl;
 
 
 
 
 
 
 
 */