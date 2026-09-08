#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <fstream>


// Implemented in quadParams.cc


// Base class for various sensors/motors 
class poseTemplate
{
    private:
        // == Member Variables = 
        Eigen::Vector3d location_; // [m] Location of the module with respect to the quad body frame, origin
        Eigen::Matrix3d R_; // Rotation matrix from module coordinate frame to quad body coordinate frame
    public:
        // == Constructors ==
        poseTemplate(){} // Null constructor to reserve memory
        
        /* 
        Argument Constructor
        @param location location of the origin of the module with respect to the quad body frame and origin
        @param R Orientation of the module in point rotation convention with respect to the body frame
        */
        poseTemplate(const Eigen::Vector3d & location, const::Eigen::Matrix3d & R): location_(location), R_(R){}
        

        // == Getters ==
        const Eigen::Vector3d& location() const {return location_;}
        const Eigen::Matrix3d& R() const {return R_;}

        // == Setters ==
        void location(const Eigen::Vector3d& location){location_=location;}
        void R(const Eigen::Matrix3d & R){R_=R;}

};


// Class to hold information about propellor
class propParams : public poseTemplate
{
    private:

        // == Parameters ==
        double kF_, kN_; //  prop lift force coeff, prop torque coeff
        double cm_, tauM_; // motor constant response, and  motor time constant
        int omegaRDir_; // Direction - +1 for CCW, -1 for CW
        std::string name_;

        // == Statics ==
        inline static const std::array<std::string, 8> varNames = {"location", "R", "kF", "kN", "omegaRDir", "cm", "tauM", "name"}; // List of variable names to recognize
        
    public:
    
    // == Constructors ==

    /*
    Empty constructor
    */
    propParams(){} 
    
    /* 
        Arg constructor
        @param kF Prop force coefficeint where F = kF * omega^2
        @param kN Prop torque coefficeint where N = kN * omega^2
        @param cm Prop motor constant where omega = cm * voltage
        @param tauM Prop time constant 
        @param omegaRDir Prop rotation direction where 1 is CCW and -1 is CW 
        @param name Name of the prop

    */
    propParams(
        const double kF, 
        const double kN, 
        const double cm, 
        const double tauM, 
        const double omegaRDir, 
        const std::string name) 
        : kF_(kF), kN_(kN), cm_(cm), tauM_(tauM), omegaRDir_(omegaRDir), name_(name)
        {}
    
    propParams(const std::string & line); //String line constructor - Defined in quadParams.cc

    // == Getters ==
    const double kF() const {return kF_;}
    const double kN() const {return kN_;}
    const int omegaRDir() const {return omegaRDir_;}
    const double cm() const {return cm_;}
    const double tauM() const {return tauM_;}
    const std::string & name() const {return name_;}

    // == Setters ==
    void kF(const double kF){kF_=kF;}
    void kN(const double kN){kN_=kN;}
    void omegaRDir(const double omegaRDir){omegaRDir_=omegaRDir;}
    void cm(const double cm){cm_=cm;}
    void tauM(const double tauM){tauM_=tauM;}
    void name(const std::string & name){name_=name;}

    // == Misc ==
    void printValues() const;
    void printValues(int nTabs) const;

};





















