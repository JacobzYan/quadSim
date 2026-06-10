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
        Eigen::Vector3d location_; // [m] Location of the module with respect to the quad body frame, origin
        Eigen::Matrix3d R_; // Rotation matrix from module coordinate frame to quad body coordinate frame
    public:
        // Constructors
        poseTemplate(const Eigen::Vector3d & location, const::Eigen::Matrix3d & R) // Arg constructor
            : location_(location), R_(R)
            {}
        poseTemplate(){} // Null constructor to reserve memory

        // Getters
        const Eigen::Vector3d& location() const {return location_;}
        const Eigen::Matrix3d& R() const {return R_;}

        // Setters
        void location(const Eigen::Vector3d& location){location_=location;}
        void R(const Eigen::Matrix3d & R){R_=R;}

};


// Class to hold information about propellor
class propParams : public poseTemplate
{
    private:
        double kMotor_; // Motor constant
        double kF_; //  prop lift force coeff
        double kN_; //  prop torque coeff
        int omegaRDir_; // Direction - +1 for CCW, -1 for CW

        double cm_, tauM_; // Motor constant response, and time constant

        std::string name_;

        // Statics
        inline static const std::vector<std::string> varNames = {"location", "R", "kMotor", "kF", "kN", "omegaRDir", "cm", "tauM", "name"}; // List of variable names to recognize
        const int nVars = varNames.size();
        
    public:
    
    // Constructors
    propParams(const double kMotor, const double kF, const double kN) // Arg constructor
        : kMotor_(kMotor), kF_(kF), kN_(kN)
        {}
    propParams(){} // Empty constructor
    propParams(const std::string & line); //String line constructor

    // Getters
    const double kMotor() const {return kMotor_;}
    const double kF() const {return kF_;}
    const double kN() const {return kN_;}
    const int omegaRDir() const {return omegaRDir_;}
    const double cm() const {return cm_;}
    const double tauM() const {return tauM_;}
    const std::string & name() const {return name_;}

    // Setters
    void kMotor(const double kMotor){kMotor_=kMotor;}
    void kF(const double kF){kF_=kF;}
    void kN(const double kN){kN_=kN;}
    void omegaRDir(const double omegaRDir){omegaRDir_=omegaRDir;}
    void cm(const double cm){cm_=cm;}
    void tauM(const double tauM){tauM_=tauM;}
    void name(const std::string & name){name_=name;}

    //Misc
    void printValues() const;
    void printValues(int nTabs) const;

};





















