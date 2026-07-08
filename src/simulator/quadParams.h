
// Class to store/get params
// Add function to pull params from some external file, define interfaces
#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <memory>

#include "poseModules.h"

// Forward declare
class sensorTemplate;


class quadParams
{
private:
    
    // Initialization temp variables
    std::string line; // Line to hold location
    int delimiterLocation; // Holds index of comma in a line
    std::string varName;
    std::string varValue;
    int varIndex;

    // Statics
    inline static const std::vector<std::string> varNames = {"g", "pAir", "m", "Cd", "Ad", "J", "xCOM", "prop", "name", "sensor"}; // List of variable names to recognize -CHANGE THIS TO ENUM IN THE FUTURE
    inline static const int nVars = varNames.size();
    
    // Settings
    std::string name_;

    // Constants
    double g_; // [m/s^2] acceleration due to gravity
    double pAir_; // [kg/m^3] air density

    // Quad Physical Params
    double m_; // [kg] mass of the quad
    double Cd_; // Quad body drag coefficient
    double Ad_; // Quad body drag area from top
    Eigen::Matrix3d J_; // [kg m^2] Inertia matrix of the quad at the center of mass
    Eigen::Vector3d xCOM_; // [m] Location of the center of mass relative to the origin
    std::vector<const propParams*> props_; // Vector of pointers to prop objects

    // Quad Controller Params
    std::shared_ptr<double> gPtr_ = std::make_shared<double>(g_);
    std::shared_ptr<double> mPtr_ = std::make_shared<double>(m_);
    
    // Sensor Params
    std::vector<const sensorTemplate*> sensors_; // Vector of pointers to sensor objects
    const inline static std::vector<std::string> sensorTypes = {"Default", "IMU", "Camera"}; // Types of sensors used for config file reading init


    // Controller Params
    // Implement pointer for quadController

public:
    // Constructors + Destructor
    quadParams(const std::vector<std::string> configPaths){setParams(configPaths);} // csv Constructor
    quadParams(){} // Null constructor to reserve memory
    ~quadParams() // Only need to delete pointers for props
    {
        std::cout << "Deallocating Props for quad: " << name() << std::endl; // DEBUG
        for(int i=props_.size()-1;i>=0;i--){delete props_[i];}
        std::cout << "\tProps Deallocated" << std::endl; // DEBUG
    } 

    // Copy Constructors
    quadParams(const quadParams & other): // Copy constructor (Used when you construct a new copy of an existing obj ie. quadParams newParams = oldParams;)
        g_(other.g()),
        pAir_(other.pAir()),
        m_(other.m()),
        Cd_(other.Cd()),
        Ad_(other.Ad()),
        J_(other.J()),
        xCOM_(other.xCOM()),
        name_(other.name())
    {
        for(const propParams* p : other.props_){props_.push_back(new propParams(*p));}// Allocate new deep copys
    }
    quadParams & operator=(const quadParams& other) // Copy assignment operator (Used to set existing objects to contain the same data ie. quadParams p1(config1); quadParams p2(config2); p1=p2;)
    {
        if(this == &other) return *this; // Protects agains setting obj equal to itself (ie. p1=p1;)

        // Clear out this obj's props from the end
        for(int i = props_.size()-1; i>=0; i--){delete props_[i];} // Delete pointers
        props_.clear(); // Clear out props_

        // Re-populate with copies of props pointed to by other.props_
        for(const propParams* p : other.props_) {props_.push_back(new propParams(*p));}

        return *this; // Returns ref to self to allow for chained asignements (ie. a=b=c)
    }


    // Getters - Complex structures returned by ref
    const double g()    const {return g_;} //[m/s^2] acceleration due to gravity
    const double pAir() const {return pAir_;} // [kg/m^3] air density
    const double m()    const {return m_;} // [kg] mass of the quad
    const double Cd()   const {return Cd_;} // quad body drag coefficient
    const double Ad()   const {return Ad_;} // quad body top plane drag area
    const Eigen::Matrix3d& J()    const {return J_;} // [kg m^2] Inertia matrix of the quad at the center of mass
    const Eigen::Vector3d& xCOM() const {return xCOM_;} // [m] Location of the center of mass relative to the origin
    const std::vector< const propParams*>& props() const {return props_;} // Returns reference to vector of pointers to prop objects
    const std::vector<const sensorTemplate*>& sensors() const {return sensors_;} // Returns referense to sensors
    const std::string & name() const {return name_;}

    // Setters
    void g(const double g) {g_=g;} //[m/s^2] acceleration due to gravity - You probably shouldn't be setting this...
    void pAir(const double pAir) {pAir_=pAir;} // [kg/m^3] air density - You probably shouldn't be setting this...
    void m(const double m) {m_=m;} // [kg] quad mass
    void Cd(const double Cd) {Cd_=Cd;} // quad body drag coefficient
    void Ad(const double Ad) {Ad_=Ad;} // quad body top plane drag area
    void J(const Eigen::Matrix3d& J){J_=J;} // [kg m^2] Inertia matrix of the quad at the center of mass
    void xCOM(const Eigen::Vector3d& xCOM){xCOM_=xCOM;} // [m] Location of the center of mass relative to the origin
    void props(const std::vector<const propParams*>& props){props_=props;} // Returns reference to vector of pointers to prop objects
    void name(const std::string & name){name_=name;}

    // Specialized setters
    const bool setProp(const propParams* prop, const int i) // Set the propParams object at index i, returns true if successful
    {
        if(props_.size() > i-1){props_[i]=prop;return true;}
        else{return false;}
    } 
    void addProp(const propParams* prop_ptr){props_.push_back(prop_ptr);} // Add a prop to the vector
    void addSensor(std::string line); // Implemented in quadParams.cc

    // config initialization functions
    bool readFile(std::string path);
    void setParams(const std::vector<std::string> configPaths);

    //Misc
    void printValues() const;
    void printValues(int nTabs) const;

    
};




struct enviornment
{
    int test=0;
    double time;
    double dt;
};