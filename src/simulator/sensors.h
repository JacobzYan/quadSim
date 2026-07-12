#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <fstream>

#include "poseModules.h"
#include "quadParams.h"
#include "quadState.h"
// Implemented as header only classes


// Incorporate types of sensors as enumerated "modes"? - for testing starting with an omnicient sensor
// Enumerated Types
enum class SensorClass
{
    IMU, // Able to determine rates 
    LOB, // Sensor type that can determine a line of bearing to an object of known location
};
// // Likely can depricate this, control through a more general sensor type
// enum class dataFusionMode
// {
//     Independent,
//     Group
// };



// Template for future sensors. By default, acts as an omnicient IMU. call getMeas() then access the public member variable "est"
class sensorTemplate : poseTemplate
{
    // Define stuff like variance/covariance
    // Back calculate predicted pose? - cameras need multiple measurements

    // IMU: Only needs 1 sensor, pose to predict drone pose
    // Cameras/Lidar/GPS: Need either multiple sensors or multiple measurements to determine pose
    // Need to enumerate something for no measurement
    
    private:
        inline static const quadState::VectorNd nullVec = quadState::VectorNd::Constant(std::numeric_limits<double>::quiet_NaN());
    public:
        quadState::VectorNd estMemory = quadState::VectorNd::Zero();
        quadState est;
        std::string type = "Default";
        std::string name = "Default";

        // Constructors
        sensorTemplate(): est(estMemory.data()){}
        
        // default sensor is omnicient
        void getMeas(quadState * qState, const enviornment env) // CHANGE THIS, MAKE GETTERS CONST THEN MAKE qState TAKE CONST
        {
            est.pos(qState->pos());
            est.vel(qState->vel());
            est.omegaB(qState->omegaB());
        }

        // Misc
        void printValues() const;
        void printValues(int nTabs) const;

};



class IMUSensor : sensorTemplate
{
    private:

    public:
        IMUSensor(){}
        IMUSensor(const std::string & line);
};

class cameraSensor : sensorTemplate
{
    private:

    public:
        cameraSensor(){}
        cameraSensor(const std::string & line);
};