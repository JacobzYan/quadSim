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




class sensorTemplate : poseTemplate
{
    // Define stuff like variance/covariance
    // Back calculate predicted pose? - cameras need multiple measurements

    // IMU: Only needs 1 sensor, pose to predict drone pose
    // Cameras/Lidar/GPS: Need either multiple sensors or multiple measurements to determine pose
    // Need to enumerate something for no measurement
    
    private:
        // std::shared_ptr<quadParams> paramsPtr_;
    
    
    public:
        sensorTemplate(){}
        quadState est;
        // omniscientSensor for testing
        quadState estimatePosition(const quadState & state, const enviornment & env) 
        {
            est.pos()=state.pos();
            est.RBI()=state.RBI();
            return est;
        }
};