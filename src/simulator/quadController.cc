
// Want to make a class that will just give you the next state
// Keep all the info that the quad has to make decisions inside Quad
//     Needs:
//         Internal Estimator
//         Sensor readings( as input)
//         Controller

// Make simulator the actual simulation of pose
//     Needs:
//         ODE Function as to whats going on
//         Sensor Error/noise models
//         Vector of quads to simulate
//          Storage of information on the enviornment

// Quad
//     Controller
//         Angular Controller
//         Linear Controller
//     QuadParamsFunc

// Simulator - get passed prop params
//     ODE Func

#include "quadController.h"
#include "quadParams.h"
#include "quadState.h"



trajCtrlPacket PDTrajectoryController::response(
                                    const quadParams & params,
                                    const quadState & state,
                                    // const Eigen::Vector3d & x,
                                    // const Eigen::Vector3d & xDot, 
                                    const Eigen::Vector3d & xDes, 
                                    // const Eigen::Matrix3d & RBI,
                                    const Eigen::Vector3d & xDotDes,                                       
                                    const Eigen::Vector3d & xDotDotDes
                                    )
{
    // Control gains + Feed forward desired accel, gravity, normalize to extract the portion || to zRBI
    FDesVector_ = (kp_*(state.pos()-xDes) + kd_*(state.vel()-xDotDes) - Eigen::Vector3d(0,0,params.g() * params.m()));
    ZDesVector_ = FDesVector_.normalized();
    FDes_= FDesVector_.transpose() * state.RBI().transpose() * normalizeFDesZ;
    response_.ZDesVector = ZDesVector_;
    response_.FDes = FDes_;
    return response_;
}





const Eigen::Vector3d & PDAttitudeController::response(
                                        const quadParams & params,
                                        const quadState & state,
                                        const trajCtrlPacket & trajCtrlOutput,  
                                        const Eigen::Vector3d & yawDes
                                        )
        {
            // X, Y, Z axes of desired
            RDes.row(2)= trajCtrlOutput.ZDesVector;
            RDes.row(1)= trajCtrlOutput.ZDesVector.cross(yawDes).normalized();
            RDes.row(0)= RDes.row(1).cross(trajCtrlOutput.ZDesVector);
            
            RErr = RDes * state.RBI().transpose();
            // Populate Axis Angle Error
            AAErr(0) = RErr(2,3) - RErr(3,2);
            AAErr(1) = RErr(1,3) - RErr(3,1);
            AAErr(2) = RErr(1,2) - RErr(2,1);

            // Proportional, Derivative, Conversion from  body to Inertial 
            response_ = kp_.asDiagonal() * AAErr - kd_.asDiagonal() * state.omegaB() + state.omegaB().asSkewSymmetric() * params.J() * state.omegaB();
            return response_;
        }


        
naiveEstimator::naiveEstimator(std::shared_ptr<quadParams> paramsPtr): paramsPtr_(paramsPtr), IMUPtr(paramsPtr->sensors()[0]), stateEstimatorTemplate(paramsPtr){}

void naievePDController::updateVCBaseMat(const quadParams & params)
{
    /*
    INTENDED STRUCTURE:
    [kf1 * cm1^2 ... kfn * cmn^2]
    [kf1 * y1    ...    kfn * yn]
    [-kf1 * x1   ...   -kfn * xn]
    [kn1, -kn2   ... kn(n-1), -knn]
    */
    Eigen::Matrix<double, 2, 4> temp, temp2;
    temp = params.propLocation().block(1,0,2,4); // Grabs X,Y Pos without editing them
    

    Eigen::Vector4d kTVec = (params.propKn().array() / params.propKf().array() * params.propDir().array()).matrix();
    VCBaseMat.row(0) = Eigen::Vector4d::Ones();
    VCBaseMat.row(1) = temp.row(1); // Puts Y values in row 2, -x values in row 1
    VCBaseMat.row(2) = - temp.row(0);
    VCBaseMat.row(3) = kTVec;

    // SPLIT INTO 2 LINES FOR DEBUG
    // VCBaseMat = (VCBaseMat.array() * paramsPtr_->propKf().array() / paramsPtr_->propCm().transpose().array()).matrix();
    VCBaseMat = VCBaseMat * params.propKf().asDiagonal();
    VCBaseMat = VCBaseMat * params.propCm().asDiagonal().inverse();

    VCFMax = (params.propCm().array().square() * params.propKf().array().square()).sum() * eaMax * eaMax;
}

void naievePDController::getVoltages(Eigen::Vector4d & motorVoltages, const Eigen::Vector3d & NDemand, const double FDemand)
{
        
        const static double dTorqueModifier = .05;
        double torqueModifier = 1;
        Eigen::Vector4d vWorking, eaVec;
        
        while(true)
        {
            vWorking(0) = std::min(FDemand, VCFMax); // Working F value
            vWorking.segment(1,3) = NDemand * torqueModifier; // Working N value

            eaVec = VCBaseMat * vWorking;

            if(eaVec.maxCoeff() < eaMax)
            {
                break;
            }

            torqueModifier -= dTorqueModifier;
            

        }
        motorVoltages = eaVec;


        // Limit max voltage - TODO
        // Set negative F des to 0
        // Determine max force from motor max voltage
        // If force demand from any motor is too high, incrementally decrease demanded torque until no motor torque demand is too high
    }





naievePDController naievePDController::strLineConstructor(const std::string & line, const quadParams params)
{
    // Temp helper variables
    std::string varName;
    std::string varValue; 
    std::string packet;
    int delimiterLocation;
    int varIndex;
    // Data storage
    std::string name = "NA";
    double trajKp, trajKd = 0;
    Eigen::Vector3d attKp, attKd = Eigen::Vector3d::Zero();

    // Process line
    std::string processingLine = line;
    cutWhitespace(processingLine);

    // Seperate out controller parameters with semicolon delimeters
    replaceDelimiters(processingLine,';');
    std::istringstream iss(processingLine);

    
    while(iss >> packet)
    {
        // Seperate out controller parameters
        delimiterLocation = packet.find("=");
        varName = packet.substr(0,delimiterLocation);
        varValue = packet.substr(delimiterLocation+1, packet.size()-delimiterLocation-1);

        // Trim all whitespace
        cutWhitespace(varName);

        // Ensure there is an equals sign
        if(delimiterLocation==std::string::npos)
        {
            std::cout << "THIS PACKET CONTAINS NO EQUALS SIGN DELIMITER:" << std::endl << packet << std::endl;
            continue;
        }

        // Check if line start matches any variable names
        varIndex = -1;
        for(int i=0;i<varNames.size();i++)
        {
            if(varNames[i] == varName)
            {
                varIndex = i;
                break;
            }
        }

        // Trim Whitespace for name, cut all whitespace for others
        if(varIndex==7){trim(varValue);} // Name - preserve internal spaces
        else{cutWhitespace(varValue);}

        // Assign the appropriate value
        switch(varIndex)
        {
            case 0: // name
                name = varValue;
                break;
            case 1: // trajKp
                trajKp = std::stod(varValue);
                break;
            case 2: // trajKd
                trajKd = std::stod(varValue);
                break;
            case 3: // attKp
                attKp = splitVector3d(varValue, ',');
                break;
            case 4: // attKd
                attKd = splitVector3d(varValue, ',');
                break;
            default:
                break;
        }
    }
    
    PDTrajectoryController trajCon(trajKp, trajKd);
    PDAttitudeController attCon(attKp, attKd);
    naiveEstimator Estimator(std::make_shared<quadParams>(params));


    return naievePDController(name, std::make_shared<PDTrajectoryController>(trajCon),std::make_shared<PDAttitudeController>(attCon), std::make_shared<naiveEstimator>(Estimator));
}

std::shared_ptr<naievePDController> strLineConstructorPtr(const std::string & line, const quadParams params)
{
    naievePDController tempController = naievePDController::strLineConstructor( line, params);
    return std::make_shared<naievePDController>
    (
        tempController.name(),
        std::static_pointer_cast<PDTrajectoryController>(std::const_pointer_cast<trajectoryControllerTemplate>(tempController.trajCtrlPtr())), // Need to cast pointer to static obect from getter to be editable again
        std::static_pointer_cast<PDAttitudeController>(std::const_pointer_cast<attitudeControllerTemplate>(tempController.attCtrlPtr())),
        std::static_pointer_cast<naiveEstimator>(std::const_pointer_cast<stateEstimatorTemplate>(tempController.estPtr()))
    );
}

quadState::stateVector naiveEstimator::estState (const std::vector<sensorTemplate*> measSensorPointers, std::vector<std::vector<double>> sensorReadings)
{
    return quadState::stateVector(quadState::VectorNd().data());
}
quadState::stateVector naiveEstimator::estState()
{
 return quadState::stateVector(quadState::VectorNd().data());
}


void naievePDController::getState(enviornment env, quadState & state){}







