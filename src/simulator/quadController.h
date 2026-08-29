#pragma once
#include <Eigen/Dense>
#include <ode/ode_dopri_54.h>


#include "quadState.h"
#include "quadParams.h" 
#include "sensors.h"
#include "utils.h"

// Passing Datastructures
struct trajectoryControllerPacket
{
    Eigen::Vector3d zBI;
    double FDes;
    trajectoryControllerPacket(const Eigen::Vector3d & zBIInput, const double FDesInput){zBI=zBIInput;FDes=FDesInput;}
    trajectoryControllerPacket(){}
};
struct poseEstimate
{
    Eigen::Vector3d position;
    Eigen::Matrix3d RBI;
};
struct controllerDemands
{
    double F; // Total demanded force
    Eigen::Vector3d NB; // Demanded torque vector expressed in the body frame
};

const static int nStates_ = 15, nDynSensor=6;
struct trajectory
{
    double dt;
    std::vector<Eigen::Vector3d> position;
    std::vector<Eigen::Vector3d> velocity;
    std::vector<Eigen::Vector3d> acceleration;
    std::vector<double> heading;
};




// Trajectory Controllers
class trajectoryControllerTemplate
{
    protected:
        // Controller variables - pointers to allow easy modification of change in gains to flow down
        double m_;
        double g_;
        std::shared_ptr<quadParams> paramsPtr_;

        // Helper Variables
        Eigen::Vector3d gVector;
        Eigen::Vector3d FDesVec;
        trajectoryControllerPacket response_;
        

    public:
        // Explicitly set kp and kd
        /* 
            @param m Quad Mass
            @param g Acceleration due to gravity, OPTIONAL (default 9.81 m/s^2)
        */
        trajectoryControllerTemplate(
                        const double m,
                        const double g=9.81
                        )
                        : g_(g), m_(m) {gVector << 0,0,m*g;}

        // Getters
        const double m() const {return m_;}
        const double g() const {return g_;}


        // Setters
        void m(const double m) {m_=m; gVector[2]=m_*g_;}
        void g(const double g) {g_=g; gVector[2]=g_*g_;}
        void mg(const double m, const double g) {m_=m; g_=g; gVector[2]=m_*g_;}

    // Calculate control law response
    /* 
        @param x Quad estimated position in world frame
        @param xDot Quad estimated velocity in world frame
        @param xDes Quad desired position in world frame
        @param RBI Quad current orientation in world frame
        @param xDotDes Quad desired velocity in world frame
        @param xDotDotDes Quad desired acceleration in world frame (feed forward)

    */
    const trajectoryControllerPacket response(
                                    const Eigen::Vector3d & x,
                                    const Eigen::Vector3d & xDot, 
                                    const Eigen::Vector3d & xDes, 
                                    const Eigen::Matrix3d & RBI,
                                    const Eigen::Vector3d & xDotDes = Eigen::Vector3d::Zero(),                                       
                                    const Eigen::Vector3d & xDotDotDes = Eigen::Vector3d::Zero()
                                    );
};

class PDTrajectoryController : public trajectoryControllerTemplate
{
    private:
        // Controller variables - pointers to allow easy modification of change in gains to flow down
        std::shared_ptr<double> kpPtr_;
        std::shared_ptr<double> kdPtr_;
        double m_;
        double g_;
        std::shared_ptr<quadParams> paramsPtr_;
        // Helper Variables
        Eigen::Vector3d gVector;
        Eigen::Vector3d FDesVec;
        Eigen::Vector3d normalizeFDesZ;
        trajectoryControllerPacket response_;
        

    public:
        // Explicitly set kp and kd
        /*
        @todo Evaluate if kp and kd can just be stored by value
        @param kpPtr Pointer to the proportional gain of the controller
        @param kdPtr Pointer to the derivative gain
        @param m Quad Mass
        @param g Acceleration due to gravity, OPTIONAL (default 9.81 m/s^2)
        
        */
        PDTrajectoryController(
                        std::shared_ptr<double> kpPtr, 
                        std::shared_ptr<double> kdPtr,
                        const double m,
                        const double g=9.81
                        
                        )
                        : trajectoryControllerTemplate(m, g), kpPtr_(kpPtr), kdPtr_(kdPtr), g_(g), m_(m) {gVector << 0,0,m*g; normalizeFDesZ << 0,0,1;}

        // Getters
        const double kp() const {return * kpPtr_ ;}
        const double kd() const {return * kdPtr_ ;}

        // Setters
        void kp(const double kp) {* kpPtr_ = kp;}
        void kd(const double kd) {* kdPtr_ = kd;}
        void m(const double m) {m_=m; gVector[2]=m_*g_;}
        void g(const double g) {g_=g; gVector[2]=g_*g_;}
        void mg(const double m, const double g) {m_=m; g_=g; gVector[2]=m_*g_;}

    // Calculate control law response
    const trajectoryControllerPacket response(
                                    const Eigen::Vector3d & x,
                                    const Eigen::Vector3d & xDot, 
                                    const Eigen::Vector3d & xDes, 
                                    const Eigen::Matrix3d & RBI,
                                    const Eigen::Vector3d & xDotDes = Eigen::Vector3d::Zero(),                                       
                                    const Eigen::Vector3d & xDotDotDes = Eigen::Vector3d::Zero()
                                    );
};


// Attitude Controllers
class attitudeControllerTemplate
{
        protected:
        // Controller variables - pointers to allow easy modification of change in gains to flow down
        std::shared_ptr<quadParams> paramsPtr_;

        // Helper Variables
        Eigen::Vector3d FIDes;
        Eigen::Vector3d FFGravity;
        Eigen::Vector3d response_ = Eigen::Vector3d::Zero();

    public:
    // Explicitly set kp and kd
        /*
        @param kpPtr pointer to a 3d vector of propritional gains
        @param kdPtr pointer to a 3d vector of derivative gains
        @param paramsPtr pointer to the quadParams referenced
        */
        attitudeControllerTemplate(std::shared_ptr<quadParams> paramsPtr): paramsPtr_(paramsPtr) {}

        // Calculate control law response
        /*
        @param RBI rotation matrix of quad body
        @param FDes Desired Force Vector
        @param yawDes Desired angle of the x axis of the quad body in radians
        */
        const Eigen::Vector3d & response(
                                        const Eigen::Matrix3d & RBI,
                                        const Eigen::Vector3d & FDes, 
                                        const Eigen::Vector3d & yawDes
                                        );
};

class PDAttitudeController : public attitudeControllerTemplate
{
        private:
        
        double m_;
        double g_;
        std::shared_ptr<quadParams> paramsPtr_;
        // Helper Variables
        Eigen::Vector3d FIDes;
        Eigen::Vector3d FFGravity;
        Eigen::Vector3d response_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d tempYI = Eigen::Vector3d::Zero();
        Eigen::Vector3d tempXI = Eigen::Vector3d::Zero();
        Eigen::Matrix3d RDes =Eigen::Matrix3d::Zero();
        Eigen::Matrix3d RErr =Eigen::Matrix3d::Zero();
        Eigen::Vector3d AAErr = Eigen::Vector3d::Zero();

        std::shared_ptr<Eigen::Vector3d> kpPtr_, kdPtr_;


    public:
    // Explicitly set kp and kd
        PDAttitudeController(
                        const quadParams & params,
                        std::shared_ptr<Eigen::Vector3d> kpPtr, 
                        std::shared_ptr<Eigen::Vector3d> kdPtr,
                        const double m,
                        const double g=9.81
                        )
                        : attitudeControllerTemplate(std::make_shared<quadParams>(params)), kpPtr_(kpPtr), kdPtr_(kdPtr), g_(g), m_(m) {AAErr << 0,0,0;}

        // Getters
        const Eigen::Vector3d kp() const {return * kpPtr_ ;}
        const Eigen::Vector3d kd() const {return * kdPtr_ ;}

        // Setters
        void kp(const Eigen::Vector3d kp) {* kpPtr_ = kp;}
        void kd(const Eigen::Vector3d kd) {* kdPtr_ = kd;}

        // Calculate control law response
        const Eigen::Vector3d & response(
                                        const Eigen::Matrix3d & RBI,
                                        const Eigen::Vector3d & xDes, 
                                        const Eigen::Vector3d & zDes, 
                                        const Eigen::Vector3d & omegaB,
                                        const Eigen::Matrix3d & JB
                                        );
};


// State Estimators
class stateEstimatorTemplate
{
    protected:
        
        // Reference Info storage
        std::shared_ptr<quadParams> paramsPtr_;
        std::shared_ptr<enviornment> env_Ptr;
        
        

    public:

        // Output
        quadState::VectorNd estStateMemory = quadState::VectorNd::Zero(); // Need to allocate memory for the map
        quadState::stateVector estState_; 

        // Constructors
        stateEstimatorTemplate(std::shared_ptr<quadParams> paramsPtr): paramsPtr_(paramsPtr), estState_(estStateMemory.data())
        {
            
        }

        // Estimate the state given the sensors and their respective readings - possibly make this just pull from quadParams?
        virtual quadState::stateVector estState (const std::vector<sensorTemplate*> measSensorPointers, std::vector<std::vector<double>> sensorReadings);
        virtual quadState::stateVector estState(); // PLACEHOLDER FOR NAIEVE ESTIMATOR
};

class ukfEstimator : public stateEstimatorTemplate
{
    private:
        // Sigma Point Spacing Parameters
        int nStates=nStates_, nProp, nCorrect, nh, nzMeas, nzDyn; //nStates = number of state variables, nProp = number of measurements that affect propogation, nCorrect = number of measurements that are used to correct the current state 
        double alpha, beta, mu, lambda;
        
        const static int nf = nStates_ + nDynSensor; // n states in the dyanmics function
        const static int nSigmaf = 2*nf+1;
        const static int nReserve = std::max(50, nSigmaf); // Max number of states to reserve, downsized with conservativeResize() in implementation

        using augDynVector = Eigen::Vector<double, nf>;
        // using augMeasVector = Eigen::Vector<double, nReserve>;

        augDynVector predAugState, WDyn;
        Eigen::VectorXd predAugMeas, WMeas;


        Eigen::Matrix<double, nStates_, nSigmaf> dynSigmaPoints;
        Eigen::MatrixXd sigmaPoints, measSigmaPoints, sigmaPointMatrixH, Q, R, P, Pxz, Pzz, S;
        int sign=-1;

        quadState::VectorNd estStateMemory;
        quadState::stateVector estState_; // Need to allocate memory for the map
        Eigen::Matrix<double, nStates_, nReserve> sigmaPointMatrixF;

    public:
        ukfEstimator(std::shared_ptr<quadParams> paramsPtr): stateEstimatorTemplate(paramsPtr), nh(0), estState_(estStateMemory.data())
        {
            
        }
        quadState::stateVector estState (const std::vector<sensorTemplate*> measSensorPointers, std::vector<std::vector<double>> sensorReadings);

        const std::vector<std::vector<double>> hMeas(const std::vector<sensorTemplate*>, const quadState::stateVector & state);
        const quadState::stateVector fDyn(const Eigen::Vector<double, 6> & IMUReading, const quadState::stateVector & state);
    
};

class naiveEstimator : public stateEstimatorTemplate // WIP
{
    private:
        const sensorTemplate * IMUPtr;

    public:
        naiveEstimator(std::shared_ptr<quadParams> paramsPtr): IMUPtr(paramsPtr->sensors()[0]), stateEstimatorTemplate(paramsPtr) // REWORK THIS TO USE SMART POINTERS - take time to go through repo and replace all normal pointers with smart
        {}

};




// Controllers
enum controllerTypes {UnknownController, naievePDControllerType};
class quadControllerTemplate
{
    protected:
        // These not needed?
        std::shared_ptr<trajectoryControllerTemplate> trajCtrlPtr_; 
        std::shared_ptr<attitudeControllerTemplate> attCtrlPtr_; 
        std::shared_ptr<stateEstimatorTemplate> estPtr_; 
    
        // std::vector<std::shared_ptr<sensorTemplate>> sensors_; // This info should be accessible in the paramsPtr_
        std::shared_ptr<quadParams> paramsPtr_;
        
        public:
        quadControllerTemplate();
        quadControllerTemplate
        (
            std::shared_ptr<trajectoryControllerTemplate> trajCon, 
            std::shared_ptr<attitudeControllerTemplate> attCon,
            std::shared_ptr<stateEstimatorTemplate> estCon
        ):
            trajCtrlPtr_(trajCon), 
            attCtrlPtr_(attCon),
            estPtr_(estCon){}
        quadControllerTemplate();
        // Virtual Functions
        virtual controllerDemands getDemands(quadState state, trajectory traj);
        virtual void getVoltages(Eigen::Vector4d* motorVoltages, const Eigen::Vector3d & NDemand, const double FDemand);
        virtual void getState(enviornment env, quadState & state);
        virtual void updateParams();
};

class naievePDController : public quadControllerTemplate // Transistion to having the whole controller be uninherited?
{
    private:
        Eigen::Matrix4d VCBaseMat;
        double VCFMax;
        static constexpr double eaMax = 12; // Later this should be a quad parameter
        inline static const std::array<std::string, 5> varNames = {"name", "trajKp", "trajKd", "attKp", "attKd"};

        std::string name_;
        std::shared_ptr<PDTrajectoryController> trajCtrlPtr_; 
        std::shared_ptr<PDAttitudeController> attCtrlPtr_; 
        std::shared_ptr<naiveEstimator> estPtr_; 
    
        public:

    double placholderTrajKp = 1;
    double placeholderTrajKd = 1;


    // Getters and setters
    const std::string name() const {return name_;}
    const double trajKp() const {return trajCtrlPtr_->kp();}
    const double trajKd() const {return trajCtrlPtr_->kd();}
    const Eigen::Vector3d attKp() const {return attCtrlPtr_->kp();}
    const Eigen::Vector3d attKd() const {return attCtrlPtr_->kd();}

    void name(const std::string & name){name_=name;}
    void trajKp(const double & trajKp){trajCtrlPtr_->kp(trajKp);}
    void trajKd(const double & trajKd){trajCtrlPtr_->kd(trajKd);}
    void attKp(const Eigen::Vector3d & attKp){attCtrlPtr_->kp(attKp);}
    void attKd(const Eigen::Vector3d & attKd){attCtrlPtr_->kd(attKd);}

    

    Eigen::Vector3d placeholderAttKp = Eigen::Vector3d::Ones();
    Eigen::Vector3d placeholderAttKd = Eigen::Vector3d::Ones();
    


    naievePDController
    (
        std::shared_ptr<quadParams> quadParamsPtr,
        const double trajKp=1,
        const double trajKd=1,
        const Eigen::Vector3d attKp = Eigen::Vector3d::Ones(),
        const Eigen::Vector3d attKd = Eigen::Vector3d::Ones()
    ): 
        quadControllerTemplate(std::make_shared<PDTrajectoryController>(new PDTrajectoryController(std::make_shared<double>(trajKp), std::make_shared<double>(trajKd),quadParamsPtr->m())), attCtrlPtr_, estPtr_),
        VCBaseMat(Eigen::Matrix4d::Zero())
        {updateVCBaseMat();}

    naievePDController(const std::string & line): quadControllerTemplate()
    {
        // Temp helper variables
        std::string varName;
        std::string varValue; 
        int delimiterLocation;
        int varIndex;

        // Process line
        std::string processingLine = line;
        cutWhitespace(processingLine);

        // Seperate out controller parameters with semicolon delimeters
        replaceDelimiters(processingLine,';');
        std::istringstream iss(processingLine);

        std::string packet;
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
                    name(varValue);
                    break;
                case 1: // trajKp
                    trajKp(std::stod(varValue));
                    break;
                case 2: // trajKd
                    trajKd(std::stod(varValue));
                    break;
                case 3: // attKp
                    attKp(splitVector3d(varValue, ','));
                    break;
                case 4: // attKd
                    attKd(splitVector3d(varValue, ','));
                    break;
                default:
                    break;
            }
        }
    }
    
    void updateVCBaseMat()
    {
        /*
        INTENDED STRUCTURE:
        [kf1 * cm1^2 ... kfn * cmn^2]
        [kf1 * y1    ...    kfn * yn]
        [-kf1 * x1   ...   -kfn * xn]
        [kn1, -kn2   ... kn(n-1), -knn]
        */
        Eigen::Matrix<double, 2, 4> temp, temp2;
        temp = paramsPtr_->propLocation().block(1,0,2,4); // Grabs X,Y Pos without editing them
       

        Eigen::Vector4d kTVec = (paramsPtr_->propKn().array() / paramsPtr_->propKf().array() * paramsPtr_->propDir().array()).matrix();
        VCBaseMat.row(0) = Eigen::Vector4d::Ones();
        VCBaseMat.row(1) = temp.row(1); // Puts Y values in row 2, -x values in row 1
        VCBaseMat.row(2) = - temp.row(0);
        VCBaseMat.row(3) = kTVec;

        // SPLIT INTO 2 LINES FOR DEBUG
        // VCBaseMat = (VCBaseMat.array() * paramsPtr_->propKf().array() / paramsPtr_->propCm().transpose().array()).matrix();
        VCBaseMat = VCBaseMat * paramsPtr_->propKf().asDiagonal();
        VCBaseMat = VCBaseMat * paramsPtr_->propCm().asDiagonal().inverse();

        VCFMax = (paramsPtr_->propCm().array().square() * paramsPtr_->propKf().array().square()).sum() * eaMax * eaMax;
    }


    // Assumes motors face up
    void getVoltages(Eigen::Vector4d* motorVoltages, const Eigen::Vector3d & NDemand, const double FDemand) override
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
        * motorVoltages = eaVec;


        // Limit max voltage - TODO
        // Set negative F des to 0
        // Determine max force from motor max voltage
        // If force demand from any motor is too high, incrementally decrease demanded torque until no motor torque demand is too high
    }

    void getState(enviornment env, quadState & state) // Later need to make these more generic, move computation to derived classes that this holds
    {
        estPtr_->estStateMemory = state.stateAsVec();
    }
        
};


// Read in type of controller from the data only
controllerTypes readControllerType(const std::string & line)
{
    // Controller Names
    std::array<std::string, 1> controllerNames = {"naievePD"};

    // Temp helper variables
    std::string varName, varValue, controllerName, packet;
    int delimiterLocation, controllerIndex;

    // Process line
    std::string processingLine = line;
    cutWhitespace(processingLine);

    // Seperate out prop parameters with semicolon delimeters
    replaceDelimiters(processingLine,';');
    std::istringstream iss(processingLine);

    
    while(iss >> packet)
    {
        // Seperate out prop parameters
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

        // Check if this packet has the controller type
        controllerIndex = -1;
        for(int i=0;i<controllerNames.size();i++)
        {
            if(controllerNames[i] == varName)
            {
                controllerIndex = i;
                break;
            }
        }

        // Skip to future packets if type is not here
        if(controllerIndex==-1)
        {
            continue;
        }


        // Assign the appropriate controller constructor
        switch(controllerIndex)
        {
            case 0: // naieve PD Controller
                // Construct PD controller here - pass the rest of the iss as data for the controller?
                // naievePDController foo(processingLine);
                return naievePDControllerType;
                break;

            default:
                return UnknownController;
                break;
        }
    }
}