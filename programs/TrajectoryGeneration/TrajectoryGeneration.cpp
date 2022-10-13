// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TrajectoryGeneration.hpp"

#include <algorithm>

#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <yarp/os/LogStream.h>
#include <KdlVectorConverter.hpp>

using namespace roboticslab::KinRepresentation;
using namespace roboticslab::KdlVectorConverter;

#define DEFAULT_MAX_DIFF_INV 0.0000001
#define DEFAULT_PREFIX "/trajectoryGeneration/"
#define DEFAULT_DEVICE_NAME "trunkAndRightArm"
#define DEFAULT_KINEMATICS_CONFIG "teo-trunk-rightArm-fetch.ini"
#define DEFAULT_RANGE_RRT 1.0
#define DEFAULT_PRUNE_THRESHOLD 0.5
#define DEFAULT_MAX_PLANNER_TIME 5.0


std::vector<std::string>availablePlanners = {"RRTConnect", "RRTStar"};


namespace errorsTrajectoryGeneration{

  const std::string goal_not_inv_kin ("invKin failed for goal pose");
  const std::string goal_collision("robot collides at the goal configuration");
  const std::string pose_6_elements("pose list must have 6 elements (x, y, z, rotx, roty, rotz)");
  const std::string joints_elements("size of joints list is different than the number of joints");
  const std::string start_collision("robot collides at the start configuration");
  const std::string path_not_found("path NOT found");
  const std::string joints_outside_bounds("joints outside bounds");
  const std::string not_get_current_Q("could NOT get encoders values");
};


static KDL::Chain makeTeoTrunkAndRightArmKinematicsFromDH()
{
    const KDL::Joint rotZ(KDL::Joint::RotZ);
    const KDL::Joint fixed(KDL::Joint::None); //Rigid Connection
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.0, -KDL::PI / 2, 0.1932, 0.0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.305, 0.0, -0.34692, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.32901, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, KDL::PI / 2, 0, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.215, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(-0.09, 0, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0, KDL::PI / 2, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0, 0, 0.0975, 0)));

    return chain;
}
/************************************************************************/

static KDL::Chain makeTeoTrunkAndLeftArmKinematicsFromDH()
{
    const KDL::Joint rotZ(KDL::Joint::RotZ);
    const KDL::Joint fixed(KDL::Joint::None); //Rigid Connection
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.0, -KDL::PI / 2, 0.1932, 0.0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.305, 0.0, 0.34692, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.32901, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, KDL::PI / 2, 0, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.215, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(-0.09, 0, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0, KDL::PI / 2, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0, 0, 0.0975, 0)));

    return chain;
}
/************************************************************************/

static KDL::Chain makeTeoFixedTrunkAndRightArmKinematicsFromDH(){
    const KDL::Joint rotZ(KDL::Joint::RotZ);
    const KDL::Joint fixed(KDL::Joint::None); //Rigid Connection

    KDL::Chain chain;
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0.0, -KDL::PI/2, 0.1932,0.0)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0.305, 0.0, 0.34692, -KDL::PI/2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0, -KDL::PI / 2,        0,            0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0, -KDL::PI / 2,        0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0, -KDL::PI / 2, -0.32901, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0,  KDL::PI / 2,        0,            0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0, -KDL::PI / 2,   -0.215,            0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(-0.09,            0,        0, -KDL::PI / 2)));

    return chain;
}

/************************************************************************/

static KDL::Chain makeTeoFixedTrunkAndLeftArmKinematicsFromDH(){
    const KDL::Joint rotZ(KDL::Joint::RotZ);
    const KDL::Joint fixed(KDL::Joint::None); //Rigid Connection

    KDL::Chain chain;
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0.0, -KDL::PI / 2, 0.1932, 0.0)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0.305, 0.0, 0.34692, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.32901, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, KDL::PI / 2, 0, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.215, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(-0.09, 0, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0, KDL::PI / 2, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0, 0, 0.0975, 0)));

    return chain;
}

void TrajectoryGeneration::changeJointsLimitsFromConfigFile(KDL::JntArray & qlim, const yarp::os::Searchable& config, const std::string& mode){
    if(!config.check(mode))
    {
        yInfo()<<"q"<<mode<<"limits NOT defined in config file. Continue with the limits defined in the IControlLimits devices.";
    }else{
        yarp::os::Bottle * qConfig = config.find(mode).asList();
        for(int i=0; i<numJoints; i++){
            if(qConfig->get(i).asString()!= "D" && !qConfig->get(i).isNull()){
                qlim(i) =std::stof(qConfig->get(i).asString());
            }
        }
    }
}


/************************************************************************/

bool TrajectoryGeneration::open(yarp::os::Searchable& config)
{
    robot = config.check("robot", yarp::os::Value(DEFAULT_ROBOT), "name of /robot to be used").asString();
    
    planningSpace = config.check("planningSpace", yarp::os::Value(DEFAULT_PLANNING_SPACE), "planning space").asString();
    deviceName = config.check("deviceName", yarp::os::Value(DEFAULT_DEVICE_NAME), "device name").asString();

    kinematicsConfig = config.check("kinematicsConfig", yarp::os::Value(DEFAULT_KINEMATICS_CONFIG), "kinematics config").asString();
    plannerType = config.check("planner", yarp::os::Value(DEFAULT_PLANNER), "planner type").asString();
    plannerRange = config.check("plannerRange", yarp::os::Value(DEFAULT_RANGE_RRT), "range the planner is supposed to use").asFloat64();
    pruneThreshold = config.check("pruneThreshold", yarp::os::Value(DEFAULT_PRUNE_THRESHOLD), "prune theshold used for RRTStar").asFloat64();
    maxPlannerTime = config.check("maxPlannerTime", yarp::os::Value(DEFAULT_MAX_PLANNER_TIME), "seconds the algorithm is allowed to spend planning").asFloat64();
    prefix = "/trajectoryGeneration/"+deviceName;
    printf("TrajectoryGeneration using robot: %s\n",robot.c_str());
    printf("TrajectoryGeneration using planningSpace: %s\n",planningSpace.c_str());
    printf("TrajectoryGeneration using deviceName: %s\n",deviceName.c_str());
    printf("TrajectoryGeneration using kinematicsConfig: %s\n",kinematicsConfig.c_str());
    printf("TrajectoryGeneration using plannerType: %s\n", plannerType.c_str());
    printf("TrajectoryGeneration using plannerRange: %f\n", plannerRange);
    printf("TrajectoryGeneration using pruneThreshold: %f\n", pruneThreshold);


    printf("--------------------------------------------------------------\n");
    if (config.check("help"))
    {
        printf("TrajectoryGeneration options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--robot: %s [%s]\n", robot.c_str(), DEFAULT_ROBOT);
        ::exit(0);
    }

    if(!openDevices())
        return false;


    //  Getting the limits of each joint
    printf("---- Joint limits of %s\n",deviceName.c_str());
    qmin.resize(numJoints);
    qmax.resize(numJoints);
    m_qmin.resize(numJoints);
    m_qmax.resize(numJoints);

    for(unsigned int joint = 0; joint < numJoints; joint++){
        double min, max;
        iControlLimits->getLimits(joint, &min, &max);
        qmin(joint) = min;
        qmax(joint) = max;
        m_qmin[joint] = min;
        m_qmax[joint] = max;
    }

    changeJointsLimitsFromConfigFile(qmin,config, "qmin");
    changeJointsLimitsFromConfigFile(qmax,config, "qmax");


    for(unsigned int joint =0; joint<numJoints; joint++){
        yInfo("Joint %d limits: [%f,%f]", joint, qmin(joint), qmax(joint));
        qrMin.addFloat64(qmin(joint));
        qrMax.addFloat64(qmax(joint));
    }

    // LIMITS IN RADIANS FOR 
    qminRad.resize(numJoints);
    qmaxRad.resize(numJoints);

    for(unsigned int joint =0; joint<numJoints; joint++){
        yInfo("Joint %d limits: [%f,%f]", joint, qmin(joint), qmax(joint));
        qminRad(joint)=qmin(joint)*KDL::deg2rad;
        qmaxRad(joint) = qmax(joint)*KDL::deg2rad;
    }

    // ----- Configuring KDL Solver for device -----

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("kinematics"); // context to find kinematic config files
    std::string kinPath = rf.findFileByName(kinematicsConfig);
    armSolverOptions.fromConfigFile(kinPath);
    armSolverOptions.put("device", "KdlSolver");
    armSolverOptions.put("mins", yarp::os::Value::makeList(qrMin.toString().c_str()));
    armSolverOptions.put("maxs", yarp::os::Value::makeList(qrMax.toString().c_str()));
    armSolverOptions.put("ik", "nrjl"); // to use screw theory IK
    armSolverOptions.put("maxIter", 50000);
    armSolverOptions.put("eps", 0.0015);
    armSolverDevice.open(armSolverOptions);
    if (!armSolverDevice.isValid())
    {
        yError() << "KDLSolver solver device for "<<deviceName<<" is not valid";
        return false;
    }

    if (!armSolverDevice.view(armICartesianSolver))
    {
        yError() << "Could not view iCartesianSolver in KDLSolver";
        return false;
    }

    yInfo() << "Acquired armICartesianSolver interface";

    yInfo() << "Running";
    std::vector<double> position;

    if (!rpcServer.open(prefix + "/rpc:s"))
    {
        yError() << "Unable to open RPC server port" << rpcServer.getName();
        return false;
    }


    m_clientGetGraspingPoses.open("/client");
    yarp::os::Network::connect("/client", "/getGraspingPoses/rpc:s");
    

    //Init collisions objects
    if(deviceName == "trunkAndRightArm")
        chain = makeTeoTrunkAndRightArmKinematicsFromDH();
    else if (deviceName == "rightArm")
    {
        chain = makeTeoFixedTrunkAndRightArmKinematicsFromDH();
    }
    else if(deviceName == "trunkAndLeftArm")
    {
        chain = makeTeoTrunkAndLeftArmKinematicsFromDH();
    }
    else if(deviceName == "leftArm"){
        chain = makeTeoFixedTrunkAndLeftArmKinematicsFromDH();
    }
    else{
        yError()<<"Invalid deviceName. Options: trunkAndRightArm, rightArm, trunkAndLeftArm, leftArm";
    }

    unsigned int nj = chain.getNrOfJoints();


    yInfo()<<"offset collisions created";

    if(deviceName != "trunkAndRightArm"){
        yError()<<"No trunkAndRightArm device";
        return false;
    }


    rf.setDefaultContext("kinematics");
    std::string kinematicsFileFullPath = rf.findFileByName( "teo-trunk-rightArm-fetch.ini" );

    rf.setDefaultContext("teoCheckSelfCollisions");
    std::string selfCollisionsFileFullPath = rf.findFileByName( "teo-trunk-RightArm-fetch-collisions.ini");

    rf.setDefaultContext("teoCheckCollisions");
    std::string fixedObjectsFileFullPath = rf.findFileByName("fixed-table-collision.ini");

    yInfo()<<kinematicsFileFullPath;
    yInfo()<<selfCollisionsFileFullPath;
    yInfo()<<fixedObjectsFileFullPath;



    m_checkCollisions = new TeoCheckCollisionsLibrary(fixedObjectsFileFullPath);
    m_checkCollisions->setSelfCollisionsFileFullPath(selfCollisionsFileFullPath);
    m_checkCollisions->setKinematicsFileFullPath(kinematicsFileFullPath);
    m_checkCollisions->setQMin(m_qmin);
    m_checkCollisions->setQMax(m_qmax);
    m_checkCollisions->configureCollisionObjects();
    m_checkCollisions->getBoxShapes(m_boxShapes);
    m_checkCollisions->configureEnvironmentFixedObjects();
    m_checkCollisions->getBoxShapesFixedObjects(m_boxShapesFixedObjects);

    nlopt_opt opt_obo;
    opt_obo = nlopt_create(NLOPT_LN_BOBYQA, numJoints);
    iksolver = new  TRAC_IK::TRAC_IK(chain, qminRad, qmaxRad, timeout_in_secs, 0.002, TRAC_IK::Speed);  
    boundsSolver.vel.x(0.0005);
    boundsSolver.vel.y(0.0005);
    boundsSolver.vel.z(0.0005);
    boundsSolver.rot.x(0.01);
    boundsSolver.rot.y(0.01);
    boundsSolver.rot.z(0.01);




    
    if(planningSpace == "cartesian"){ // cartesian space
        space = ob::StateSpacePtr(new ob::SE3StateSpace());
        ob::RealVectorBounds bounds{3};

        bounds.setLow(0, -0.0);
        bounds.setHigh(0, 0.6);

        bounds.setLow(1, -0.8);
        bounds.setHigh(1, 0.2);

        bounds.setLow(2, -0.1);
        bounds.setHigh(2, 0.6);

        space->as<ob::SE3StateSpace>()->setBounds(bounds);
    
        //space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        si->setStateValidityChecker(std::bind(&TrajectoryGeneration::isValid, this, std::placeholders::_1));
        si->setup();

        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
    }
    else{// joint space
        space = ob::StateSpacePtr(new ob::RealVectorStateSpace(nj));
        ob::RealVectorBounds bounds{nj};
        for(unsigned int j=0; j<nj; j++){
            bounds.setLow(j, qmin(j)+MARGIN_BOUNDS);
            bounds.setHigh(j, qmax(j)-MARGIN_BOUNDS);
        }
        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        si->setStateValidityChecker(std::bind(&TrajectoryGeneration::isValid, this, std::placeholders::_1));
        si->setup();

        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
    }

    bool availablePlannerSelected = false;
    for (unsigned i=0; i<availablePlanners.size(); i++)
        if(plannerType == availablePlanners[i]){
            availablePlannerSelected = true;
            break;
        }
    if (!availablePlannerSelected)
    {
        yWarning()<<"Selected NOT available planner. Using default planner: "<<DEFAULT_PLANNER;
        plannerType = DEFAULT_PLANNER;
    }


    if (plannerType == "RRTConnect"){
        auto plannerRRT = (new og::RRTConnect(si));
        plannerRRT->setRange(plannerRange);
        planner = ob::PlannerPtr(plannerRRT);
    }else if (plannerType == "RRTStar")
    {
        auto plannerRRT = (new og::RRTstar(si));
        plannerRRT->setRange(plannerRange);
        plannerRRT->setPruneThreshold(1.0);
        plannerRRT->setTreePruning(true);
        plannerRRT->setInformedSampling(true);
        planner = ob::PlannerPtr(plannerRRT);
    }

    
    if(!inPort.open(prefix + "/pointCloud:i"))
    {
        yError()<<"Could not open"<<inPort.getName()<<"open";
        return false;
    }

   
    rpcServer.setReader(*this);

    return true;
}

bool TrajectoryGeneration::openDevices(){
    
    // Lets try open the solver with the name on deviceName

    
    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robot+"/"+deviceName);
    options.put("local", "/" +robot + "/"+deviceName);
    device.open(options);
    if (!device.isValid())
    {
        yError() << "Robot "<<deviceName<<" device not available";
        device.close();
        yarp::os::Network::fini();
        return false;
    }

    // connecting our device with "IEncoders" interface
    if (!device.view(iEncoders))
    {
        yError() << "Problems acquiring IEncoders interface in "<<deviceName;
        return false;
    }
    else
    {
        yInfo() << "Acquired IEncoders interface in "<<deviceName;
        if (!iEncoders->getAxes(&numJoints))
            yError() << "Problems acquiring numJoints";
        else
            yWarning() << "Number of joints:" << numJoints;
    }

    if (!device.view(iControlLimits))
    {
        yError() << "Could not view iControlLimits in "<<deviceName;
        return false;
    }

 // connecting our device with "control mode" interface, initializing which control mode we want (position)
    if (!device.view(iControlMode))
    {
        yError() << "Problems acquiring IControlMode interface";
        return false;
    }
    else
        yInfo() << "Acquired IControlMode interface";

    // connecting our device with "PositionControl" interface
    if (!device.view(iPositionControl))
    {
        yError() << "Problems acquiring IPositionControl interface";
        return false;
    }
    else
        yInfo() << "Acquired IPositionControl interface";


    
    yInfo()<<"Devices open";
    return true;
}


bool TrajectoryGeneration::getCurrentQ(std::vector<double> & currentQ){

    if(!iEncoders->getEncoders(currentQ.data())){
        yError() << " Failed getEncoders() of "<<deviceName;
        return false;
    }
    return true;

}


/************************************************************************/
bool TrajectoryGeneration::isValid(const ob::State *state)
{

    if(planningSpace == "cartesian"){
        // cast the abstract state type to the type we expect
        const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);


        KDL::Frame frame;

        KDL::Rotation rotKdl = KDL::Rotation::Quaternion(rot->x, rot->y, rot->z, rot->w);
        KDL::Vector posKdl = KDL::Vector(pos->values[0], pos->values[1], pos->values[2]);
        double x, y, z, w;
        rotKdl.GetQuaternion(x, y, z, w);

        frame.M = rotKdl;
        frame.p = posKdl;
        std::vector<double> testAxisAngle = frameToVector(frame);

        std::vector<double> currentQ(numJoints);


        // Check if we are in the starting state

        std::vector<double> xStart;

        if (!armICartesianSolver->fwdKin(currentQ, xStart))
        {
            yError() << "fwdKin failed";
            return false;
        }


        bool computeInvKin = true;

        double diffSumSquare = 0;
        for (int i = 0; i < 6; i++)
        {
            diffSumSquare += (xStart[i] - testAxisAngle[i]) * (xStart[i] - testAxisAngle[i]);
        }


        if (diffSumSquare < DEFAULT_MAX_DIFF_INV)
        {
            computeInvKin = false;
        }

        std::vector<double> desireQ(numJoints);


        if (computeInvKin)
        {
            KDL::JntArray qInit = KDL::JntArray(numJoints);
            KDL::JntArray qInitRad = KDL::JntArray(numJoints);
            for (unsigned int i = 0; i < qInit.rows(); i++)
            {
                qInit(i) = currentQ[i];
                qInitRad(i) = currentQ[i]*KDL::deg2rad;

            }
            

            KDL::JntArray jointpositionsRad = KDL::JntArray(numJoints);
            KDL::JntArray jointpositions = KDL::JntArray(numJoints);


            int foundik = (*iksolver).CartToJnt(qInitRad, frame, jointpositionsRad, boundsSolver);
            if (foundik!=1)
            {
                yError() << "invKin() failed";
                return false;
            }

            // if (!armICartesianSolver->invKin(testAxisAngle, currentQ, desireQ))
            // {
            //     yError() << "invKin() failed";
            //     return false;
            // }
        
        
            goalQ.clear();

            for (unsigned int i = 0; i < jointpositionsRad.rows(); i++)
            {
                jointpositions(i) = jointpositionsRad(i)*KDL::rad2deg;
                desireQ[i] = jointpositions(i);
                goalQ.push_back(desireQ[i]);
            }
            
            yInfo()<<"desireQ: "<<desireQ;


            m_checkCollisions->updateCollisionObjectsTransform(desireQ);
            bool collide = m_checkCollisions->collision();
            return !collide;
        }
        return true;
    }
    else{ // joint space
        
        const ob::RealVectorStateSpace::StateType *jointState = state->as<ob::RealVectorStateSpace::StateType>();
        KDL::JntArray jointpositions = KDL::JntArray(numJoints);
        std::vector<double> currentQ(numJoints);

        for (unsigned int i = 0; i < jointpositions.rows(); i++)
        {
            jointpositions(i) = jointState->values[i];
            currentQ[i] = jointpositions(i);
            // yInfo()<<"Joint("<<i<<")="<<jointpositions(i);
        }
        

        jointsInsideBounds = m_checkCollisions->updateCollisionObjectsTransform(currentQ);

        
        if(jointsInsideBounds){
            yInfo()<<"Joints inside bounds";
            bool collide = m_checkCollisions->collision();
            yInfo()<<"eo: selfCollision"<<collide;
            return !collide;
        }
        else{
            
            yInfo()<<"eo: joints outside bounds";
            return false;
        }
    }
}

bool TrajectoryGeneration::computeDiscretePath(ob::ScopedState<ob::SE3StateSpace> start, ob::ScopedState<ob::SE3StateSpace> goal, std::vector<std::vector<double>> &jointsTrajectory, bool &validStartState, bool &validGoalState)
{

    // pdef->clearStartStates();
    // pdef->addStartState(start);

    auto plannerRRT = (new og::RRTstar(si));
    plannerRRT->setRange(0.05);
    plannerRRT->setPruneThreshold(0.1);
    plannerRRT->setTreePruning(true);
    plannerRRT->setInformedSampling(true);


    ob::State *startState = pdef->getStartState(0);

  
    // if (collide(startState))
    //     yInfo("Start state collides");
    // else
    // {
    //     yInfo("Start state doesn't collide");
    // }

    if (isValid(startState)){
        yInfo() << "Valid starting state";
        validStartState = true;
    }
    else
    {
        yInfo() << "Not valid starting state";
        validStartState = false;
        return false;
    }

    pdef->clearGoal();

    pdef->setGoalState(goal);

    ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();


    // if (collide(goalState))
    //     yInfo("Goal state collides");
    // else
    // {
    //     yInfo("Goal state doesn't collide");
    // }

    if (isValid(goalState)){
        yInfo() << "Valid goal state";
        validGoalState = true;
    }
    else
    {
        yInfo() << "Not valid goal state";
        validGoalState = false;
        return false;
    }

    planner->clear();
    planner->setProblemDefinition(pdef);

    planner->setup();

    bool solutionFound = planner->solve(maxPlannerTime);

    if (solutionFound == true)
    {
        yInfo() << "Sol";
        ob::PathPtr path = pdef->getSolutionPath();
        yInfo() << "Found discrete path from start to goal";
        path->print(std::cout);
        pth = path->as<og::PathGeometric>();
        yInfo() << pth->getStateCount();
    }


    std::size_t iState = 0;

    while (iState < pth->getStateCount())
    {
        ob::State *state = pth->getState(iState);

        const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        KDL::Frame frame;

        KDL::Rotation rotKdl = KDL::Rotation::Quaternion(rot->x, rot->y, rot->z, rot->w);
        KDL::Vector posKdl = KDL::Vector(pos->values[0], pos->values[1], pos->values[2]);
        //frame.M.Quaternion(rot->x, rot->y, rot->z, rot->w);
        double x, y, z, w;
        rotKdl.GetQuaternion(x, y, z, w);
        frame.M = rotKdl;
        frame.p = posKdl;
        std::vector<double> pose = frameToVector(frame);
        
        std::vector<double> currentQ(numJoints);

        if(!getCurrentQ(currentQ)){
            return false;
        }


        bool computeInvKin = true;
        std::vector<double>xStart;
        double diffSumSquare = 0;
        for (int i = 0; i < 6; i++)
        {
            diffSumSquare += (xStart[i] - pose[i]) * (xStart[i] - pose[i]);
        }

        yInfo() << "diffSumSquare: " << diffSumSquare;

        if (diffSumSquare < DEFAULT_MAX_DIFF_INV)
        {
            yInfo() << "Start state-> we do not calculate the inverse kinematics";
            computeInvKin = false;
        }

        std::vector<double> desireQ(numJoints);
        KDL::JntArray goaljointpositions = KDL::JntArray(numJoints);
        KDL::JntArray goaljointpositionsRad = KDL::JntArray(numJoints);


        if (computeInvKin)
        {

            KDL::JntArray qInit = KDL::JntArray(numJoints);
            KDL::JntArray qInitRad = KDL::JntArray(numJoints);
            for (unsigned int i = 0; i < qInit.rows(); i++)
            {
                qInit(i) = currentQ[i];
                qInitRad(i) = qInit(i)*KDL::deg2rad;
            }
            

            int foundik = (*iksolver).CartToJnt(qInitRad, frame, goaljointpositionsRad, boundsSolver);
            if (foundik!=1)
            {
                yError() << "invKin() failed";
                return false;
            }                                                                                                                                                                                               
            else{
                Bottle bPose;
                for(int i = 0; i < 6; i++){
                    bPose.addFloat64(pose[i]);
                }
                std::vector<double> jointsPosition;     
                jointsPosition.reserve(numJoints);
                for(int i = 0; i<numJoints; i++){
                    jointsPosition.emplace_back(goaljointpositionsRad(i)*KDL::rad2deg);
                }
                jointsTrajectory.push_back(jointsPosition);
            }


            // if (!armICartesianSolver->invKin(pose, currentQ, desireQ))
            // {
            //     yError() << "invKin() failed";
            // }
           
        }
        
        iState++;
    }

//#endif
    // followDiscretePath();

    return solutionFound;
}


bool TrajectoryGeneration::computeDiscretePath(ob::ScopedState<ob::RealVectorStateSpace> start, ob::ScopedState<ob::RealVectorStateSpace> goal, std::vector<std::vector<double>> &jointsTrajectory,
                                                std::string & errorMessage)
{

    pdef->clearStartStates();
    pdef->addStartState(start);

    start.print();

    ob::RealVectorBounds bounds = space->as<ob::RealVectorStateSpace>()->getBounds();
    for(unsigned int j=0; j<numJoints; j++){
        yInfo()<<"Low: "<<bounds.low[j]<<" high: "<<bounds.high[j];
    }



    ob::State *startState = pdef->getStartState(0);

  
   yInfo()<<"Is startState valid?";
    if (!isValid(startState)){
        yInfo()<<"Not valid start state";
        if (si->satisfiesBounds(startState))
            errorMessage = errorsTrajectoryGeneration::start_collision;
        else{
            errorMessage = errorsTrajectoryGeneration::joints_outside_bounds;
        }
        return false;
    }

    pdef->clearGoal();

    pdef->setGoalState(goal);

    ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();

   yInfo()<<"Is goalState valid?";

    if (!isValid(goalState)){
        yInfo()<<"Not valid goal state";
        if (si->satisfiesBounds(goalState))
            errorMessage = errorsTrajectoryGeneration::goal_collision;
        else{
            errorMessage = errorsTrajectoryGeneration::joints_outside_bounds;
        }
        return false;
    }
    yInfo()<<"goalState VALID";


    
    pdef->print();
    planner->clear();
    planner->printProperties(std::cout);
    planner->printSettings(std::cout);
    planner->setProblemDefinition(pdef);
    yInfo()<<"setProblemDefinition done";

    planner->setup();
    yInfo()<<"setup done";
    pdef->clearSolutionPaths();
    yInfo()<<"clearSolutionPaths done";


    yInfo()<<"solve";
    bool solutionFound = planner->solve(maxPlannerTime);
    printf("Solution Found: %d", solutionFound);

    if (solutionFound)
    {
        yInfo() << "Sol";
        ob::PathPtr path = pdef->getSolutionPath();
        yInfo() << "Found discrete path from start to goal";
        path->print(std::cout);
        pth = path->as<og::PathGeometric>();
        yInfo() << pth->getStateCount();

        std::size_t iState = 0;

        while (iState < pth->getStateCount())
        {
            ob::State *state = pth->getState(iState);

            const ob::RealVectorStateSpace::StateType *jointState = state->as<ob::RealVectorStateSpace::StateType>();
            
            std::vector<double> poseQ;
            poseQ.reserve(numJoints);
            for(unsigned int j=0; j<numJoints; j++)
                poseQ.emplace_back(jointState->values[j]);

            jointsTrajectory.push_back(poseQ);
            yInfo()<<poseQ;
            
            iState++;
        }
        return true;
    }else{
        errorMessage == errorsTrajectoryGeneration::path_not_found;
        return false;
    }


}

bool TrajectoryGeneration::checkGoalPose(yarp::os::Bottle * bGoal, std::vector<double> & desireQ, std::string & errorMessage){
    yInfo()<<"Check goal pose";
    std::vector<double> xGoal(6);
    if (bGoal->size() != 6){
        errorMessage = errorsTrajectoryGeneration::pose_6_elements;
        return false;
    }
    for (int i = 0; i < 6; i++)
        xGoal[i] = bGoal->get(i).asFloat64();
    yInfo() <<"Goal: "<< xGoal[0] << " " << xGoal[1] << " " << xGoal[2] << " " << xGoal[3] << " " << xGoal[4] << " " << xGoal[5];

    std::vector<double> currentQ(numJoints);

    if(!getCurrentQ(currentQ)){
        errorMessage = errorsTrajectoryGeneration::not_get_current_Q;
        return false;
    }
    else{
        // Lests add the start state, just for checking
        ob::ScopedState<ob::RealVectorStateSpace> start(space);
        for(unsigned int j=0; j<numJoints; j++){
            start[j] = currentQ[j];
        }

        pdef->clearStartStates();
        pdef->addStartState(start);

        start.print();


        ob::State *startState = pdef->getStartState(0);

        yInfo()<<"Check start state: ";
  
        if (!isValid(startState)){
            
            if (si->satisfiesBounds(startState))
                errorMessage = errorsTrajectoryGeneration::start_collision;
            else{
                errorMessage = errorsTrajectoryGeneration::joints_outside_bounds;
            }
            return false;
        }
        std::vector<double> xStart(6);

        if (!armICartesianSolver->fwdKin(currentQ, xStart))
        {
            yError() << "fwdKin failed";
            return false;
        }
        yInfo() <<"xStart: "<< xStart[0] << " " << xStart[1] << " " << xStart[2] << " " << xStart[3] << " " << xStart[4] << " " << xStart[5];

        yInfo()<<currentQ;

        KDL::JntArray qInit = KDL::JntArray(numJoints);
        KDL::JntArray qInitRad = KDL::JntArray(numJoints);

        for (unsigned int i = 0; i < qInit.rows(); i++)
        {
            qInit(i) = currentQ[i];
            qInitRad(i) = currentQ[i]*KDL::deg2rad;
        }
            
        KDL::JntArray goaljointpositionsRad = KDL::JntArray(numJoints);
        KDL::JntArray goaljointpositions = KDL::JntArray(numJoints);
        KDL::Frame frame = vectorToFrame(xGoal);

        // if (!armICartesianSolver->invKin(xGoal, currentQ, desireQ))
        // {
        //     printf("Not found invKin()");
        //     errorMessage = errorsTrajectoryGeneration::goal_not_inv_kin;
        //     return false;
        // }
        int foundik = (*iksolver).CartToJnt(qInitRad, frame, goaljointpositionsRad, boundsSolver);
        if (foundik!=1)
        {
            yError() << "invKin() failed";
            return false;
        } 
        else{   

            // Check if it's outside bounds
            for(int j=0; j<numJoints; j++)
            {
                goaljointpositions(j) = goaljointpositionsRad(j)*KDL::rad2deg;
                if(goaljointpositions(j)<=qmin(j))
                    goaljointpositions(j) = qmin(j)+0.001;
                if(goaljointpositions(j)>=qmax(j)){
                    goaljointpositions(j) = qmax(j)-0.001;
                }

                desireQ[j] = goaljointpositions(j);

            }
            yInfo()<<desireQ;

         
         
            ob::ScopedState<ob::RealVectorStateSpace> goal(space);
            for(unsigned int j=0; j<numJoints; j++){
                goal[j] = goaljointpositions(j);
            }
            pdef->clearGoal();

            pdef->setGoalState(goal);

            ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();
            if(isValid(goalState)){
                return true;
            }
            else{
                if (si->satisfiesBounds(goalState))
                    errorMessage = errorsTrajectoryGeneration::goal_collision;
                else{
                    errorMessage = errorsTrajectoryGeneration::joints_outside_bounds;
                }
                return false;
            }
        }    
    }
}

bool TrajectoryGeneration::checkGoalJoints(yarp::os::Bottle * bGoal, std::string & errorMessage){
    if (bGoal->size() != numJoints){
        yWarning()<<errorsTrajectoryGeneration::joints_elements;
        errorMessage = errorsTrajectoryGeneration::joints_elements;
        return false;
    }
    else{
        std::vector<double> goalQ(numJoints);
        std::vector<double> xGoal(6);

        for (int i = 0; i < numJoints; i++)
            goalQ[i] = bGoal->get(i).asFloat64();

        if (!armICartesianSolver->fwdKin(goalQ, xGoal))
        {
            yError() << "fwdKin failed";
            return false;
        }
        yInfo() <<"Goal: "<< xGoal[0] << " " << xGoal[1] << " " << xGoal[2] << " " << xGoal[3] << " " << xGoal[4] << " " << xGoal[5];

        
        ob::ScopedState<ob::RealVectorStateSpace> goal(space);
        for(unsigned int j=0; j<numJoints; j++){
            goal[j] = bGoal->get(j).asFloat64();
        }
        pdef->clearGoal();

        pdef->setGoalState(goal);

        ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();
        if(isValid(goalState)){
            return true;
        }
        else{
            if (si->satisfiesBounds(goalState))
                errorMessage = errorsTrajectoryGeneration::goal_collision;
            else{
                errorMessage = errorsTrajectoryGeneration::joints_outside_bounds;
            }
            return false;
        }
    }
}

/************************************************************************/
void TrajectoryGeneration::getSuperquadrics(std::vector<int> &label_idx, std::vector<std::array<float,11>> &params){
    yInfo()<<"Lets get the superquadrics";
    yarp::os::Bottle cmd;
    cmd.addString("gsup");
    yInfo()<<"Sending message..."<<cmd.toString();
    yarp::os::Bottle response;
    m_clientGetGraspingPoses.write(cmd, response);
    yInfo()<<"Got response:"<<response.toString();


    for(int i=0; i<response.size(); i++){
        label_idx.push_back(response.get(i).find("label_idx").asInt32());
        std::array<float,11> object_params;
        object_params[0] = response.get(i).find("axes0").asFloat32();
        object_params[1] = response.get(i).find("axes1").asFloat32();
        object_params[2] = response.get(i).find("axes2").asFloat32();
        object_params[3] = response.get(i).find("e1").asFloat32();
        object_params[4] = response.get(i).find("e2").asFloat32();
        object_params[5] = response.get(i).find("x").asFloat32();
        object_params[6] = response.get(i).find("y").asFloat32();
        object_params[7] = response.get(i).find("z").asFloat32();
        object_params[8] = response.get(i).find("roll").asFloat32();
        object_params[9] = response.get(i).find("pitch").asFloat32();
        object_params[10] = response.get(i).find("yaw").asFloat32();
        params.push_back(object_params);

        yInfo()<<label_idx[i];
        yInfo()<<object_params;
    }
}


bool TrajectoryGeneration::read(yarp::os::ConnectionReader &connection)
{
    yarp::os::Bottle reply;
    auto *writer = connection.getWriter();


    yarp::os::Bottle command;
    if (!command.read(connection) || writer == nullptr)
    {
        return false;
    }
    yInfo() << "command:" << command.toString();



    std::vector<double> currentQ(numJoints);

     
    if (planningSpace=="cartesian"){ //TODO: Cartesian
        std::vector<double> xStart;
        if(!getCurrentQ(currentQ)){
            return false;
        }

        if (!armICartesianSolver->fwdKin(currentQ, xStart))
        {
            yError() << "fwdKin failed";
            return false;
        }
        
        yInfo() << "Start:" << xStart[0] << " " << xStart[1] << " " << xStart[2] << " " << xStart[3] << " " << xStart[4] << " " << xStart[5];
        armSolverOptions.unput("mins");
        armSolverOptions.unput("maxs");
        yarp::os::Bottle qMin, qMax;
        qMin.addFloat64(currentQ[0]-2.5);
        qMin.addFloat64(currentQ[1]-2.5);
        qMax.addFloat64(currentQ[0]+0.5);
        qMax.addFloat64(currentQ[1]+0.5);
        for(int i=0; i<currentQ.size(); i++){
            qMin.addFloat64(qrMin.get(i).asFloat64());
            qMax.addFloat64(qrMax.get(i).asFloat64());
        }
        armSolverOptions.put("mins", yarp::os::Value::makeList(qMin.toString().c_str()));
        armSolverOptions.put("maxs", yarp::os::Value::makeList(qMax.toString().c_str()));
        armSolverDevice.close();

        armSolverDevice.open(armSolverOptions);
        if (!armSolverDevice.isValid())
        {
            yError() << "KDLSolver solver device for "<<deviceName<<" is not valid";
            return false;
        }


        if (!armSolverDevice.view(armICartesianSolver))
        {
            yError() << "Could not view iCartesianSolver in KDLSolver";
            return false;
        }

        if (!armICartesianSolver->fwdKin(currentQ, xStart))
        {
            yError() << "fwdKin failed";

            reply.addString("fwdKin failed for the start state");
            return reply.write(*writer);
        }
        yInfo() << "Start:" << xStart[0] << " " << xStart[1] << " " << xStart[2] << " " << xStart[3] << " " << xStart[4] << " " << xStart[5];

        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

        KDL::Frame frame = vectorToFrame(xStart);
        double qx, qy, qz, qw;
        frame.M.GetQuaternion(qx, qy, qz, qw);
        ob::ScopedState<ob::SE3StateSpace> start(space);

        start->setXYZ(xStart[0], xStart[1], xStart[2]);
        start->rotation().x = qx;
        start->rotation().y = qy;
        start->rotation().z = qz;
        start->rotation().w = qw;
        pdef->clearStartStates();
        pdef->addStartState(start);

        // if (command.get(0).toString() == "Check goal"){
        //     yInfo()<<"Check goal";
        //     Bottle * bGoal = command.get(1).asList();
        //     std::vector<double> xGoal(6);
        //     for (int i = 0; i < 6; i++)
        //         xGoal[i] = bGoal->get(i).asDouble();
        //     yInfo() <<"Goal: "<< xGoal[0] << " " << xGoal[1] << " " << xGoal[2] << " " << xGoal[3] << " " << xGoal[4] << " " << xGoal[5];

        //     frame = vectorToFrame(xGoal);
        //     frame.M.GetQuaternion(qx, qy, qz, qw);
        //     ob::ScopedState<ob::SE3StateSpace> goal(space);

        //     goal->setXYZ(xGoal[0], xGoal[1], xGoal[2]);
        //     goal->rotation().x = qx;
        //     goal->rotation().y = qy;
        //     goal->rotation().z = qz;
        //     goal->rotation().w = qw;

        //     pdef->clearGoal();

        //     pdef->setGoalState(goal);

        //     ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();
        //     if(isValid(goalState)){
        //         yInfo() << "Valid";
        //         reply.addString("Valid");
        //         Bottle bJointsPosition;

        //         for(int j = 0; j<numJoints; j++){
        //             yInfo()<<goalQ[j];
        //             bJointsPosition.addDouble(goalQ[j]);
        //         }
        //         reply.addList() = bJointsPosition;
        //     }
        //     else{
        //         yInfo() << "Not Valid";
        //         reply.addString("Not Valid");
        //     }
        // }

        // else if (command.get(0).toString() == "Compute trajectory"){
        //     Bottle * bGoal = command.get(1).asList();
        //     std::vector<double> xGoal(6);
        //     for (int i = 0; i < 6; i++)
        //         xGoal[i] = bGoal->get(i).asDouble();
        //     frame = vectorToFrame(xGoal);
        //     frame.M.GetQuaternion(qx, qy, qz, qw);
        //     ob::ScopedState<ob::SE3StateSpace> goal(space);

        //     goal->setXYZ(xGoal[0], xGoal[1], xGoal[2]);
        //     goal->rotation().x = qx;
        //     goal->rotation().y = qy;
        //     goal->rotation().z = qz;
        //     goal->rotation().w = qw;

        //     pdef->clearGoal();

        //     pdef->setGoalState(goal);
        //     std::vector<std::vector<double>>jointsTrajectory;
        //     bool validStartState, validGoalState;
        //     bool solutionFound = computeDiscretePath(start, goal, jointsTrajectory, validStartState, validGoalState);

        //     if (solutionFound)
        //     {
        //         // followDiscretePath();
        //         yInfo() << "Solution Found";
        //         reply.addString("Solution Found");
        //         Bottle bJointsTrajectory;
        //         for(int i=0; i<jointsTrajectory.size(); i++){
        //             Bottle bJointsPosition;
        //             for(int j = 0; j<numJoints; j++){
        //                 bJointsPosition.addDouble(jointsTrajectory[i][j]);
        //             }
        //             bJointsTrajectory.addList() = bJointsPosition;
        //         }
        //         reply.addList() =bJointsTrajectory;
        //     }
        //     else{
        //         if(!validStartState)
        //         {
        //             yInfo() <<"Start state NOT valid";
        //             reply.addString("Start state NOT valid");
        //         }
        //         if(!validGoalState){
        //             yInfo() <<"Goal state NOT valid";
        //             reply.addString("Goal state NOT valid"); 
        //         }
        //         if(validStartState && validGoalState){
        //             yInfo() << "Solution NOT found";
        //             reply.addString("Solution NOT found");
        //         }
        //     }
        // }
    }
    else{ // joint space
        yInfo("Joint space");
        unsigned int nj = numJoints;
        space = ob::StateSpacePtr(new ob::RealVectorStateSpace(nj));
        ob::RealVectorBounds bounds{nj};
        for(unsigned int j=0; j<nj; j++){
            bounds.setLow(j, qmin(j));
            bounds.setHigh(j, qmax(j));
        }
        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        si->setStateValidityChecker(std::bind(&TrajectoryGeneration::isValid, this, std::placeholders::_1));
        si->setup();


        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

        ob::ScopedState<ob::RealVectorStateSpace> start(space);

        if(!getCurrentQ(currentQ)){
            return false;
        }
        for(unsigned int j=0; j<numJoints; j++){
            start[j] = currentQ[j];
        }

        
        switch(command.get(0).asVocab32()){

            case VOCAB_HELP:{
                yInfo() <<"help";
                static auto usage = makeUsage();
                reply.append(usage);
            }break;
            case VOCAB_CMD_GET_SUPERQUADRICS:{
                std::vector<int> label_idx; 
                std::vector<std::array<float,11>> params;
                getSuperquadrics(label_idx, params);
                m_checkCollisions->setSuperquadrics(label_idx, params);
                m_checkCollisions->updateEnvironmentCollisionObjects();

            }break;
            case VOCAB_UPDATE_POINTCLOUD:{
                yInfo() << "Update the pointcloud for collision checking";
                if(!yarp::os::Network::connect("/rgbdObjectDetection/state:o", inPort.getName())){
                    yError() << "Unable to connect /getGraspingPoses/xtion/pointCloud:o to "<<inPort.getName();
                    reply.addString("Unable to connect to /pointcloud port");
                }
                inPort.read(inCloud);
                yInfo()<<inCloud.size();
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
                if(yarp::pcl::toPCL<yarp::sig::DataXYZRGBA, pcl::PointXYZRGBA>(inCloud, *cloud)){
                    yInfo()<<"Could store the yarp pointcloud in a pcl cloud";
                }


            }break;
            case VOCAB_CHECK_GOAL_POSE:
                {yInfo() << "Check goal pose (x, y, z, rotx, roty, rotz)";
                std::vector<double> desireQ(numJoints);
                std::string errorMessage;
                if(!checkGoalPose(command.get(1).asList(), desireQ, errorMessage)){
                    reply.addVocab32(VOCAB_FAILED);
                    reply.addString(errorMessage);
                }
                else{
                    yInfo() << "Valid goal pose";
                    reply.addVocab32(VOCAB_OK);
                    Bottle bJointsPosition;
                    for(int j = 0; j<numJoints; j++){
                        bJointsPosition.addFloat64(desireQ[j]);
                    }
                    reply.addList() = bJointsPosition;
                }
                }break;
            case VOCAB_CHECK_GOAL_JOINTS:
                {
                yInfo() << "Check goal joints (j0, j1, ..., jn)";
                std::string errorMessage;
                if(!checkGoalJoints(command.get(1).asList(), errorMessage)){
                    reply.addVocab32(VOCAB_FAILED);
                    reply.addString(errorMessage);
                }
                else{
                    reply.addVocab32(VOCAB_OK);
                }
                }break;
            case VOCAB_COMPUTE_JOINTS_PATH_GOAL_POSE:
                {
                    yInfo()<<"Compute path in joint space to goal pose (x, y, z, rotx, roty, rotz";
                    std::vector<double> desireQ(numJoints);
                    std::string errorMessage;
                    if(!checkGoalPose(command.get(1).asList(), desireQ, errorMessage)){
                        reply.addVocab32(VOCAB_FAILED);
                        reply.addString(errorMessage);
                    }
                    else{
                        ob::ScopedState<ob::RealVectorStateSpace> goal(space);
                        for(unsigned int j=0; j<numJoints; j++){
                            goal[j] = desireQ[j];
                        }
                        pdef->clearGoal();
                        pdef->setGoalState(goal);

                        std::vector<std::vector<double>>jointsTrajectory;
                        std::string errorMessage;
                        yInfo()<<"Lets compute the path";
                        bool solutionFound = computeDiscretePath(start, goal, jointsTrajectory, errorMessage);

                        if (solutionFound){
                            reply.addVocab32(VOCAB_OK);
                            Bottle bJointsTrajectory;
                            for(int i=0; i<jointsTrajectory.size(); i++){
                                Bottle bJointsPosition;
                                for(int j = 0; j<numJoints; j++){
                                    bJointsPosition.addFloat64(jointsTrajectory[i][j]);
                                }
                                bJointsTrajectory.addList() = bJointsPosition;
                            }
                            reply.addList() =bJointsTrajectory;
                        }
                        else{
                            reply.addVocab32(VOCAB_FAILED);
                            reply.addString(errorMessage);
                        }
                    }
                }break;
            case VOCAB_COMPUTE_JOINTS_PATH_GOAL_JOINTS:
                {yInfo()<<"Compute path in joint space to goal joints configuration (j0, j1, ..., jn)";
                Bottle * bGoal = command.get(1).asList();
                std::string errorMessage;
                if (bGoal->size() != numJoints){
                    reply.addVocab32(VOCAB_FAILED);
                    reply.addString(errorsTrajectoryGeneration::joints_elements);
                }
                else{
                    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
                    for(unsigned int j=0; j<numJoints; j++){
                        goal[j] = bGoal->get(j).asFloat64();
                    }
                    pdef->clearGoal();
                    pdef->setGoalState(goal);

                    ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();

                    if (!si->satisfiesBounds(goalState)){
                        errorMessage = errorsTrajectoryGeneration::joints_outside_bounds;
                        return false;
                    }
                    else{
                        std::vector<std::vector<double>>jointsTrajectory;
                        bool solutionFound = computeDiscretePath(start, goal, jointsTrajectory, errorMessage);
                        if (solutionFound){
                            reply.addVocab32(VOCAB_OK);
                            Bottle bJointsTrajectory;
                            for(int i=0; i<jointsTrajectory.size(); i++){
                                Bottle bJointsPosition;
                                for(int j = 0; j<numJoints; j++){
                                    bJointsPosition.addFloat64(jointsTrajectory[i][j]);
                                }
                                bJointsTrajectory.addList() = bJointsPosition;
                            }
                            reply.addList() =bJointsTrajectory;
                        }
                        else{
                            reply.addVocab32(VOCAB_FAILED);
                            reply.addString(errorMessage);
                        }
                    }
                }
                }break;

            }      
    }
    return reply.write(*writer);
}