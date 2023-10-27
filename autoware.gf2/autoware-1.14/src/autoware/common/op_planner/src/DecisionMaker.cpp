
/// \file DecisionMaker.cpp
/// \brief Initialize behaviors state machine, and calculate required parameters for the state machine transition conditions
/// \author Hatem Darweesh
/// \date Dec 14, 2016


#include "op_planner/DecisionMaker.h"
#include "op_utility/UtilityH.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"
#include <ros/ros.h>


namespace PlannerHNS
{

DecisionMaker::DecisionMaker()
{
  m_iCurrentTotalPathId = 0;
  pLane = 0;
  m_pCurrentBehaviorState = 0;
  m_pGoToGoalState = 0;
  m_pStopState= 0;
  m_pWaitState= 0;
  m_pMissionCompleteState= 0;
  m_pAvoidObstacleState = 0;
  m_pTrafficLightStopState = 0;
  m_pTrafficLightWaitState = 0;
  m_pStopSignStopState = 0;
  m_pStopSignWaitState = 0;
  m_pFollowState = 0;
  m_MaxLaneSearchDistance = 3.0;
  m_pStopState = 0;
  m_pMissionCompleteState = 0;
  m_pGoalState = 0;
  m_pGoToGoalState = 0;
  m_pWaitState = 0;
  m_pInitState = 0;
  m_pFollowState = 0;
  m_pAvoidObstacleState = 0;
  scenario_ended = false;
}

DecisionMaker::~DecisionMaker()
{
  delete m_pStopState;
  delete m_pMissionCompleteState;
  delete m_pGoalState;
  delete m_pGoToGoalState;
  delete m_pWaitState;
  delete m_pInitState;
  delete m_pFollowState;
  delete m_pAvoidObstacleState;
  delete m_pTrafficLightStopState;
  delete m_pTrafficLightWaitState;
  delete m_pStopSignWaitState;
  delete m_pStopSignStopState;
}

void DecisionMaker::Init(const ControllerParams& ctrlParams, const PlannerHNS::PlanningParams& params,const CAR_BASIC_INFO& carInfo)
{
    m_CarInfo = carInfo;
    m_ControlParams = ctrlParams;
    m_params = params;
    pre_locate_index_ = 0;

    m_pidVelocity.Init(0.01, 0.004, 0.01);
    m_pidVelocity.Setlimit(m_params.maxSpeed, 0);

    m_pidStopping.Init(0.05, 0.05, 0.1);
    m_pidStopping.Setlimit(m_params.horizonDistance, 0);

    m_pidFollowing.Init(0.05, 0.05, 0.01);
    m_pidFollowing.Setlimit(m_params.minFollowingDistance, 0);

    InitBehaviorStates();

    if(m_pCurrentBehaviorState)
      m_pCurrentBehaviorState->SetBehaviorsParams(&m_params);
    
    mode_order = 0;
}

void DecisionMaker::InitBehaviorStates()
{

  m_pStopState         = new StopState(&m_params, 0, 0);
  m_pMissionCompleteState   = new MissionAccomplishedStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), 0);
  m_pGoalState        = new GoalStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pMissionCompleteState);
  m_pGoToGoalState       = new ForwardStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoalState);
  m_pInitState         = new InitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);

  m_pFollowState        = new FollowStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
  m_pAvoidObstacleState    = new SwerveStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
  m_pStopSignWaitState    = new StopSignWaitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
  m_pStopSignStopState    = new StopSignStopStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pStopSignWaitState);

  m_pTrafficLightWaitState  = new TrafficLightWaitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
  m_pTrafficLightStopState  = new TrafficLightStopStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);

  m_pWaitState = new WaitState(m_pStopState->m_pParams,m_pStopState->GetCalcParams(),m_pGoToGoalState);


  m_pGoToGoalState->InsertNextState(m_pAvoidObstacleState);
  m_pGoToGoalState->InsertNextState(m_pStopSignStopState);
  m_pGoToGoalState->InsertNextState(m_pTrafficLightStopState);
  m_pGoToGoalState->InsertNextState(m_pFollowState);
  m_pGoToGoalState->InsertNextState(m_pWaitState);

  m_pGoToGoalState->decisionMakingCount = 0;//m_params.nReliableCount;

  m_pGoalState->InsertNextState(m_pGoToGoalState);

  m_pStopSignWaitState->decisionMakingTime = m_params.stopSignStopTime;
  m_pStopSignWaitState->InsertNextState(m_pStopSignStopState);
  m_pStopSignWaitState->InsertNextState(m_pGoalState);


  m_pTrafficLightStopState->InsertNextState(m_pTrafficLightWaitState);

  m_pTrafficLightWaitState->InsertNextState(m_pTrafficLightStopState);
  m_pTrafficLightWaitState->InsertNextState(m_pGoalState);

  m_pFollowState->InsertNextState(m_pAvoidObstacleState);
  m_pFollowState->InsertNextState(m_pStopSignStopState);
  m_pFollowState->InsertNextState(m_pTrafficLightStopState);
  m_pFollowState->InsertNextState(m_pGoalState);
  m_pFollowState->InsertNextState(m_pWaitState);
  m_pFollowState->decisionMakingCount = 0;//m_params.nReliableCount;

  m_pInitState->decisionMakingCount = 0;//m_params.nReliableCount;

  m_pCurrentBehaviorState = m_pInitState;
}

 bool DecisionMaker::GetNextTrafficLight(const int& prevTrafficLightId, const std::vector<PlannerHNS::TrafficLight>& trafficLights, PlannerHNS::TrafficLight& trafficL)
 {
   for(unsigned int i = 0; i < trafficLights.size(); i++)
   {
     double d = hypot(trafficLights.at(i).pos.y - state.pos.y, trafficLights.at(i).pos.x - state.pos.x);
     if(d <= trafficLights.at(i).stoppingDistance)
     {
       double a_diff = UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(UtilityHNS::UtilityH::FixNegativeAngle(trafficLights.at(i).pos.a) , UtilityHNS::UtilityH::FixNegativeAngle(state.pos.a));

       if(a_diff < M_PI_2 && trafficLights.at(i).id != prevTrafficLightId)
       {
         //std::cout << "Detected Light, ID = " << trafficLights.at(i).id << ", Distance = " << d << ", Angle = " << trafficLights.at(i).pos.a*RAD2DEG << ", Car Heading = " << state.pos.a*RAD2DEG << ", Diff = " << a_diff*RAD2DEG << std::endl;
         trafficL = trafficLights.at(i);
         return true;
       }
     }
   }

   return false;
 }

 void DecisionMaker::CalculateImportantParameterForDecisionMaking(const PlannerHNS::VehicleState& car_state,
     const int& goalID, const bool& bEmergencyStop, const std::vector<TrafficLight>& detectedLights,
     const TrajectoryCost& bestTrajectory)
 {
   if(m_TotalPath.size() == 0) return;

   // 这里其实是给这些必要的计算参数赋值，以方便根据这些参数得到当前状态
   PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();

   // linkx add 2022_09_07 
   pValues->iPrevSafeLane = pValues->iCurrSafeLane;

   
   if(m_CarInfo.max_deceleration != 0)
     pValues->minStoppingDistance = std::min(-pow(car_state.speed, 2)/(m_CarInfo.max_deceleration),5.0);
  
   pValues->iCentralTrajectory    = m_pCurrentBehaviorState->m_pParams->rollOutNumber/2;
	
   if(pValues->iPrevSafeTrajectory < 0)
	   pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;


   pValues->stoppingDistances.clear();
   pValues->currentVelocity     = car_state.speed;
   pValues->bTrafficIsRed       = false;
   pValues->currentTrafficLightID   = -1;
   pValues->bFullyBlock       = false;

   pValues->distanceToNext = bestTrajectory.closest_obj_distance;
   pValues->velocityOfNext = bestTrajectory.closest_obj_velocity;

   if(bestTrajectory.index >=0 &&  bestTrajectory.index < (int)m_RollOuts.size())
     pValues->iCurrSafeTrajectory = bestTrajectory.index;
   else
     pValues->iCurrSafeTrajectory = pValues->iCentralTrajectory;

   pValues->bFullyBlock = bestTrajectory.bBlocked;
// pValues->iCurrSafeLane 均为-1, lane_index 就是当前最优局部路径所在的全局轨迹的编号
   if(bestTrajectory.lane_index >=0)
      pValues->iCurrSafeLane = bestTrajectory.lane_index;
   else
   {
     PlannerHNS::RelativeInfo info;
     PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_TotalPath, state, m_params.rollOutDensity*m_params.rollOutNumber/2.0 + 0.1, info);
     pValues->iCurrSafeLane = info.iGlobalPath;
   }
   

   double critical_long_front_distance =  m_CarInfo.wheel_base/2.0 + m_CarInfo.length/2.0 + m_params.verticalSafetyDistance;

  if(judgeLocalization()){
    pValues->localizationFlag = true;
    if(ReachEndOfGlobalPath(pValues->minStoppingDistance + critical_long_front_distance, pValues->iCurrSafeLane)){ 
      pValues->currentGoalID = -1;
      scenario_ended = true;
      ROS_ERROR("get the end point!");
      }
    else
      pValues->currentGoalID = goalID;
  }
  else {
      ROS_ERROR("localization error!");
      pValues->localizationFlag = false;
  }


   m_iCurrentTotalPathId = pValues->iCurrSafeLane;

   int stopLineID = -1;
   int stopSignID = -1;
   int trafficLightID = -1;
   double distanceToClosestStopLine = 0;
   bool bGreenTrafficLight = true;

    // 这里就会用到全局轨迹的每个点的pLane，所以在目前贴边轨迹中无法使用
    distanceToClosestStopLine = PlanningHelpers::GetDistanceToClosestStopLineAndCheck(m_TotalPath.at(pValues->iCurrSafeLane), state, m_params.giveUpDistance, stopLineID, stopSignID, trafficLightID) - critical_long_front_distance;

    //std::cout << "StopLineID" << stopLineID << ", StopSignID: " << stopSignID << ", TrafficLightID: " << trafficLightID << ", Distance: " << distanceToClosestStopLine << ", MinStopDistance: " << pValues->minStoppingDistance << std::endl;

   if(distanceToClosestStopLine > m_params.giveUpDistance && distanceToClosestStopLine < (pValues->minStoppingDistance + 1.0))
   {
     if(m_pCurrentBehaviorState->m_pParams->enableTrafficLightBehavior)
     {
       pValues->currentTrafficLightID = trafficLightID;
       //std::cout << "Detected Traffic Light: " << trafficLightID << std::endl;
       for(unsigned int i=0; i< detectedLights.size(); i++)
       {
         if(detectedLights.at(i).id == trafficLightID)
           bGreenTrafficLight = (detectedLights.at(i).lightState == GREEN_LIGHT);
       }
     }

     if(m_pCurrentBehaviorState->m_pParams->enableStopSignBehavior)
       pValues->currentStopSignID = stopSignID;

    pValues->stoppingDistances.push_back(distanceToClosestStopLine);
    //std::cout << "LP => D: " << pValues->distanceToStop() << ", PrevSignID: " << pValues->prevTrafficLightID << ", CurrSignID: " << pValues->currentTrafficLightID << ", Green: " << bGreenTrafficLight << endl;
   }


   //std::cout << "Distance To Closest: " << distanceToClosestStopLine << ", Stop LineID: " << stopLineID << ", Stop SignID: " << stopSignID << ", TFID: " << trafficLightID << std::endl;

   pValues->bTrafficIsRed = !bGreenTrafficLight;

   if(bEmergencyStop)
  {
    pValues->bFullyBlock = true;
    pValues->distanceToNext = 1;
    pValues->velocityOfNext = 0;
  }
  // std::cout<<"-----------------"<<std::endl;
   //cout << "Distances: " << pValues->stoppingDistances.size() << ", Distance To Stop : " << pValues->distanceToStop << endl;
 }

 void DecisionMaker::UpdateCurrentLane(const double& search_distance)
 {
   PlannerHNS::Lane* pMapLane = 0;
  PlannerHNS::Lane* pPathLane = 0;
  // std::cout<<"m_TotalPath.at(m_iCurrentTotalPathId): "<<m_iCurrentTotalPathId<<std::endl;
  pPathLane = MappingHelpers::GetLaneFromPath(state, m_TotalPath.at(m_iCurrentTotalPathId));
  if(!pPathLane)
  {
    std::cout << "Performance Alert: Can't Find Lane Information in Global Path, Searching the Map :( " << std::endl;
    pMapLane  = MappingHelpers::GetClosestLaneFromMap(state, m_Map, search_distance);
  }

  if(pPathLane)
    pLane = pPathLane;
  else if(pMapLane)
    pLane = pMapLane;
  else
    pLane = 0;
 }

 bool DecisionMaker::ReachEndOfGlobalPath(const double& min_distance, const int& iGlobalPathIndex)
 {
   if(m_TotalPath.size()==0) return false;

   PlannerHNS::RelativeInfo info;
   PlanningHelpers::GetRelativeInfo(m_TotalPath.at(iGlobalPathIndex), state, info);
  //  std::cout<<"pre_index: "<<pre_locate_index_<<"min_distance: "<<min_distance<<", closest index: "<<info.iFront<<", global size: "<<m_TotalPath.at(iGlobalPathIndex).size()<<std::endl;
   if (fabs(info.iFront - pre_locate_index_)>5/m_params.pathDensity)
   {
       info.iFront = pre_locate_index_;
   }
   pre_locate_index_ = info.iFront;

  //  PlanningHelpers::GetRelativeInfo(m_TotalOriginalPath.at(iGlobalPathIndex),state,info);

   double d = 0;
   for(unsigned int i = info.iFront; i < m_TotalPath.at(iGlobalPathIndex).size()-1; i++)
   {
     d+= hypot(m_TotalPath.at(iGlobalPathIndex).at(i+1).pos.y - m_TotalPath.at(iGlobalPathIndex).at(i).pos.y, m_TotalPath.at(iGlobalPathIndex).at(i+1).pos.x - m_TotalPath.at(iGlobalPathIndex).at(i).pos.x);
     if(d > min_distance)
       return false;
   }
   return true;
 }

 bool DecisionMaker::judgeLocalization(){
  // if(pre_state.pos.x == 0 && pre_state.pos.y == 0){
  //   pre_state = state;
  //   return true;
  // }
  // if(distance2points(state.pos,pre_state.pos) > 2) return false;
  // pre_state = state;
  // return true;
  
  if(fitness_score <= pose_match_threshold){
    // std::cout<<"fitness score: "<<fitness_score<<std::endl;
    return true;
  }
  return false;

 }

 void DecisionMaker::SetNewGlobalPath(const std::vector<std::vector<WayPoint> >& globalPath)
 {
   if(m_pCurrentBehaviorState)
   {
     m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath = true;
     m_TotalOriginalPath = globalPath;
   }
 }

 bool DecisionMaker::SelectSafeTrajectory()
 {
   bool bNewTrajectory = false;
   PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

   if(!preCalcPrams || m_RollOuts.size() == 0) return bNewTrajectory;

  int currIndex = PlannerHNS::PlanningHelpers::GetClosestNextPointIndexFast(m_Path, state);
  int index_limit = 0;
  if(index_limit<=0)
     index_limit =  m_Path.size()/4.0; //  m_Path.size()/2.0 必须走完原来局部路径一半才能换新的全局轨迹。
  if(currIndex > index_limit
      || preCalcPrams->bRePlan
      || preCalcPrams->bNewGlobalPath || mode_order == 2)  
  {
    std::cout << "New Local Plan !! " << currIndex << ", "<< preCalcPrams->bRePlan << ", " << preCalcPrams->bNewGlobalPath  << ", " <<  m_TotalOriginalPath.at(0).size() << ", PrevLocal: " << m_Path.size()<< "mode order: "<<mode_order;
    m_Path = m_RollOuts.at(preCalcPrams->iCurrSafeTrajectory);
    std::cout << ", NewLocal: " << m_Path.size() << std::endl;

    preCalcPrams->bNewGlobalPath = false;
    preCalcPrams->bRePlan = false;
    bNewTrajectory = true;
  }

  return bNewTrajectory;
 }

 PlannerHNS::BehaviorState DecisionMaker::GenerateBehaviorState(const PlannerHNS::VehicleState& vehicleState)
 {
  // 取出前面计算的必要参数
  PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();
//在这里进入下一状态
  m_pCurrentBehaviorState = m_pCurrentBehaviorState->GetNextState();
  if(m_pCurrentBehaviorState==0)
    m_pCurrentBehaviorState = m_pInitState;

  PlannerHNS::BehaviorState currentBehavior;

  currentBehavior.state = m_pCurrentBehaviorState->m_Behavior;
  currentBehavior.followDistance = preCalcPrams->distanceToNext;

  currentBehavior.minVelocity    = 0;
  currentBehavior.stopDistance   = preCalcPrams->distanceToStop();
  currentBehavior.followVelocity   = preCalcPrams->velocityOfNext;
  // std::cout<<"preCalcPrams->iPrevSafeTrajectory: "<<preCalcPrams->iPrevSafeTrajectory<<std::endl;
  if(preCalcPrams->iPrevSafeTrajectory<0 || preCalcPrams->iPrevSafeTrajectory >= m_RollOuts.size())
    currentBehavior.iTrajectory    = preCalcPrams->iCurrSafeTrajectory;
  else
    currentBehavior.iTrajectory    = preCalcPrams->iPrevSafeTrajectory;

  double average_braking_distance = -pow(vehicleState.speed, 2)/(m_CarInfo.max_deceleration) + m_params.additionalBrakingDistance;

  if(average_braking_distance  < m_params.minIndicationDistance)
    average_braking_distance = m_params.minIndicationDistance;
// 判断左转还是右转，m_path 是此刻的best 局部路径
  currentBehavior.indicator = PlanningHelpers::GetIndicatorsFromPath(m_Path, state, average_braking_distance );

  return currentBehavior;
 }

 double DecisionMaker::UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const VehicleState& CurrStatus, const double& dt)
 {
  if(m_TotalOriginalPath.size() ==0 ) return 0;

  RelativeInfo info, total_info;
  PlanningHelpers::GetRelativeInfo(m_TotalOriginalPath.at(m_iCurrentTotalPathId), state, total_info);
  PlanningHelpers::GetRelativeInfo(m_Path, state, info);
  double average_braking_distance = -pow(CurrStatus.speed, 2)/(m_CarInfo.max_deceleration) + m_params.additionalBrakingDistance;
  // double max_velocity  = PlannerHNS::PlanningHelpers::GetVelocityAhead(m_TotalOriginalPath.at(m_iCurrentTotalPathId), total_info, total_info.iBack, average_braking_distance);
  double max_velocity = m_params.maxSpeed;

  unsigned int point_index = 0;
  double critical_long_front_distance = m_CarInfo.length/2.0;

  if(beh.state == TRAFFIC_LIGHT_STOP_STATE || beh.state == STOP_SIGN_STOP_STATE)
  {
    PlanningHelpers::GetFollowPointOnTrajectory(m_Path, info, beh.stopDistance - critical_long_front_distance, point_index);

    double e = -beh.stopDistance;
    double desiredVelocity = m_pidStopping.getPID(e);

//    std::cout << "Stopping : e=" << e << ", desiredPID=" << desiredVelocity << ", PID: " << m_pidStopping.ToString() << std::endl;

    if(desiredVelocity > max_velocity)
      desiredVelocity = max_velocity;
    else if(desiredVelocity < m_params.minSpeed)
      desiredVelocity = 0;

    for(unsigned int i =  0; i < m_Path.size(); i++)
      m_Path.at(i).v = desiredVelocity;

    return desiredVelocity;
  }
  else if(beh.state == FOLLOW_STATE)
  {

    double deceleration_critical = 0;
    double inv_time = 2.0*((beh.followDistance- (critical_long_front_distance+m_params.additionalBrakingDistance))-CurrStatus.speed);
    if(inv_time <= 0)
      deceleration_critical = m_CarInfo.max_deceleration;
    else
      deceleration_critical = CurrStatus.speed*CurrStatus.speed/inv_time;

    if(deceleration_critical > 0) deceleration_critical = -deceleration_critical;
    if(deceleration_critical < - m_CarInfo.max_acceleration) deceleration_critical = - m_CarInfo.max_acceleration;

    double desiredVelocity = (deceleration_critical * dt) + CurrStatus.speed;

    if(desiredVelocity > m_params.maxSpeed)
      desiredVelocity = m_params.maxSpeed;

    if((desiredVelocity < 0.1 && desiredVelocity > -0.1) || beh.followDistance <= 0) //use only effective velocities
      desiredVelocity = 0;

    //std::cout << "Acc: V: " << desiredVelocity << ", Accel: " << deceleration_critical<< std::endl;

    for(unsigned int i = 0; i < m_Path.size(); i++)
      m_Path.at(i).v = desiredVelocity;

    return desiredVelocity;

  }
  else if(beh.state == FORWARD_STATE || beh.state == OBSTACLE_AVOIDANCE_STATE )
  {
    double target_velocity = max_velocity;
    bool bSlowBecauseChange=false;
    if(m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory != m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
    {
      // 切换轨迹速度 改为 正常速度 的多少倍
      target_velocity*=0.7;  // 0.5
      bSlowBecauseChange = true;
    }

    double e = target_velocity - CurrStatus.speed;
    double desiredVelocity = m_pidVelocity.getPID(e);

    if(desiredVelocity>max_velocity)
      desiredVelocity = max_velocity;
    else if(desiredVelocity < m_params.minSpeed)
      desiredVelocity = 0;

    for(unsigned int i = 0; i < m_Path.size(); i++)
      m_Path.at(i).v = desiredVelocity;

    // std::cout <<"Target velocity: "<<target_velocity<<",current velocity: "<<CurrStatus.speed<<", error : "<<e<< " , Desired Velocity: " << desiredVelocity << ", Change Slowdown: " << bSlowBecauseChange  << std::endl;

    return desiredVelocity;
  }
  else if(beh.state == STOP_SIGN_WAIT_STATE || beh.state == TRAFFIC_LIGHT_WAIT_STATE)
  {
    double target_velocity = 0;
    for(unsigned int i = 0; i < m_Path.size(); i++)
      m_Path.at(i).v = target_velocity;

    return target_velocity;
  }
  else
  {
    double target_velocity = 0;
    for(unsigned int i = 0; i < m_Path.size(); i++)
      m_Path.at(i).v = target_velocity;

    return target_velocity;
  }

  return max_velocity;
 }

 PlannerHNS::BehaviorState DecisionMaker::DoOneStep(
    const double& dt,
    const PlannerHNS::WayPoint currPose,
    const PlannerHNS::VehicleState& vehicleState,
    const int& goalID,
    const std::vector<TrafficLight>& trafficLight,
    const TrajectoryCost& tc,
    const bool& bEmergencyStop)
{
   PlannerHNS::BehaviorState beh;
   state = currPose;
   m_TotalPath.clear();
   // m_TotalOriginalPath 来自于局部路径的回调函数，是匹配局部路径后的全局轨迹
  for(unsigned int i = 0; i < m_TotalOriginalPath.size(); i++)
  {
    t_centerTrajectorySmoothed.clear();
    PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_TotalOriginalPath.at(i), state, m_params.horizonDistance ,  m_params.pathDensity , t_centerTrajectorySmoothed);
    m_TotalPath.push_back(t_centerTrajectorySmoothed);
  }

  if(m_TotalPath.size()==0) return beh;

  // UpdateCurrentLane(m_MaxLaneSearchDistance);

  CalculateImportantParameterForDecisionMaking(vehicleState, goalID, bEmergencyStop, trafficLight, tc);

  beh = GenerateBehaviorState(vehicleState);

  beh.bNewPlan = SelectSafeTrajectory();

  beh.maxVelocity = UpdateVelocityDirectlyToTrajectory(beh, vehicleState, dt);

  //std::cout << "Eval_i: " << tc.index << ", Curr_i: " <<  m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory << ", Prev_i: " << m_pCurrentBehaviorState->GetCalcParams()->iPrevSafeTrajectory << std::endl;

  return beh;
 }

 void DecisionMaker::reset(){
    m_pCurrentBehaviorState = m_pInitState;
    pre_state.pos.x = 0;
    pre_state.pos.y = 0;
    scenario_ended = false;
 }

} /* namespace PlannerHNS */
