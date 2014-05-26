#include <sot_tasks/FeatureGraspingPoint.h>
#include <sot/core/factory.hh>
#include <dynamic-graph/all-commands.h>
#include <sot/core/exception-feature.hh>

#include <actionlib/client/terminal_state.h>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureGraspingPoint,"FeatureGraspingPoint");


FeatureGraspingPoint::FeatureGraspingPoint(const string& pointName) 
    : FeatureVisualPoint( pointName),
      distance_threshold_(0.05f),
      is_closed_(true),
      auto_closing_(true),
      nh_(),
      ac_(nh_,"right_hand_controller/follow_joint_trajectory", true)
{
    std::string docstring;
    docstring =
            "\n"
            "    Opens the hand, being ready for grasping "
            "\n";
    addCommand(std::string("openHand"),
               dynamicgraph::command::makeCommandVoid0(*this,
                                                       &FeatureGraspingPoint::openHand,
                                                       docstring)
               );


    docstring =
            "\n"
            "    Opens the hand, being ready for grasping "
            "\n";
    addCommand(std::string("closeHand"),
               dynamicgraph::command::makeCommandVoid0(*this,
                                                       &FeatureGraspingPoint::closeHand,
                                                       docstring)
               );

    docstring =
            "\n"
            "    Initializes collisions for task avoidance\n"
            "      takes a string of signal names (separated by :) for collision pairs \n"
            "     and creates a signal input socket (p1,p2, jVel_p1) for each collision object"
            "\n";
    addCommand(std::string("enableAutomaticClosing"),
               new dynamicgraph::command::Setter<FeatureGraspingPoint, bool >
               (*this, &FeatureGraspingPoint::enableAutomaticClosing, docstring));

    // create the action client
    // true causes the client to spin its own thread
    bool in_time = ac_.waitForServer(ros::Duration(20));
    if (!in_time)
    {
        ROS_ERROR_STREAM("sot grasping could not reach any action server");
    }
    ROS_INFO_STREAM("sot_grasping action server successfully initialized");

    goal_open_ = control_msgs::FollowJointTrajectoryGoal();

    goal_open_.trajectory.joint_names.push_back("hand_right_thumb_joint");
    goal_open_.trajectory.joint_names.push_back("hand_right_middle_joint");
    goal_open_.trajectory.joint_names.push_back("hand_right_index_joint");

    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(4);

    point.positions.push_back(0.1);
    point.positions.push_back(0.1);
    point.positions.push_back(0.1);

    point.velocities.push_back(0);
    point.velocities.push_back(0);
    point.velocities.push_back(0);

    goal_open_.trajectory.points.push_back(point);

    for (int i=0;i<goal_open_.trajectory.joint_names.size();++i)
    {
        control_msgs::JointTolerance tol;
        tol.name = goal_open_.trajectory.joint_names[i];
        tol.position = 5;
        tol.velocity = 5;
        tol.acceleration = 5;
        goal_open_.goal_tolerance.push_back(tol);
    }
    goal_open_.goal_time_tolerance = ros::Duration(3);

    goal_close_ = control_msgs::FollowJointTrajectoryGoal(goal_open_);
    goal_close_.trajectory.points[0].positions[0] = 1.5;
    goal_close_.trajectory.points[0].positions[1] = 4.0;
    goal_close_.trajectory.points[0].positions[2] = 4.0;

    openHand();
}

void FeatureGraspingPoint::openHand(){
    // doo cool stuff here
    if (is_closed_)
    {
        std::cout << "i open my hand now" << std::endl;
        goal_open_.trajectory.header.stamp = ros::Time::now();
        ac_.sendGoal(goal_open_);

        bool result = ac_.waitForResult(ros::Duration(5));
        std::cerr << "opening was: " << (result?"good":"wrong") << std::endl;

        is_closed_ = false;
    }
}

void FeatureGraspingPoint::closeHand(){
    // doo cool stuff here
    if (!is_closed_)
    {
        std::cout << "i close my hand now" << std::endl;
        goal_close_.trajectory.header.stamp = ros::Time::now();
        ac_.sendGoal(goal_close_);

        bool result = ac_.waitForResult(ros::Duration(5));
        std::cerr << "closing was: " << (result?"good":"wrong") << std::endl;
        is_closed_ = true;
    }
}

void FeatureGraspingPoint::enableAutomaticClosing(const bool& enable)
{
    auto_closing_ = enable;
}

ml::Vector&
FeatureGraspingPoint::computeError( ml::Vector& error,int time )
{
  const Flags &fl = selectionSIN(time);
  error.resize(dimensionSOUT(time)) ;
  unsigned int cursorL = 0;

  // distance in z to a specific point in 3D
  const double & Z = ZSIN(time);
  // xy point in 2D
  const double & x = xySIN(time)(0);
  const double & y = xySIN(time)(1);

  float d = sqrt(x*x+y*y+Z*Z);
  if (d < 0.1f){
      error.setZero();
      return error;
  }

  if(! isReferenceSet() )
    { throw(ExceptionFeature(ExceptionFeature::BAD_INIT,
                 "S* is not of adequate type.")); }

  if( fl(0) )
    { error( cursorL++ ) = x - getReference()->xySIN(time)(0) ;   }
  if( fl(1) )
    { error( cursorL++ ) = y - getReference()->xySIN(time)(1) ;   }

  return error ;
}

ml::Matrix& FeatureGraspingPoint::
computeJacobian( ml::Matrix& J,int time )
{
    const Flags &fl = selectionSIN(time);

    const int dim = dimensionSOUT(time);
    L.resize(dim,6) ;
    unsigned int cursorL = 0;

    // distance in z to a specific point in 3D
    const double & Z = ZSIN(time);
    // xy point in 2D
    const double & x = xySIN(time)(0);
    const double & y = xySIN(time)(1);

    std::cerr << "Z " << Z << std::endl;
    std::cerr << "x " << x << std::endl;
    std::cerr << "y " << y << std::endl;

    if( Z<0 )
    {
        std::cerr << "unable to look at ball" << std::endl;
        L.multiply(articularJacobianSIN(time),J);
        return J;
    }

    float d = sqrt(x*x+y*y+Z*Z);
    if (d < 0.1f){
        if( fabs(d)<distance_threshold_ && auto_closing_)
        {
            closeHand();
        }
            return J;
    }

    if( fl(0) )
    {
        L(cursorL,0) = -1/Z  ;
        L(cursorL,1) = 0 ;
        L(cursorL,2) = x/Z ;
        L(cursorL,3) = x*y ;
        L(cursorL,4) = -(1+x*x) ;
        L(cursorL,5) = y ;

        cursorL++;
    }

    if( fl(1) )
    {
        L(cursorL,0) = 0 ;
        L(cursorL,1)  = -1/Z ;
        L(cursorL,2) = y/Z ;
        L(cursorL,3) = 1+y*y ;
        L(cursorL,4) = -x*y ;
        L(cursorL,5) = -x ;

        cursorL++;
    }

    // Jacobian of articulated joint
    // fancyVector^T * J
    L.multiply(articularJacobianSIN(time),J);

    return J;
}


