#include <sot/core/feature-visual-point.hh>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

class FeatureGraspingPoint : public FeatureVisualPoint
{

	public:
	static const std::string CLASS_NAME;
	FeatureGraspingPoint(const std::string& name);
	
	void closeHand();
	void openHand();
    void enableAutomaticClosing(const bool& enable);

	ml::Matrix& computeJacobian( ml::Matrix& J,int time );
    ml::Vector& computeError( ml::Vector& error,int time );

	private:
	const float distance_threshold_;
	bool is_closed_;
    bool auto_closing_;

    ros::NodeHandle nh_;
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
	control_msgs::FollowJointTrajectoryGoal goal_open_;
	control_msgs::FollowJointTrajectoryGoal goal_close_;
	
};

}}
