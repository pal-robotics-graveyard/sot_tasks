#include <sot/core/feature-visual-point.hh>

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

class FeatureGraspingPoint : public FeatureVisualPoint
{

	public:
	static const std::string CLASS_NAME;
	FeatureGraspingPoint(const std::string& name);
	
	void closeHand();
	void openHand();

	ml::Matrix& computeJacobian( ml::Matrix& J,int time );

	private:
	const float distance_threshold_;
	bool is_closed_;
};

}}
