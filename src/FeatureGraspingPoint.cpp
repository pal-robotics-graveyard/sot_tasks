#include <sot_tasks/FeatureGraspingPoint.h>
#include <sot/core/factory.hh>
#include <dynamic-graph/all-commands.h>
#include <sot/core/exception-feature.hh>
using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureGraspingPoint,"FeatureGraspingPoint");


FeatureGraspingPoint::FeatureGraspingPoint(const string& pointName) 
: FeatureVisualPoint( pointName), 
  distance_threshold_(0.01f), 
  is_closed_(true)
{

	openHand();
	std::string docstring;
	docstring =
	    "\n"
	    "    Opens the hand, being ready for grasping "
	    "\n";
	addCommand(std::string("openHand"),
	       dynamicgraph::command::makeCommandVoid0(*this, &FeatureGraspingPoint::openHand, docstring));


	docstring =
	    "\n"
	    "    Opens the hand, being ready for grasping "
	    "\n";
	addCommand(std::string("closeHand"),
	       dynamicgraph::command::makeCommandVoid0(*this, &FeatureGraspingPoint::closeHand, docstring));

}

void FeatureGraspingPoint::openHand(){
	// doo cool stuff here
	if (is_closed_)
	{
		std::cout << "i would open my hand now" << std::endl;
		is_closed_ = false;
	}
}

void FeatureGraspingPoint::closeHand(){
	// doo cool stuff here
	if (!is_closed_)
	{
		std::cout << "i would close my hand now" << std::endl;
		is_closed_ = true;
	}
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

  if( Z<0 )
    { throw(ExceptionFeature(ExceptionFeature::BAD_INIT,
				"Negative Z value in Feature-Grasping is not allowed"," (Z=%.1f).",Z)); }

  if( fabs(Z)<distance_threshold_ )
    {
	closeHand();
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

