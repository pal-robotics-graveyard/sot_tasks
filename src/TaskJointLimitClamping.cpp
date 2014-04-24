/*
 * Copyright (c) 2013, PAL Robotics, S.L.
 * Copyright 2011, Nicolas Mansard, LAAS-CNRS
 *
 * This file is part of sot-dyninv.
 * sot-dyninv is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-dyninv is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-dyninv.  If not, see <http://www.gnu.org/licenses/>.
 */

/** \author Karsten Knese
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/all-commands.h>

#include <sot_tasks/TaskJointLimitClamping.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;
using namespace dynamicgraph::sot::dyninv;


/* --- DG FACTORY ------------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskJointLimitClamping,"TaskJointLimitClamping");

/* ---------------------------------------------------------------------- */
/* --- CONSTRUCTION ----------------------------------------------------- */
/* ---------------------------------------------------------------------- */


TaskJointLimitClamping::TaskJointLimitClamping( const std::string & name )
    : TaskJointLimits(name)
    ,CONSTRUCT_SIGNAL_IN(upperVelocityLimits,ml::Vector)
{
//    jacobianSOUT.setFunction( boost::bind(&TaskJointLimitClamping::jacobianSOUT_function,this,_1,_2) );
    taskSOUT.setFunction( boost::bind(&TaskJointLimitClamping::computeTask,this,_1,_2) );
    taskSOUT.addDependency( upperVelocityLimitsSIN );
    // Commands
    std::string docstring;

    docstring =
            "\n"
            " Child of Task Joint Limits in order to clamp the upper bound to a signal defined upper bound."
            "\n";

    signalRegistration(upperVelocityLimitsSIN);
}

dynamicgraph::sot::VectorMultiBound&
TaskJointLimitClamping::computeTask(dynamicgraph::sot::VectorMultiBound& res,const int& iter )
{
    const dynamicgraph::sot::VectorMultiBound& taskJointLimits = TaskJointLimits::computeTask(res, iter);
    const ml::Vector& upperVelocityLimits = upperVelocityLimitsSIN(iter);
    const Flags& selec = selecSIN(iter);

    // respect the offset for the 6DoF FreeFlyer
    // velocityLimit signal comes inclusively the FF
    // taskjointlimits from parent already takes care about it and removes FF
    assert((taskJointLimits.size()+6)==upperVelocityLimits.size());

    int idx =0;
    for (int i = 6; i < (int)upperVelocityLimits.size(); ++i) {
        if (selec(i)){
            double upperBound = taskJointLimits[idx].boundSup;
            double lowerBound = taskJointLimits[idx].boundInf;
            // checking for absolute boundary conditions.
            // limit both spinning directions of the motors to the absolute limit value
            if((taskJointLimits[idx]).boundSup >upperVelocityLimits(i)){
                upperBound = upperVelocityLimits(i);
            }
            if ((taskJointLimits[idx]).boundInf < (-1*upperVelocityLimits(i))){
                lowerBound = -1*upperVelocityLimits(i);
            }
            res[idx] = MultiBound(lowerBound, upperBound);
        }
        idx++;
    }
    return res;
}
