/*
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

#ifndef __sot_TaskJointLimitClamping_H__
#define __sot_TaskJointLimitClamping_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-dyninv/task-joint-limits.h>
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <sot/core/task.hh>
#include <sot/core/flags.hh>

namespace dynamicgraph {
namespace sot {
namespace dyninv{
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class TaskJointLimitClamping
        :public TaskJointLimits
{

public: /* --- CONSTRUCTOR ---- */
    static const std::string CLASS_NAME;
    TaskJointLimitClamping( const std::string& name );

public:  /* --- SIGNALS --- */
//    ml::Matrix& jacobianSOUT_function( ml::Matrix& res,const int& iter );
    dg::sot::VectorMultiBound& computeTask( dg::sot::VectorMultiBound&,const int& iter);
    DECLARE_SIGNAL_IN(upperVelocityLimits,ml::Vector);

}; // class TaskVelocityClamping

} // namepspace dynin
} // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __sot_TaskJointLimitClamping_H__

