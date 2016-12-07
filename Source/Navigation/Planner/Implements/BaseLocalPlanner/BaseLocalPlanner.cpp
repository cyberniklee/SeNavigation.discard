/*
 * BaseLocalPlanner.cpp
 *
 *  Created on: 2016年12月2日
 *      Author: seeing
 */

#include "BaseLocalPlanner.h"

namespace NS_Planner {

BaseLocalPlanner::BaseLocalPlanner() {
	// TODO Auto-generated constructor stub

}

BaseLocalPlanner::~BaseLocalPlanner() {
	// TODO Auto-generated destructor stub
}

void BaseLocalPlanner::onInitialize()
{

}

bool BaseLocalPlanner::computeVelocityCommands(NS_DataType::Twist& cmd_vel)
{

}

bool BaseLocalPlanner::isGoalReached()
{

}

bool BaseLocalPlanner::setPlan(const std::vector<NS_DataType::PoseStamped>& plan)
{

}

} /* namespace NS_Planner */
