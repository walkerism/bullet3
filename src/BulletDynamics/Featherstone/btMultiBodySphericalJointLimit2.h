/*
	New btMultiBodySphericalJointLimit class based on original. This version allows for a
	seperate minimum value. However, must obey following rule:
	
	minimum_value < 0 < maximum_value
*/

///This file was written by Luke Murray

#ifndef BT_MULTIBODY_SPHERICAL_JOINT_LIMIT_2_H
#define BT_MULTIBODY_SPHERICAL_JOINT_LIMIT_2_H

#include "btMultiBodySphericalJointLimit.h"

class btMultiBodySphericalJointLimit2 : public btMultiBodySphericalJointLimit
{
protected:
	// m_swingxRange, m_swingyRange, m_twistRange are used for _max ranges
	btScalar m_swingxRange_min;
	btScalar m_swingyRange_min;
	btScalar m_twistRange_min;

public:
	btMultiBodySphericalJointLimit2(btMultiBody* body, int link, 
		btScalar swingxRange_max,
		btScalar swingxRange_min,
		btScalar swingyRange_max,
		btScalar swingyRange_min,
		btScalar twistRange_max,
		btScalar twistRange_min,
		btScalar maxAppliedImpulse);
	
	virtual ~btMultiBodySphericalJointLimit2();

	virtual void createConstraintRows(btMultiBodyConstraintArray& constraintRows,
									  btMultiBodyJacobianData& data,
									  const btContactSolverInfo& infoGlobal);
};

#endif  //BT_MULTIBODY_SPHERICAL_JOINT_LIMIT_2_H
