/*
	New btMultiBodySphericalJointLimit class based on original. This version allows for a
	seperate minimum value. However, must obey following rule:
	
	minimum_value < 0 < maximum_value
*/

///This file was written by Luke Murray

#include "btMultiBodySphericalJointLimit2.h"
#include "LinearMath/btTransformUtil.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "LinearMath/btIDebugDraw.h"

btMultiBodySphericalJointLimit2::btMultiBodySphericalJointLimit2(btMultiBody* body, int link, 
	btScalar swingxRange_max,
	btScalar swingxRange_min,
	btScalar swingyRange_max,
	btScalar swingyRange_min,
	btScalar twistRange_max,
	btScalar twistRange_min,
	btScalar maxAppliedImpulse)
	: btMultiBodySphericalJointLimit(body, link, 
	swingxRange_max,
	swingyRange_max,
	twistRange_max,
	maxAppliedImpulse
	)
{
	m_swingxRange_min = abs(swingxRange_min);
	m_swingyRange_min = abs(swingyRange_min);
	m_twistRange_min = abs(twistRange_min);
}


btMultiBodySphericalJointLimit2::~btMultiBodySphericalJointLimit2()
{
}

void btMultiBodySphericalJointLimit2::createConstraintRows(btMultiBodyConstraintArray& constraintRows,
												 btMultiBodyJacobianData& data,
												 const btContactSolverInfo& infoGlobal)
{
	// only positions need to be updated -- data.m_jacobians and force
	// directions were set in the ctor and never change.

	if (m_numDofsFinalized != m_jacSizeBoth)
	{
		finalizeMultiDof();
	}

	//don't crash
	if (m_numDofsFinalized != m_jacSizeBoth)
		return;
	

	if (m_maxAppliedImpulse == 0.f)
		return;

	const btScalar posError = 0;
	const btVector3 zero(0, 0, 0);

	
	btVector3 axis[3] = { btVector3(1, 0, 0), btVector3(0, 1, 0), btVector3(0, 0, 1) };
	
	btQuaternion currentQuat(m_bodyA->getJointPosMultiDof(m_linkA)[0],
		m_bodyA->getJointPosMultiDof(m_linkA)[1],
		m_bodyA->getJointPosMultiDof(m_linkA)[2],
		m_bodyA->getJointPosMultiDof(m_linkA)[3]);

	btQuaternion refQuat = m_desiredPosition;
	btVector3 vTwist(0,0,1);
	
	btVector3 vConeNoTwist = quatRotate(currentQuat, vTwist);
	vConeNoTwist.normalize();
	btQuaternion qABCone = shortestArcQuat(vTwist, vConeNoTwist);
	qABCone.normalize();
	btQuaternion qABTwist = qABCone.inverse() * currentQuat;
	qABTwist.normalize();
	btQuaternion desiredQuat = qABTwist;


	btQuaternion relRot = currentQuat.inverse() * desiredQuat;
	btVector3 angleDiff;
	btGeneric6DofSpring2Constraint::matrixToEulerXYZ(btMatrix3x3(relRot), angleDiff);
	
	btScalar limitRanges_max[3] = {m_swingxRange, m_swingyRange, m_twistRange};
	btScalar limitRanges_min[3] = {m_swingxRange_min, m_swingyRange_min, m_twistRange_min};
	
	/// twist axis/angle
	btQuaternion qMinTwist = qABTwist;
	btScalar twistAngle = qABTwist.getAngle();

	if (twistAngle > SIMD_PI)  // long way around. flip quat and recalculate.
	{
		qMinTwist = -(qABTwist);
		twistAngle = qMinTwist.getAngle();
	}
	btVector3 vTwistAxis = btVector3(qMinTwist.x(), qMinTwist.y(), qMinTwist.z());
	if (twistAngle > SIMD_EPSILON)
		vTwistAxis.normalize();
	
	if (vTwistAxis.dot(vTwist)<0)
		twistAngle*=-1.;

	angleDiff[2] = twistAngle;


	for (int row = 0; row < getNumRows(); row++)
	{
		btScalar allowed_max = limitRanges_max[row];
		btScalar allowed_min = limitRanges_min[row];
		btScalar damp = 1;
		if((angleDiff[row]>-allowed_min)&&(angleDiff[row]<allowed_max))
		{
			angleDiff[row]=0;
			damp=0;

		} else
		{
			if (angleDiff[row]>allowed_max)
			{
				angleDiff[row]-=allowed_max;
				
			}
			if (angleDiff[row]<-allowed_min)
			{
				angleDiff[row]+=allowed_min;
			} 
		}
		

		int dof = row;
		
		btScalar currentVelocity = m_bodyA->getJointVelMultiDof(m_linkA)[dof];
		btScalar desiredVelocity = this->m_desiredVelocity[row];
		
		double kd = m_use_multi_dof_params ? m_kd[row % 3] : m_kd[0];
		btScalar velocityError = (desiredVelocity - currentVelocity) * kd;

		btMatrix3x3 frameAworld;
		frameAworld.setIdentity();
		frameAworld = m_bodyA->localFrameToWorld(m_linkA, frameAworld);
		btScalar posError = 0;
		{
			btAssert(m_bodyA->getLink(m_linkA).m_jointType == btMultibodyLink::eSpherical);
			switch (m_bodyA->getLink(m_linkA).m_jointType)
			{
				case btMultibodyLink::eSpherical:
				{
					btVector3 constraintNormalAng = frameAworld.getColumn(row % 3);
					double kp = m_use_multi_dof_params ? m_kp[row % 3] : m_kp[0];
					posError = kp*angleDiff[row % 3];
					double max_applied_impulse = m_use_multi_dof_params ? m_maxAppliedImpulseMultiDof[row % 3] : m_maxAppliedImpulse;
					//should multiply by time step
					//max_applied_impulse *= infoGlobal.m_timeStep

					double min_applied_impulse = -max_applied_impulse;
					

					if (posError>0)
						max_applied_impulse=0;
					else
						min_applied_impulse=0;

					if (btFabs(posError)>SIMD_EPSILON)
					{
						btMultiBodySolverConstraint& constraintRow = constraintRows.expandNonInitializing();
						fillMultiBodyConstraint(constraintRow, data, 0, 0, constraintNormalAng,
							zero, zero, zero,//pure angular, so zero out linear parts
							posError,
							infoGlobal,
							min_applied_impulse, max_applied_impulse, true,
							1.0, false, 0, 0,
							damp);
						constraintRow.m_orgConstraint = this;
						constraintRow.m_orgDofIndex = row;
					}
					break;
				}
				default:
				{
					btAssert(0);
				}
			};
		}
	}
}


void btMultiBodySphericalJointLimit::debugDraw(class btIDebugDraw* drawer)
{
	btTransform tr;
	tr.setIdentity();
	if (m_bodyB)
	{
		btVector3 pivotBworld = m_bodyB->localPosToWorld(m_linkB, m_pivotB);
		tr.setOrigin(pivotBworld);
		drawer->drawTransform(tr, 0.1);
	}
}
