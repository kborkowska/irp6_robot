#!/usr/bin/env python

import rospy
import tf
import actionlib
import sys
import time

from numpy import *
from numpy.linalg import *

from controller_manager_msgs.srv import *
from std_msgs.msg import *
from diagnostic_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *
from cartesian_trajectory_msgs.msg import *
from force_control_msgs.msg import *
from tf.transformations import *
from trapezoidal_trajectory_msgs.msg import *

from sensor_msgs.msg import *

import threading
import PyKDL
import tf_conversions.posemath as pm

from irpos import IRPOS

class IRPOS_T(IRPOS):


	def __init__(self, nodeName, robotName, robotJointNumbers, scheme_name):
		super(IRPOS_T, self).__init__(nodeName, robotName, robotJointNumbers, scheme_name)

		robotNameLower = robotName.lower()

		self.joint_client_trapezoid = actionlib.SimpleActionClient('/'+robotNameLower+'_arm/trapezoid_trajectory_action_joint', TrapezoidTrajectoryAction)
		self.joint_client_trapezoid.wait_for_server()

		self.motor_client_trapezoid = actionlib.SimpleActionClient('/'+robotNameLower+'_arm/trapezoid_trajectory_action_motor', TrapezoidTrajectoryAction)
		self.motor_client_trapezoid.wait_for_server()

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorMotor', self.robot_name+'mSplineTrajectoryGeneratorJoint',self.robot_name+'mTrapezoidTrajectoryGeneratorJoint', self.robot_name+'mTrapezoidTrajectoryGeneratorMotor',self.robot_name+'mPoseInt', self.robot_name+'mForceTransformation', self.robot_name+'mForceControlLaw', self.robot_name+'tfgSplineTrajectoryGeneratorJoint', self.robot_name+'tfgSplineTrajectoryGeneratorMotor'], True)

		self.joint_client_trapezoid.cancel_goal()
		self.motor_client_trapezoid.cancel_goal()

	def trapezoid_error_code_to_string(self, error_code):
		if (error_code==0): 
			return "SUCCESSFUL"
		elif (error_code==-1): 
			return self.RCOLOR+"INVALID_GOAL"
		elif (error_code==-2): 
			return self.RCOLOR+"INVALID_JOINTS"
		elif (error_code==-3): 
			return self.RCOLOR+"OLD_HEADER_TIMESTAMP"
		elif (error_code==-4): 
			return self.RCOLOR+"PATH_TOLERANCE_VIOLATED"
		elif (error_code==-5): 
			return self.RCOLOR+"GOAL_TOLERANCE_VIOLATED"
		elif (error_code==-6): 
			return self.RCOLOR+"INVALID_LIMIT_ARRAY"
		elif (error_code==-7): 
			return self.RCOLOR+"TRAJECTORY_NOT_FEASIBLE"
		elif (error_code==-8): 
			return self.RCOLOR+"CANT_CALCULATE_COEFFS"
		elif (error_code==-9): 
			return self.RCOLOR+"MAX_VEL_UNREACHEABLE"
		elif (error_code==-10): 
			return self.RCOLOR+"BREACHED_POS_LIMIT"
		elif (error_code==-11): 
			return self.RCOLOR+"ACC_TOO_SMALL_FOR_DURATION"
		elif (error_code==-12): 
			return self.RCOLOR+"DURATION_TOO_LONG"
		elif (error_code==-13): 
			return self.RCOLOR+"DURATION_TOO_SHORT"
		elif (error_code==-14): 
			return self.RCOLOR+"IMPOSSIBLE_VELOCITY"
		return "UNKNOWN"


#---------------------------------# MOTOR VELOCITY #---------------------------------#

	def move_to_motor_position_trapezoid_velocity(self, motor_positions, max_velocities, max_accelerations, save_data, research_mode):
		print self.BCOLOR+"[IRPOS][TRAPEZOID_VELOCITY] Move to motor position"+self.ENDC

		self.conmanSwitch([self.robot_name+'mTrapezoidTrajectoryGeneratorMotor'], [], True)

		motorGoal = TrapezoidTrajectoryGoal()

		motorGoal.duration_mode = False
		motorGoal.trajectory.joint_names = self.robot_joint_names
		motorGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		motorGoal.trajectory.points.append(JointTrajectoryPoint(motor_positions,\
																[0.0, 0.0, 0.0,0.0, 0.0, 0.0],\
																[0.0, 0.0, 0.0,0.0, 0.0, 0.0],\
																[], rospy.Duration(0.0)))

		for j in self.robot_joint_names:
			motorGoal.path_tolerance.append(JointTolerance(j, self.MOTOR_POS_TOLERANCE, 0, 0))
			motorGoal.goal_tolerance.append(JointTolerance(j, self.MOTOR_POS_TOLERANCE, 0, 0))

		motorGoal.save_data = save_data
		motorGoal.research_mode = research_mode
		motorGoal.max_velocities = max_velocities
		motorGoal.max_accelerations = max_accelerations

		self.motor_client_trapezoid.send_goal(motorGoal)
		self.motor_client_trapezoid.wait_for_result()

		result = self.motor_client_trapezoid.get_result()
		code = self.trapezoid_error_code_to_string(result.result.error_code)
		print self.BCOLOR+"[IRPOS][TRAPEZOID_VELOCITY] Result: "+str(code)+self.ENDC
		if (result.result.error_code != 0):
			print self.BCOLOR+"[IRPOS][TRAPEZOID_VELOCITY] "+result.result.error_string+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mTrapezoidTrajectoryGeneratorMotor'], True)
		return result


	def move_along_motor_trajectory_trapezoid_velocity(self, points, max_velocities, max_accelerations, save_data, research_mode):
		print self.BCOLOR+"[IRPOS][TRAPEZOID_VELOCITY] Move along motor trajectory"+self.ENDC

		self.conmanSwitch([self.robot_name+'mTrapezoidTrajectoryGeneratorMotor'], [], True)
		
		motorGoal = TrapezoidTrajectoryGoal()

		motorGoal.duration_mode = False
		motorGoal.trajectory.joint_names = self.robot_joint_names
		motorGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		for i in points:
			motorGoal.trajectory.points.append(JointTrajectoryPoint(i.positions, i.velocities, i.accelerations, i.effort, i.time_from_start))
			#print str(i.positions)
		for j in self.robot_joint_names:
			motorGoal.path_tolerance.append(JointTolerance(j, self.MOTOR_POS_TOLERANCE, 0, 0))
			motorGoal.goal_tolerance.append(JointTolerance(j, self.MOTOR_POS_TOLERANCE, 0, 0))

		motorGoal.save_data = save_data
		motorGoal.research_mode = research_mode
		motorGoal.max_velocities = max_velocities
		motorGoal.max_accelerations = max_accelerations

		self.motor_client_trapezoid.send_goal(motorGoal)
		self.motor_client_trapezoid.wait_for_result()
		
		result = self.motor_client_trapezoid.get_result()
		code = self.trapezoid_error_code_to_string(result.result.error_code)
		print self.BCOLOR+"[IRPOS][TRAPEZOID_VELOCITY] Result: "+str(code)+self.ENDC
		if (result.result.error_code != 0):
			print self.BCOLOR+"[IRPOS][TRAPEZOID_VELOCITY] "+result.result.error_string+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mTrapezoidTrajectoryGeneratorMotor'], True)
		return result

#---------------------------------# MOTOR DURATION #---------------------------------#

	def move_to_motor_position_trapezoid_duration(self, motor_positions, save_data, research_mode):
		print self.BCOLOR+"[IRPOS][TRAPEZOID_DURATION] Move to motor position"+self.ENDC

		self.conmanSwitch([self.robot_name+'mTrapezoidTrajectoryGeneratorMotor'], [], True)

		motorGoal = TrapezoidTrajectoryGoal()

		motorGoal.duration_mode = True
		motorGoal.trajectory.joint_names = self.robot_joint_names
		motorGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		motorGoal.trajectory.points.append(JointTrajectoryPoint(motor_positions,\
																[0.0, 0.0, 0.0,0.0, 0.0, 0.0],\
																[0.0, 0.0, 0.0,0.0, 0.0, 0.0],\
																[], rospy.Duration(0.0)))

		for j in self.robot_joint_names:
			motorGoal.path_tolerance.append(JointTolerance(j, self.MOTOR_POS_TOLERANCE, 0, 0))
			motorGoal.goal_tolerance.append(JointTolerance(j, self.MOTOR_POS_TOLERANCE, 0, 0))

		motorGoal.save_data = save_data
		motorGoal.research_mode = research_mode

		self.motor_client_trapezoid.send_goal(motorGoal)
		self.motor_client_trapezoid.wait_for_result()

		result = self.motor_client_trapezoid.get_result()
		code = self.trapezoid_error_code_to_string(result.result.error_code)
		print self.BCOLOR+"[IRPOS][TRAPEZOID_DURATION] Result: "+str(code)+self.ENDC
		if (result.result.error_code != 0):
			print self.BCOLOR+"[IRPOS][TRAPEZOID_DURATION] "+result.result.error_string+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mTrapezoidTrajectoryGeneratorMotor'], True)
		return result


	def move_along_motor_trajectory_trapezoid_duration(self, points, save_data, research_mode):
		print self.BCOLOR+"[IRPOS][TRAPEZOID_DURATION] Move along motor trajectory"+self.ENDC

		self.conmanSwitch([self.robot_name+'mTrapezoidTrajectoryGeneratorMotor'], [], True)
		
		motorGoal = TrapezoidTrajectoryGoal()

		motorGoal.duration_mode = True
		motorGoal.trajectory.joint_names = self.robot_joint_names
		motorGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		for i in points:
			motorGoal.trajectory.points.append(JointTrajectoryPoint(i.positions, i.velocities, i.accelerations, i.effort, i.time_from_start))
			#print str(i.positions)
		for j in self.robot_joint_names:
			motorGoal.path_tolerance.append(JointTolerance(j, self.MOTOR_POS_TOLERANCE, 0, 0))
			motorGoal.goal_tolerance.append(JointTolerance(j, self.MOTOR_POS_TOLERANCE, 0, 0))

		motorGoal.save_data = save_data
		motorGoal.research_mode = research_mode

		self.motor_client_trapezoid.send_goal(motorGoal)
		self.motor_client_trapezoid.wait_for_result()
		
		result = self.motor_client_trapezoid.get_result()
		code = self.trapezoid_error_code_to_string(result.result.error_code)
		print self.BCOLOR+"[IRPOS][TRAPEZOID_DURATION] Result: "+str(code)+self.ENDC
		if (result.result.error_code != 0):
			print self.BCOLOR+"[IRPOS][TRAPEZOID_DURATION] "+result.result.error_string+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mTrapezoidTrajectoryGeneratorMotor'], True)
		return result

#---------------------------------# JOINT VELOCITY #---------------------------------#

	def move_to_joint_position_trapezoid_velocity(self, joint_positions, max_velocities, max_accelerations, save_data, research_mode):
		print self.BCOLOR+"[IRPOS][TRAPEZOID_VELOCITY] Move to joint position"+self.ENDC

		self.conmanSwitch([self.robot_name+'mTrapezoidTrajectoryGeneratorJoint'], [], True)
		
		jointGoal = TrapezoidTrajectoryGoal()

		jointGoal.duration_mode = False
		jointGoal.trajectory.joint_names = self.robot_joint_names
		jointGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		jointGoal.trajectory.points.append(JointTrajectoryPoint(joint_positions,\
																[0.0, 0.0, 0.0,0.0, 0.0, 0.0],\
																[0.0, 0.0, 0.0,0.0, 0.0, 0.0],\
																[], rospy.Duration(0.0)))

		for j in self.robot_joint_names:
			jointGoal.path_tolerance.append(JointTolerance(j, self.JOINT_POS_TOLERANCE, 0, 0))
			jointGoal.goal_tolerance.append(JointTolerance(j, self.JOINT_POS_TOLERANCE, 0, 0))

		jointGoal.save_data = save_data
		jointGoal.research_mode = research_mode
		jointGoal.max_velocities = max_velocities
		jointGoal.max_accelerations = max_accelerations

		self.joint_client_trapezoid.send_goal(jointGoal)
		self.joint_client_trapezoid.wait_for_result()
		
		result = self.joint_client_trapezoid.get_result()
		code = self.trapezoid_error_code_to_string(result.result.error_code)
		print self.BCOLOR+"[IRPOS][TRAPEZOID_VELOCITY] Result: "+str(code)+self.ENDC
		if (result.result.error_code != 0):
			print self.BCOLOR+"[IRPOS][TRAPEZOID_VELOCITY] "+result.result.error_string+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mTrapezoidTrajectoryGeneratorJoint'], True)
		return result

	def move_along_joint_trajectory_trapezoid_velocity(self, points, max_velocities, max_accelerations, save_data, research_mode):
		print self.BCOLOR+"[IRPOS][TRAPEZOID_VELOCITY] Move along joint trajectory"+self.ENDC

		self.conmanSwitch([self.robot_name+'mTrapezoidTrajectoryGeneratorJoint'], [], True)
		
		jointGoal = TrapezoidTrajectoryGoal()

		jointGoal.duration_mode = False
		jointGoal.trajectory.joint_names = self.robot_joint_names
		jointGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		for i in points:
			jointGoal.trajectory.points.append(JointTrajectoryPoint(i.positions, i.velocities, i.accelerations, i.effort, i.time_from_start))
			#print str(i.positions)
		for j in self.robot_joint_names:
			jointGoal.path_tolerance.append(JointTolerance(j, self.JOINT_POS_TOLERANCE, 0, 0))
			jointGoal.goal_tolerance.append(JointTolerance(j, self.JOINT_POS_TOLERANCE, 0, 0))

		jointGoal.save_data = save_data
		jointGoal.research_mode = research_mode
		jointGoal.max_velocities = max_velocities
		jointGoal.max_accelerations = max_accelerations

		self.joint_client_trapezoid.send_goal(jointGoal)
		self.joint_client_trapezoid.wait_for_result()
		
		result = self.joint_client_trapezoid.get_result()
		code = self.trapezoid_error_code_to_string(result.result.error_code)
		print self.BCOLOR+"[IRPOS][TRAPEZOID_VELOCITY] Result: "+str(code)+self.ENDC
		if (result.result.error_code != 0):
			print self.BCOLOR+"[IRPOS][TRAPEZOID_VELOCITY] "+result.result.error_string+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mTrapezoidTrajectoryGeneratorJoint'], True)
		return result

#---------------------------------# JOINT DURATION #---------------------------------#

	def move_to_joint_position_trapezoid_duration(self, joint_positions, save_data, research_mode):
		print self.BCOLOR+"[IRPOS][TRAPEZOID_DURATION] Move to joint position"+self.ENDC

		self.conmanSwitch([self.robot_name+'mTrapezoidTrajectoryGeneratorJoint'], [], True)
		
		jointGoal = TrapezoidTrajectoryGoal()

		jointGoal.duration_mode = True
		jointGoal.trajectory.joint_names = self.robot_joint_names
		jointGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		jointGoal.trajectory.points.append(JointTrajectoryPoint(joint_positions,\
																[0.0, 0.0, 0.0,0.0, 0.0, 0.0],\
																[0.0, 0.0, 0.0,0.0, 0.0, 0.0],\
																[], rospy.Duration(0.0)))

		for j in self.robot_joint_names:
			jointGoal.path_tolerance.append(JointTolerance(j, self.JOINT_POS_TOLERANCE, 0, 0))
			jointGoal.goal_tolerance.append(JointTolerance(j, self.JOINT_POS_TOLERANCE, 0, 0))

		jointGoal.save_data = save_data
		jointGoal.research_mode = research_mode

		self.joint_client_trapezoid.send_goal(jointGoal)
		self.joint_client_trapezoid.wait_for_result()
		
		result = self.joint_client_trapezoid.get_result()
		code = self.trapezoid_error_code_to_string(result.result.error_code)
		print self.BCOLOR+"[IRPOS][TRAPEZOID_DURATION] Result: "+str(code)+self.ENDC
		if (result.result.error_code != 0):
			print self.BCOLOR+"[IRPOS][TRAPEZOID_DURATION] "+result.result.error_string+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mTrapezoidTrajectoryGeneratorJoint'], True)
		return result

	def move_along_joint_trajectory_trapezoid_duration(self, points, save_data, research_mode):
		print self.BCOLOR+"[IRPOS][TRAPEZOID_DURATION] Move along joint trajectory"+self.ENDC

		self.conmanSwitch([self.robot_name+'mTrapezoidTrajectoryGeneratorJoint'], [], True)
		
		jointGoal = TrapezoidTrajectoryGoal()

		jointGoal.duration_mode = True
		jointGoal.trajectory.joint_names = self.robot_joint_names
		jointGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		for i in points:
			jointGoal.trajectory.points.append(JointTrajectoryPoint(i.positions, i.velocities, i.accelerations, i.effort, i.time_from_start))
			#print str(i.positions)
		for j in self.robot_joint_names:
			jointGoal.path_tolerance.append(JointTolerance(j, self.JOINT_POS_TOLERANCE, 0, 0))
			jointGoal.goal_tolerance.append(JointTolerance(j, self.JOINT_POS_TOLERANCE, 0, 0))

		jointGoal.save_data = save_data
		jointGoal.research_mode = research_mode

		self.joint_client_trapezoid.send_goal(jointGoal)
		self.joint_client_trapezoid.wait_for_result()
		
		result = self.joint_client_trapezoid.get_result()
		code = self.trapezoid_error_code_to_string(result.result.error_code)
		print self.BCOLOR+"[IRPOS][TRAPEZOID_DURATION] Result: "+str(code)+self.ENDC
		if (result.result.error_code != 0):
			print self.BCOLOR+"[IRPOS][TRAPEZOID_DURATION] "+result.result.error_string+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mTrapezoidTrajectoryGeneratorJoint'], True)
		return result
