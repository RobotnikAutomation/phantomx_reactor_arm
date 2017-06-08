#!/usr/bin/env python

"""
	@file phantomx_reactor_parallel_motor_joints.py
	
	
	Subscribes:
		- joint_command
		
	Publishes:
		- joint_states
		
	@author: Robotnik Automation
	
	Software License Agreement (BSD License)
	
	Copyright (c) 2015 Robotnik Automation SLL. All Rights Reserved.

	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
	   in the documentation and/or other materials provided with the distribution.

	3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY ROBOTNIK AUTOMATION SLL "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
	AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
	OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
	AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
	EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest('phantomx_reactor_arm_controller')
import rospy
import actionlib
from control_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import JointState



class PhantomXReactor:
	"""
		Class to communicate with the dinamyxel controllers of the arm
	"""	
	def __init__(self):
		
		try:
			self.name = rospy.get_param('~name', default = 'phantomx_reactor')
			self.j1 = rospy.get_param('~j1', default = 'shoulder_yaw_joint')
			self.j2 = rospy.get_param('~j2', default = 'shoulder_pitch_joint')
			self.j3 = rospy.get_param('~j3', default = 'elbow_pitch_joint')
			self.j4 = rospy.get_param('~j4', default = 'wrist_pitch_joint')
			self.j5 = rospy.get_param('~j5', default = 'wrist_roll_joint')
			self.j6 = rospy.get_param('~j6', default = 'gripper_revolute_joint')
			self.controller_1 = rospy.get_param('~controller_1', default = '/shoulder_yaw_joint/command')
			self.controller_2 = rospy.get_param('~controller_2', default = '/shoulder_pitch_joint/command')
			self.controller_3 = rospy.get_param('~controller_3', default = '/shoulder_pitch_mimic_joint/command')
			self.controller_4 = rospy.get_param('~controller_4', default = '/elbow_pitch_joint/command')
			self.controller_5 = rospy.get_param('~controller_5', default = '/elbow_pitch_mimic_joint/command')
			self.controller_6 = rospy.get_param('~controller_6', default = '/wrist_pitch_joint/command')
			self.controller_7 = rospy.get_param('~controller_7', default = '/wrist_roll_joint/command') # optional
			self.controller_8 = rospy.get_param('~controller_8', default = '/gripper_revolute_joint/command')
			
			# j2 has to controllers syncronized (2,3)
			# j3 has to controllers syncronized (4,5)
			self.controller_1_publisher = rospy.Publisher(self.controller_1, Float64, queue_size = 10)
			self.controller_2_publisher = rospy.Publisher(self.controller_2, Float64, queue_size = 10)
			self.controller_3_publisher = rospy.Publisher(self.controller_3, Float64, queue_size = 10)
			self.controller_4_publisher = rospy.Publisher(self.controller_4, Float64, queue_size = 10)
			self.controller_5_publisher = rospy.Publisher(self.controller_5, Float64, queue_size = 10)
			self.controller_6_publisher = rospy.Publisher(self.controller_6, Float64, queue_size = 10)
			self.controller_7_publisher = rospy.Publisher(self.controller_7, Float64, queue_size = 10)
			self.controller_8_publisher = rospy.Publisher(self.controller_8, Float64, queue_size = 10)
			
			# Alternative command topics
			self.joint_1_command = rospy.Subscriber('~'+self.j1+"/command", Float64, self.joint1CommandCb)
			self.joint_2_command = rospy.Subscriber('~'+self.j2+"/command", Float64, self.joint2CommandCb)
			self.joint_3_command = rospy.Subscriber('~'+self.j3+"/command", Float64, self.joint3CommandCb)
			self.joint_4_command = rospy.Subscriber('~'+self.j4+"/command", Float64, self.joint4CommandCb)
			self.joint_5_command = rospy.Subscriber('~'+self.j5+"/command", Float64, self.joint5CommandCb)
			self.joint_6_command = rospy.Subscriber('~'+self.j6+"/command", Float64, self.joint6CommandCb)
			
			
			self.joint_command = rospy.Subscriber('~joint_command', JointState, self.jointCommandCb)
			
			self.desired_freq = rospy.get_param('~desired_freq', default = 10.0)
			
		except rospy.ROSException, e:
			rospy.logerr('%s: error getting params %s'%(rospy.get_name(), e))
			exit()

		
	def jointCommandCb(self, msg):
		"""
			Receives joint command and send it to the controller
		"""
		if len(msg.name) != len(msg.position):
			rospy.logerr('%s:jointCommandCb: length of joint names and position has to be equal'%(rospy.get_name()))
			return 
		for i in range(len(msg.name)):
			#print 'Joint %s to %lf'%(msg.name[i], msg.position[i])
			self.setJointPosition(msg.name[i], msg.position[i])
		
	def joint1CommandCb(self, msg):
		"""
			Receives joint 1 command and send it to the controller
		"""	
		self.setJointPosition(self.j1, msg.data)
		
	def joint2CommandCb(self, msg):
		"""
			Receives joint 2 command and send it to the controller
		"""	
		self.setJointPosition(self.j2, msg.data)
	
	def joint3CommandCb(self, msg):
		"""
			Receives joint 3 command and send it to the controller
		"""	
		self.setJointPosition(self.j3, msg.data)
		
	
	def joint4CommandCb(self, msg):
		"""
			Receives joint 4 command and send it to the controller
		"""	
		self.setJointPosition(self.j4, msg.data)
		
	
	def joint5CommandCb(self, msg):
		"""
			Receives joint 5 command and send it to the controller
		"""	
		self.setJointPosition(self.j5, msg.data)
		
	
	def joint6CommandCb(self, msg):
		"""
			Receives joint 6 command and send it to the controller
		"""	
		self.setJointPosition(self.j6, msg.data)
		

	def controlLoop(self):
		"""
			Runs the control loop
		"""
		t_sleep = 1.0/self.desired_freq
		
		while not rospy.is_shutdown():
			rospy.sleep(t_sleep)
				
	
	def setJointPosition(self, joint, position):
		"""
			Sends the command to the controller to set the desired joint position
		"""	
		msg = Float64()
		msg.data = position
		
		if joint == self.j1:
			self.controller_1_publisher.publish(msg)
		elif joint == self.j2:
			self.controller_2_publisher.publish(msg)
			msg.data = -position
			self.controller_3_publisher.publish(msg)
		elif joint == self.j3:
			self.controller_4_publisher.publish(msg)
			msg.data = -position
			self.controller_5_publisher.publish(msg)
		elif joint == self.j4:
			self.controller_6_publisher.publish(msg)
		elif joint == self.j5:
			self.controller_7_publisher.publish(msg)
		elif joint == self.j6:
			self.controller_8_publisher.publish(msg)
		
			
	def start(self):
		"""
			Starts the control loop and runs spin
		"""	
		try:
			self.controlLoop()
		except rospy.ROSInterruptException:
			rospy.loginfo('%s: Bye!'%rospy.get_name())


def main():

	rospy.init_node('robotnik_phantomx_reactor')
		
	phantomx_reactor_node = PhantomXReactor()
	
	phantomx_reactor_node.start()
	

	
if __name__=='__main__':
	main()
	exit()
	
