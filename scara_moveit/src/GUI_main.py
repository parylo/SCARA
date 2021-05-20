import sys
from PyQt5 import QtWidgets
from PyQt5 import QtCore
from PyQt5.uic import loadUi

import rospy
import moveit_commander
from geometry_msgs.msg import Pose

import time
import math

class MainWindow(QtWidgets.QMainWindow):

	def __init__(self):
		super().__init__()
		loadUi('QtDesigner_GUI.ui', self)


		# Moveit commander setup
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander('scara_arm')

		# Buttons
		self.move_button.clicked.connect(self.move_to_XYZ)

		# Sliders
		self.slider_pressed_flag = False

		self.slider_slider.sliderPressed.connect(self.slider_pressed)
		self.slider_slider.sliderReleased.connect(lambda: self.move_to_joint('slider', True))
		
		self.rev1_slider.sliderPressed.connect(self.slider_pressed)
		self.rev1_slider.sliderReleased.connect(lambda: self.move_to_joint('rev1', True))
		
		self.rev2_slider.sliderPressed.connect(self.slider_pressed)
		self.rev2_slider.sliderReleased.connect(lambda: self.move_to_joint('rev2', True))

	def slider_pressed(self):
		self.slider_pressed_flag = True


	def update_robot_state_labels(self):
		'''
		Get current XYZ position of end-effector and joints position from Moveit Commander. 
		Convert values from [m] to [mm] and from radians to degrees. 
		Display values in corresponding fields on GUI.
		'''

		xyz_position = self.group.get_current_pose().pose.position
		joints_position = self.group.get_current_joint_values()

		x_position = round(xyz_position.x*1000)
		y_position = round(xyz_position.y*1000)
		z_position = round(xyz_position.z*1000)

		slider_current = round(joints_position[0]*1000)
		rev1_current = round(math.degrees(joints_position[1]))
		rev2_current = round(math.degrees(joints_position[2]))

		self.x_current_pos.setText(str(x_position))
		self.y_current_pos.setText(str(y_position))
		self.z_current_pos.setText(str(z_position))

		if self.slider_pressed_flag:
			self.slider_current_val.setText(str(self.slider_slider.value()))
			self.rev1_current_val.setText(str(self.rev1_slider.value()))
			self.rev2_current_val.setText(str(self.rev2_slider.value()))
		else:	
			self.slider_current_val.setText(str(slider_current))
			self.rev1_current_val.setText(str(rev1_current))
			self.rev2_current_val.setText(str(rev2_current))
		
		
		if not self.slider_pressed_flag:
			self.slider_slider.setValue(slider_current)
			self.rev1_slider.setValue(rev1_current)
			self.rev2_slider.setValue(rev2_current)
		
	def move_to_XYZ(self):
		'''
		Get end-efector XYZ possition from user. 
		Send position to Moveit Commander in order to execute manipulator motion. 
		'''

		target_pose = Pose()

		target_pose.position.x = int(self.X_line_edit.text())/1000
		target_pose.position.y = int(self.Y_line_edit.text())/1000
		target_pose.position.z = int(self.Z_line_edit.text())/1000


		self.group.set_position_target([target_pose.position.x,  target_pose.position.y, target_pose.position.z], 'gripper')
		self.group.set_goal_tolerance(0.005)
		self.group.go()

	def move_to_joint(self, joint, slider_used = False):
		'''
		Get joint positions from slider possition.
		Send joint position to Moveit Commander in order to execute manipulator motion. 
		'''
		if slider_used:
			self.slider_pressed_flag = False

		if joint == 'slider':
			joint_pos_target = self.slider_slider.value()/1000
			self.group.set_joint_value_target([joint_pos_target, 0, 0])
		elif joint == 'rev1':
			joint_pos_target  = math.radians(self.rev1_slider.value())
			self.group.set_joint_value_target([0.2, joint_pos_target, 0])
		elif joint == 'rev2':
			joint_pos_target  = math.radians(self.rev2_slider.value())
			self.group.set_joint_value_target([0.2, 0, joint_pos_target,])
		
		
		self.group.go()

		



if __name__ == '__main__':

	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('moveit_commander', anonymous=True)

	app = QtWidgets.QApplication(sys.argv)

	widget = MainWindow()
	widget.show()

	timer = QtCore.QTimer()
	timer.timeout.connect(widget.update_robot_state_labels)
	timer.start(200) 

	sys.exit(app.exec_())


	
