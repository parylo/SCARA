import sys
from PyQt5 import QtWidgets
from PyQt5 import QtCore
from PyQt5.uic import loadUi

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

import math

class MainWindow(QtWidgets.QMainWindow):

	def __init__(self):
		super().__init__()
		loadUi('QtDesigner_GUI.ui', self)

		# Moveit commander setup
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander('scara_arm')

		# ROS
		self.gripper_state_pub = rospy.Publisher('/gripper_state', Bool, queue_size=1)

		# Buttons
		self.move_button.clicked.connect(self.move_to_XYZ)
		self.home_button.clicked.connect(self.home_action)
		self.gripper_button.clicked.connect(self.gripper_action)
		

		self.xplus_button.clicked.connect(lambda: self.move_to_direction('x'))
		self.xminus_button.clicked.connect(lambda: self.move_to_direction('x', False))
		self.yplus_button.clicked.connect(lambda: self.move_to_direction('y'))
		self.yminus_button.clicked.connect(lambda: self.move_to_direction('y', False))
		self.zplus_button.clicked.connect(lambda: self.move_to_direction('z'))
		self.zminus_button.clicked.connect(lambda: self.move_to_direction('z', False))

		# Radio Buttons
		self.move_distance = 0.01
		self.radio_button_1.toggled.connect(lambda:self.set_distance(self.radio_button_1))
		self.radio_button_2.toggled.connect(lambda:self.set_distance(self.radio_button_2))
		self.radio_button_3.toggled.connect(lambda:self.set_distance(self.radio_button_3))
	
		# Sliders
		self.slider_pressed_flag = False

		self.slider_slider.sliderPressed.connect(self.slider_pressed)
		self.slider_slider.sliderReleased.connect(lambda: self.move_to_joint('slider', True))
		self.rev1_slider.sliderPressed.connect(self.slider_pressed)
		self.rev1_slider.sliderReleased.connect(lambda: self.move_to_joint('rev1', True))
		self.rev2_slider.sliderPressed.connect(self.slider_pressed)
		self.rev2_slider.sliderReleased.connect(lambda: self.move_to_joint('rev2', True))

	def slider_pressed(self):
		'''Update slider pressed flag.'''

		self.slider_pressed_flag = True

	def set_distance(self, radio_button):
		'''After pressing radio button, distance to move is set value from button label (convert [mm] -> [m]).'''

		if radio_button.isChecked():
			button_value = int(radio_button.text().split()[0])
			self.move_distance = button_value/1000

	def update_XYZ_labels(self):
		'''
		Get current XYZ position of end-effector from Moveit Commander. 
		Convert [m] -> [mm]. 
		Display values in corresponding fields on GUI.
		'''

		xyz_position = self.group.get_current_pose().pose.position
	
		x_position = round(xyz_position.x*1000)
		y_position = round(xyz_position.y*1000)
		z_position = round(xyz_position.z*1000)

		self.x_current_pos.setText(str(x_position))
		self.y_current_pos.setText(str(y_position))
		self.z_current_pos.setText(str(z_position))

	def update_joints_position_labels(self):
		'''
		Get current joints position from Moveit Commander. 
		Convert [m] -> [mm] and [rad] -> [deg]. 
		Display values in corresponding fields on GUI. Update joints slider position.
		In case when user grab a slider, labels values are updated from slider position.
		'''

		joints_position = self.group.get_current_joint_values()

		slider_current = round(joints_position[0]*1000)
		rev1_current = round(math.degrees(joints_position[1]))
		rev2_current = round(math.degrees(joints_position[2]))

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

		joints_position = self.group.get_current_joint_values()

		if slider_used:
			self.slider_pressed_flag = False

		if joint == 'slider':
			joint_pos_target = self.slider_slider.value()/1000
			joints_position[0] = joint_pos_target
			self.group.set_joint_value_target(joints_position)
		
		elif joint == 'rev1':
			joint_pos_target  = math.radians(self.rev1_slider.value())
			joints_position[1] = joint_pos_target
			self.group.set_joint_value_target(joints_position)
		
		elif joint == 'rev2':
			joint_pos_target  = math.radians(self.rev2_slider.value())
			joints_position[2] = joint_pos_target
			self.group.set_joint_value_target(joints_position)
		
		self.group.go()

	def move_to_direction(self, axis, dir_positive = True):
		'''Move end-effector to specific direction. Distance to move is based on user choice of radio buttons.'''

		target_position = self.group.get_current_pose().pose.position

		if axis == 'x':
			if dir_positive:
				target_position.x += self.move_distance
			else:
				target_position.x -= self.move_distance
		elif axis == 'y':
			if dir_positive:
				target_position.y += self.move_distance
			else:
				target_position.y -= self.move_distance
		elif axis == 'z':
			if dir_positive:
				target_position.z += self.move_distance
			else:
				target_position.z -= self.move_distance

		self.group.set_position_target([target_position.x,  target_position.y, target_position.z], 'gripper')

		self.group.set_goal_tolerance(0.005)
		self.group.go()
		
	def home_action(self):
		'''Move manipilator to predefined home position.'''

		self.group.set_named_target("home")
		self.group.go()

	def gripper_action(self, button_state):
		'''Based on GRIPPER button state, publish gripper command On (True) or Off (False) on gripper state topic.'''

		gripper_state = Bool()

		if self.gripper_button.isChecked():

			gripper_state.data = True
			print('Gripper ON')

		else:

			gripper_state.data = False
			print('Gripper OFF')

		self.gripper_state_pub.publish(gripper_state)
		

if __name__ == '__main__':

	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('moveit_commander', anonymous=True)

	app = QtWidgets.QApplication(sys.argv)

	widget = MainWindow()
	widget.show()

	timer = QtCore.QTimer()
	timer.timeout.connect(widget.update_XYZ_labels)
	timer.timeout.connect(widget.update_joints_position_labels)
	timer.start(200) 

	sys.exit(app.exec_())


	
