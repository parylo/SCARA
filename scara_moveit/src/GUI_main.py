import sys
from PyQt5 import QtWidgets
from PyQt5 import QtCore
from PyQt5.uic import loadUi

import rospy
import moveit_commander
from geometry_msgs.msg import Pose

import time

class MainWindow(QtWidgets.QMainWindow):

	def __init__(self):
		super().__init__()
		loadUi('QtDesigner_GUI.ui', self)


		# Moveit commander setup
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander('scara_arm')

		# Button
		self.move_button.clicked.connect(self.move_to)

	def update_XYZ_position(self):
		'''
		Get current XYZ position of end-effector from Moveit Commander. Convert values from [m] to [mm] and display then in corresponding fields in GUI.
		'''

		positions = self.group.get_current_pose().pose.position

		X_position_text = str(round(positions.x*1000))
		Y_position_text = str(round(positions.y*1000))
		Z_position_text = str(round(positions.z*1000))

		self.X_current_pos.setText(X_position_text)
		self.Y_current_pos.setText(Y_position_text)
		self.Z_current_pos.setText(Z_position_text)

	def move_to(self):
		'''
		Get end-efector XYZ possition from user. Send mentioned position to Moveit Commander in order to execute manipulator motion. 
		'''
		target_pose = Pose()

		target_pose.position.x = int(self.X_line_edit.text())/1000
		target_pose.position.y = int(self.Y_line_edit.text())/1000
		target_pose.position.z = int(self.Z_line_edit.text())/1000


		self.group.set_position_target([target_pose.position.x,  target_pose.position.y, target_pose.position.z], 'gripper')
		self.group.set_goal_tolerance(0.005)
		self.group.go()



if __name__ == '__main__':

	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('moveit_commander', anonymous=True)

	app = QtWidgets.QApplication(sys.argv)

	widget = MainWindow()
	widget.show()

	timer = QtCore.QTimer()
	timer.timeout.connect(widget.update_XYZ_position)
	timer.start(200) 

	sys.exit(app.exec_())


	
