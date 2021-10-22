import os
import inspect
import numpy as np
import time

import pybullet as p
import pybullet_data
import pinocchio as se3
from pinocchio.utils import *

from py_pinocchio_bullet.wrapper import PinBulletWrapper
from robot_properties_solo.config import SoloConfig

vec2list = lambda m: np.array(m.T).reshape(-1).tolist()
list2vec = lambda m: np.matrix(m).T
np.set_printoptions(precision=5, suppress=True)
def m2a(m): return np.array(m.flat)
def a2m(a): return np.matrix(a).T


currentdir = os.path.dirname(os.path.abspath(
	inspect.getfile(inspect.currentframe())))
savepath = os.path.dirname(currentdir) +'/data/'

se3.switchToNumpyMatrix()

class Humanoid(PinBulletWrapper):
	def __init__(self, physicsClient=None, display=False,timeStep=1e-3, rendering=0, view='side', useTorqueCtrl=False):
		if physicsClient is None:
			if display:
				self.physicsClient = p.connect(p.GUI)
			else:
				self.physicsClient = p.connect(p.DIRECT)
			# Setup for fall simulation, while setup walking, do turn this off!
			p.setGravity(0, 0, -9.81)
			# p.setGravity(0,0,0)
			p.setPhysicsEngineParameter(
				fixedTimeStep=timeStep, numSubSteps=1)
			p.setPhysicsEngineParameter(numSolverIterations=150)
			# Load the plain.

			p.setAdditionalSearchPath(pybullet_data.getDataPath())
			plain_urdf = (SoloConfig.packPath +
					  "/urdf/plane_with_restitution.urdf")
			self.planeId = p.loadURDF(plain_urdf, useFixedBase=1)

			# p.changeDynamics(self.planeId, -1, restitution=0.99,
			# 				lateralFriction=1.0, spinningFriction=1.0, rollingFriction=1.0,
			# 				contactStiffness=1e6, contactDamping=2e3)

			robotStartPos = [0., 0, 0.0014875+0.5095826451096808]
			robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

			# Use real time simulation
			self.useRealTimeSim = 0
			
			# Use rendering tool for real time video output
			self.rendering = rendering
			# self.urdf_path_pybullet = SoloConfig.urdf_path_pin_wo_ball
			self.urdf_path_pybullet = SoloConfig.urdf_path_pin_biped
			self.pack_path = SoloConfig.packPath
			self.urdf_path = SoloConfig.urdf_path_pin_biped
			print('urdf_path_pybullet:',self.urdf_path_pybullet)
			self.robotId = p.loadURDF(self.urdf_path_pybullet, robotStartPos,
				robotStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE,
				useFixedBase=False)

			p.getBasePositionAndOrientation(self.robotId)

			# Create the robot wrapper in pinocchio.
			package_dirs = [os.path.dirname(
				os.path.dirname(self.urdf_path)) + '/urdf']
			self.pin_robot = SoloConfig.buildBipedWrapper()
			# self.pin_robot = SoloConfig.buildHumanoidWoBallWrapper()
			# Query all the joints.
			num_joints = p.getNumJoints(self.robotId)

			for ji in range(num_joints):
				p.changeDynamics(self.robotId, ji, linearDamping=0.,
					angularDamping=0., restitution=0., lateralFriction=1.0)
			self.base_link_name = "base_link"
			self.joint_names = ['j_l_hip_y', 'j_l_hip_r', 'j_l_hip_p', 'j_l_knee', 'j_l_ankle_p',
			'j_l_ankle_r', 'j_r_hip_y', 'j_r_hip_r', 'j_r_hip_p', 'j_r_knee', 'j_r_ankle_p',
			'j_r_ankle_r']
			controlled_joints = ['j_l_hip_y', 'j_l_hip_r', 'j_l_hip_p', 'j_l_knee', 'j_l_ankle_p',
			'j_l_ankle_r', 'j_r_hip_y', 'j_r_hip_r', 'j_r_hip_p', 'j_r_knee', 'j_r_ankle_p',
			'j_r_ankle_r']

			# Creates the wrapper by calling the super.__init__.
			super(Humanoid, self).__init__(self.robotId, self.pin_robot,
				controlled_joints,
				['j_l_foot', 'j_r_foot'], useTorqueCtrl = useTorqueCtrl
			)

			# Adjust view, close unrelevant window and rendering parameter

			# region
			resultPath = str(os.path.abspath(os.path.join(self.pack_path, '../humanoid_simulation/result')))
			if self.useRealTimeSim == 1:
				p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, os.path.join(resultPath, 'Humanoid_log.mp4'))
				p.setRealTimeSimulation(self.useRealTimeSim)
			p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
			p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
			front_view = [0.05,120,-30,[0.6,0.2,0.60]]
			#front_view = [0.05,120,-30,[3,1,3]]
			side_view = [0.1,150,-30,[0.45,0.45,0.45]]
			top_view = [0.4,-90,-89.9,[0.15,0,0.4]]
			left_view = [0.65, 179.9, -20, [0.15,0,0.4]]
			right_view = [0.65, 0.1, -20, [0.15,0,0.4]]

			def front():
				p.resetDebugVisualizerCamera(cameraDistance=front_view[0],
				cameraYaw=front_view[1],
				cameraPitch=front_view[2],
				cameraTargetPosition=front_view[3])
				self.camTargetPos = front_view[3]
				self.yaw = front_view[1]
				self.pitch = front_view[2]+15
				self.roll = 0
				self.camDistance = front_view[0]
			def side():
				p.resetDebugVisualizerCamera(cameraDistance=side_view[0],
				cameraYaw=side_view[1],
				cameraPitch=side_view[2],
				cameraTargetPosition=side_view[3])
				self.camTargetPos = side_view[3]
				self.yaw = side_view[1]
				self.pitch = side_view[2]+15
				self.roll = 0
				self.camDistance = side_view[0]
			
			def top():
				p.resetDebugVisualizerCamera(cameraDistance=top_view[0],
				cameraYaw=top_view[1],
				cameraPitch=top_view[2],
				cameraTargetPosition=top_view[3])
			def left():
				p.resetDebugVisualizerCamera(cameraDistance=left_view[0],
				cameraYaw=left_view[1],
				cameraPitch=left_view[2],
				cameraTargetPosition=left_view[3])
				self.camTargetPos = side_view[3]
				self.yaw = side_view[1]
				self.pitch = side_view[2]+15
				self.roll = 0
				self.camDistance = side_view[0]
			def right():
				p.resetDebugVisualizerCamera(cameraDistance=right_view[0]+0.3,
				cameraYaw=right_view[1],
				cameraPitch=right_view[2],
				cameraTargetPosition=right_view[3])
				self.camTargetPos = side_view[3]
				self.yaw = side_view[1]
				self.pitch = side_view[2]+15
				self.roll = 0
				self.camDistance = side_view[0]+1
			options = {'front' : front,
				'side' : side,
				'top' : top,
				'left': left,
				'right': right,
			}
			options[view]()

			p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
			p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
			p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
			p.configureDebugVisualizer(
				p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

			p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

			
 
			self.cameraUp = [0, 0, 1]
			self.cameraPos = [1, 1, 1]

			self.upAxisIndex = 2
			
			self.pixelWidth = 800
			self.pixelHeight = 800
			self.nearPlane = 0.1
			self.farPlane = 100.0
			self.fov = 60
			self.aspect = float(self.pixelWidth) / self.pixelHeight
			self.viewMatrix = p.computeViewMatrixFromYawPitchRoll(self.camTargetPos, self.camDistance, self.yaw, self.pitch,
															self.roll, self.upAxisIndex)
			self.projectionMatrix = p.computeProjectionMatrixFOV(
				self.fov, self.aspect, self.nearPlane, self.farPlane)
			self.size = (self.pixelWidth, self.pixelHeight)

