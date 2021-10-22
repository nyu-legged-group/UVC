import pinocchio as se3
import pybullet as p
from biped import Humanoid

import time
import numpy as np
se3.switchToNumpyMatrix()
np.set_printoptions(precision=6, suppress=True)

if __name__ == "__main__":
	vec2list = lambda m: np.array(m.T).reshape(-1).tolist()
	m2a = lambda m: np.array(m).reshape(-1)
	

	lFoot, rFoot = "l_foot", "r_foot"
	bipd = Humanoid(display=True, useTorqueCtrl=False, view='front')
	lFootId=bipd.pinocchio_robot.model.getFrameId(lFoot)
	rFootId=bipd.pinocchio_robot.model.getFrameId(rFoot)
	q, dq = bipd.get_state()
	q[7] = 0.  #leg_left_hip_yaw
	q[8] = 0.  #leg_left_hip_roll
	q[9] = 25./180* 3.14  #leg_left_hip_pitch
	q[10] = -50./180.* 3.14 #leg_left_knee
	q[11] = -25./180.*3.14 #leg_left_ankle_pitch
	q[12] = 0
	q[13] = 0
	q[14] = 0
	q[15] = 25./180* 3.14 #leg_right_hip_pitch
	q[16] = -50./180.* 3.14 #leg_right_knee
	q[17] = -25./180.* 3.14 #leg_right_ankle_pitch
	q[18] = 0
	bipd.reset_state(q,dq)
	stat = p.getLinkState(bipd.robotId, bipd.bullet_endeff_ids[0])[0]
	pos = stat[2]
	q[2] -=(pos-0.005)
	bipd.reset_state(q,dq)
	stat = p.getLinkState(bipd.robotId, bipd.bullet_endeff_ids[0])[0]
	pos = stat[2]
	print('Simulator: Initial Feet Height:', pos)

	horizon_length = 2000
	q_des = q[7:].copy()
	
	for i in range(horizon_length):

		q, dq = bipd.get_state_update_pinocchio()
		com = se3.centerOfMass(bipd.pinocchio_robot.model,bipd.pinocchio_robot.data, q)
		# Apply external force
		force = [0., -500., 0]
		com = vec2list(com)
		forcePos = [0, 0, 0.1]  # define point in front of torso
		debug_line_force_from_ptr=((np.asarray(forcePos)-np.asarray(force))*0.003).tolist()
		debug_line_force_from_ptr[2]=com[2]+0.1
		if i > 500 and i < 503:
			p.applyExternalForce(objectUniqueId=bipd.robotId, \
								linkIndex=0,forceObj=force, \
								posObj=forcePos, flags=p.LINK_FRAME)
			p.addUserDebugLine(debug_line_force_from_ptr,forcePos,[1,0,0], \
								lifeTime=1, \
								parentObjectUniqueId=bipd.robotId,\
								parentLinkIndex=0)
			p.addUserDebugText(str(100),debug_line_force_from_ptr,\
								[1,0,0],lifeTime=1,parentObjectUniqueId=bipd.robotId,\
								parentLinkIndex=0)



		############################FILL OUT YOUR CONTROLLER HERE#############################

		motor_cmd = q_des








		

		############################FILL OUT YOUR CONTROLLER HERE#############################
		bipd.send_joint_command(motor_cmd)

		p.stepSimulation()

		time.sleep(1e-3)