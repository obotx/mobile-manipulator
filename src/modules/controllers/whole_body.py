import numpy as np
import modern_robotics as mr


class WholeBodyControl():
    def __init__(self) -> None:
        self.R = 0.1   # wheel radius
        self.L = 0.2225 # forward-backward wheel distance
        self.W = 0.2045 # side-by-side wheel distance
        
        self.J1 = 0 # height of vertical link-1
        self.J2 = 0 # height of vertical link-2
        self.J3 = 0 # length of telescoping link
        self.J4 = 0 # angels turret
        
        # Config in (x, y, z, roll, pitch, yaw)
        self.mobile_frame = np.array([0, 0, 0.127, 0, 0, 0])              # mobile base to world frame
        self.arm_to_mobile = np.array([0.15, 0.15, 0.158566, 0, 0, 0])    # arm base to mobile base frame 
        self.ee_to_arm = np.array([[-0.2771, -0, 0.5948, 0, 0, 0]])       # end effector relative to arm 
        
        
def NextState(
        current_state:np.ndarray, 
        velocity:float, 
        dt:float, 
    ):
    
    # define matrix F to compute chassis planar twist Vb
    r = 0.1
    l = 0.2225
    w = 0.2045

    F = (r/4)*np.array([
        [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], 
        [1, 1, 1, 1], 
        [-1, 1, -1, 1]])

    velocity = np.c
 
    # update the joint angles
    updated_arm_angles = current_state[3:8] + velocity[4:]*dt
    updated_wheel_angles = current_state[-4:] + velocity[:4]*dt
    wheel_angle_increment = updated_wheel_angles - current_state[-4:]

    # define chassis planar twist
    Vb = F@wheel_angle_increment

    # define chassis planar twist in 6D
    Vb_6D = np.array([0, 0, Vb[0], Vb[1], Vb[2], 0])

    # T matrix realting new body frame pose to initial
    T_b_bnew = mr.MatrixExp6(mr.VecTose3(Vb_6D))

    # define change in coordinates relative to the body frame - d_qb
    w_bz = Vb[0]
    v_bx = Vb[1]
    v_by = Vb[2]
    d_theata_b = w_bz
    if w_bz < 1e-3:
        d_x_b = v_bx
        d_y_b = v_by
    else:
        d_x_b = v_bx*np.sin(w_bz)+v_by*(np.cos(w_bz)-1)/w_bz
        d_y_b = v_by*np.sin(w_bz)+v_bx*(-np.cos(w_bz)+1)/w_bz
    d_qb = np.array([d_theata_b, d_x_b, d_y_b])

    # transform d_qb to space frame - d_qs
    chassis_angle = current_state[0]
    rot_mat = np.array([[1, 0, 0], [0, np.cos(chassis_angle), -np.sin(chassis_angle)],[0, np.sin(chassis_angle), np.cos(chassis_angle)]])
    d_qs = rot_mat@d_qb

    # define updated chassis config
    chassis_current_state = np.array(current_state[:3])
    chassis_updated_config = chassis_current_state + d_qs

    # form updated_config list in the right order
    updated_config = []
    for item in chassis_updated_config:
        updated_config.append(item)
    for item in updated_arm_angles:
        updated_config.append(item)
    for item in updated_wheel_angles:
        updated_config.append(item)

    return np.array(updated_config)
