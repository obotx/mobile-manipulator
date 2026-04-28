import numpy as np
import mujoco


def mat2quat(mat):
    res = np.zeros(4)
    mujoco.mju_mat2Quat(res, mat)
    return res


def negQuat(q):
    res = np.zeros(4)
    mujoco.mju_negQuat(res, q)
    return res


def mulQuat(q1, q2):
    res = np.zeros(4)
    mujoco.mju_mulQuat(res, q1, q2)
    return res

def rotVecQuat(vec, quat):
    res = np.zeros(3)
    mujoco.mju_rotVecQuat(res, vec, quat)
    return res

def mulPose(p1, q1, p2, q2):
    pres = np.zeros(3)
    qres = np.zeros(4)
    mujoco.mju_mulPose(pres, qres, p1, q1, p2, q2)
    return pres, qres

def negPose(p, q):
    pres = np.zeros(3)
    qres = np.zeros(4)
    mujoco.mju_negPose(pres, qres, p, q)
    return pres, qres
