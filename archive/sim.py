import pybullet as p
import pybullet_data
import time
import numpy as np
from scipy import linalg

# ===============================
# Parameters
# ===============================
use_lqr = True  # True → LQR / False → PID
dt = 1.0/240.0
g = 9.81
m_cart = 1.0
m_pole = 0.1
l = 1.0  # length of pole
max_force = 10.0

# PID gains (for pendulum angle control)
Kp = 100
Kd = 20
Ki = 0

# ===============================
# PyBullet Setup
# ===============================
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -g)

planeId = p.loadURDF("plane.urdf")

# Cartpole URDF (simple)
cartpole = p.loadURDF("cartpole.urdf", [0, 0, 0], useFixedBase=False)
cart_id = 0
pole_joint = 0  # revolute joint
cart_joint = 1  # prismatic joint

# Reset initial state
p.resetJointState(cartpole, pole_joint, 0.1, 0)  # small perturbation
p.resetJointState(cartpole, cart_joint, 0.0, 0)

# ===============================
# LQR Gain Calculation
# ===============================
def lqr(A, B, Q, R):
    """Solve the continuous time LQR controller."""
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K

# Linearized dynamics
A = np.array([[0, 1, 0, 0],
              [g/l, 0, 0, 0],
              [0, 0, 0, 1],
              [-g/l, 0, 0, 0]])

B = np.array([[0], [1/(m_pole*l*l)], [0], [1/(m_pole*l*l)]])

Q = np.diag([10, 1, 1, 1])
R = np.array([[0.1]])

K_lqr = lqr(A, B, Q, R)
print("LQR Gain:", K_lqr)

# ===============================
# Simulation Loop
# ===============================
integral_error = 0.0

for step in range(5000):
    # Read state
    pole_angle, pole_vel = p.getJointState(cartpole, pole_joint)[0:2]
    cart_pos, cart_vel = p.getJointState(cartpole, cart_joint)[0:2]
    
    # Wrap angle to [-pi, pi]
    pole_angle = ((pole_angle + np.pi) % (2*np.pi)) - np.pi
    
    # =======================
    # Control
    # =======================
    if use_lqr:
        x = np.array([pole_angle, pole_vel, cart_pos, cart_vel])
        u = -K_lqr @ x
        u = float(np.clip(u, -max_force, max_force))
    else:
        error = 0 - pole_angle
        integral_error += error * dt
        derivative = -pole_vel
        u = Kp * error + Ki * integral_error + Kd * derivative
        u = np.clip(u, -max_force, max_force)
    
    # Apply force to cart joint
    p.setJointMotorControl2(
        cartpole,
        cart_joint,
        controlMode=p.TORQUE_CONTROL,
        force=u
    )
    
    p.stepSimulation()
    time.sleep(dt)