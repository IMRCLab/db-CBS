import jax.numpy as np
from jax import lax

def normalize_angle(angle):
	return (angle + np.pi) % (2 * np.pi) - np.pi

def diff_angle(angle1, angle2):
	return np.arctan2(np.sin(angle1-angle2), np.cos(angle1-angle2))

# Quaternion routines adapted from rowan to use autograd


def qmultiply(q1, q2):
	return np.concatenate((
		np.array([q1[0] * q2[0] - np.sum(q1[1:4]*q2[1:4])]),
		q1[0] * q2[1:4] + q2[0] * q1[1:4] + np.cross(q1[1:4], q2[1:4])))


def qconjugate(q):
	return np.concatenate((q[0:1], -q[1:4]))


def qrotate(q, v):
	quat_v = np.concatenate((np.array([0]), v))
	return qmultiply(q, qmultiply(quat_v, qconjugate(q)))[1:]


def qexp_regular_norm(q):
	e = np.exp(q[0])
	norm = np.linalg.norm(q[1:4])
	result_v = e * q[1:4] / norm * np.sin(norm)
	result_w = e * np.cos(norm)
	return np.concatenate((np.array([result_w]), result_v))


def qexp_zero_norm(q):
	e = np.exp(q[0])
	result_v = np.zeros(3)
	result_w = e
	return np.concatenate((np.array([result_w]), result_v))


def qexp(q):
	return lax.cond(np.allclose(q[1:4], 0), q, qexp_zero_norm, q, qexp_regular_norm)


def qintegrate(q, v, dt):
	quat_v = np.concatenate((np.array([0]), v*dt/2))
	return qmultiply(qexp(quat_v), q)


def qnormalize(q):
	return q / np.linalg.norm(q)


class RobotUnicycleFirstOrder:

	def __init__(self, v_min, v_max, w_min, w_max):
		self.action_desc = ["v [m/s]", "w [rad/s]"]
		self.min_u = np.array([v_min, w_min])
		self.max_u = np.array([v_max, w_max])

		self.state_desc = ["x [m]", "y [m]", "yaw [rad]"]
		self.min_x = np.array([-np.inf, -np.inf, -np.pi])
		self.max_x = np.array([np.inf, np.inf, np.pi])

		self.dt = 0.1
		self.is2D = True

	def valid_state(self, state):
		return 	(state >= self.min_x).all() and \
				(state <= self.max_x).all()

	def step(self, state, action):
		x, y, yaw = state
		v, w = action

		yaw_next = yaw + w * self.dt
		yaw_next_norm = (yaw_next + np.pi) % (2 * np.pi) - np.pi
		x_next = x + v * np.cos(yaw_next_norm) * self.dt
		y_next = y + v * np.sin(yaw_next_norm) * self.dt
		# normalize yaw between -pi and pi

		state_next = np.array([x_next, y_next, yaw_next_norm])
		return state_next


class RobotUnicycleSecondOrder:

	def __init__(self, v_limit, w_limit, a_limit, w_dot_limit):
		self.action_desc = ["a [m^2/s]", "w_dot [rad^2/s]"]
		self.min_u = np.array([-a_limit, -w_dot_limit])
		self.max_u = np.array([a_limit, w_dot_limit])

		self.state_desc = ["x [m]", "y [m]", "yaw [rad]", "v [m/s]", "w [rad/s]"]
		self.min_x = np.array([-np.inf, -np.inf, -np.pi, -v_limit, -w_limit])
		self.max_x = np.array([np.inf, np.inf, np.pi, v_limit, w_limit])

		self.dt = 0.1
		self.is2D = True

	def valid_state(self, state):
		return 	(state >= self.min_x).all() and \
				(state <= self.max_x).all()

	def step(self, state, action):
		x, y, yaw, v, w = state
		a, w_dot = action

		# For compatibility with KOMO, update v and yaw first
		v_next = v + a * self.dt
		w_dot_next = w + w_dot * self.dt
		yaw_next = yaw + w_dot_next * self.dt
		yaw_next_norm = (yaw_next + np.pi) % (2 * np.pi) - np.pi
		x_next = x + v_next * np.cos(yaw_next) * self.dt
		y_next = y + v_next * np.sin(yaw_next) * self.dt

		# x_next = x + v * np.cos(yaw) * dt
		# y_next = y + v * np.sin(yaw) * dt
		# yaw_next = yaw + w * dt
		# normalize yaw between -pi and pi

		state_next = np.array([x_next, y_next, yaw_next_norm, v_next, w_dot_next])
		return state_next


# LaValle book, Equation 13.19
class RobotCarFirstOrderWithTrailers:

	def __init__(self, v_min, v_max, phi_min, phi_max, L, hitch_lengths):
		self.action_desc = ["v [m/s]", "steering angle [rad]"]
		self.min_u = np.array([v_min, phi_min])
		self.max_u = np.array([v_max, phi_max])

		self.state_desc = ["x [m]", "y [m]", "yaw [rad]"]
		min_x = [-np.inf, -np.inf, -np.pi]
		max_x = [np.inf, np.inf, np.pi]
		for k in range(len(hitch_lengths)):
			self.state_desc.append("trailer{} yaw [rad]".format(k+1))
			min_x.append(-np.pi)
			max_x.append(np.pi)
		self.min_x = np.array(min_x)
		self.max_x = np.array(max_x)

		self.L = L
		self.hitch_lengths = hitch_lengths
		self.dt = 0.1
		self.is2D = True

	def valid_state(self, state):
		# check if theta0 and theta1 have a reasonable relative angle
		dangle = diff_angle(state[2], state[3])

		return 	(state >= self.min_x).all() and \
				(state <= self.max_x).all() and \
				np.absolute(dangle) <= np.pi / 4

	def step(self, state, action):
		""""
		x_dot = v * cos (theta_0)
		y_dot = v * sin (theta_0)
		theta_0_dot = v / L * tan(phi)
		theta_1_dot = v / hitch_lengths[0] * sin(theta_0 - theta_1)
		...
		"""
		x, y, yaw = state[0], state[1], state[2]
		v, phi = action

		yaw_next = yaw + v / self.L * np.tan(phi) * self.dt
		# normalize yaw between -pi and pi
		yaw_next_norm = normalize_angle(yaw_next)
		x_next = x + v * np.cos(yaw_next) * self.dt
		y_next = y + v * np.sin(yaw_next) * self.dt

		state_next_list = [x_next, y_next, yaw_next_norm]

		for i, d in enumerate(self.hitch_lengths):
			theta_dot = v / d
			for j in range(0, i):
				theta_dot *= np.cos(state[2+j] - state[3+j])
			theta_dot *= np.sin(state[2+i] - state[3+i])

			theta = state[3+i]
			theta_next = theta + theta_dot * self.dt
			theta_next_norm = normalize_angle(theta_next)
			state_next_list.append(theta_next_norm)

		state_next = np.array(state_next_list)
		return state_next


class Quadrotor:

	def __init__(self):
		self.action_desc = ["f1 [N]", "f2 [N]", "f3 [N]", "f4 [N]"]
		self.min_u = np.zeros(4)
		self.max_u = np.ones(4) * 12.0 / 1000.0 * 9.81
		max_v = 2
		max_omega = 4

		self.state_desc = [
			"x [m]", "y [m]", "z [m]",
			"qx", "qy", "qz", "qw",
			"vx [m/s]", "vy [m/s]", "vz [m/s]",
			"w_x", "w_y", "w_z"]
		min_x = [-np.inf, -np.inf, -np.inf,
				-1.001, -1.001, -1.001, -1.001,
				-max_v, -max_v, -max_v,
				-max_omega, -max_omega, -max_omega]
		max_x = [np.inf, np.inf, np.inf,
				1.001, 1.001, 1.001, 1.001,
				max_v, max_v, max_v,
				max_omega, max_omega, max_omega]
		self.min_x = np.array(min_x)
		self.max_x = np.array(max_x)

		# parameters (Crazyflie 2.0 quadrotor)
		self.mass = 0.034  # kg
		# self.J = np.array([
		# 	[16.56,0.83,0.71],
		# 	[0.83,16.66,1.8],
		# 	[0.72,1.8,29.26]
		# 	]) * 1e-6  # kg m^2
		self.J = np.array([16.571710e-6, 16.655602e-6, 29.261652e-6])

		# Note: we assume here that our control is forces
		arm_length = 0.046  # m
		arm = 0.707106781 * arm_length
		t2t = 0.006  # thrust-to-torque ratio
		self.B0 = np.array([
                    [1, 1, 1, 1],
                 			[-arm, -arm, arm, arm],
                 			[-arm, arm, arm, -arm],
                 			[-t2t, t2t, -t2t, t2t]
                ])
		self.g = 9.81  # not signed

		if self.J.shape == (3, 3):
			self.inv_J = np.linalg.pinv(self.J)  # full matrix -> pseudo inverse
		else:
			self.inv_J = 1 / self.J  # diagonal matrix -> division

		self.dt = 0.01
		self.is2D = False

	def valid_state(self, state):
		return 	(state >= self.min_x).all() and \
				(state <= self.max_x).all()

	def step(self, state, action):
		# compute next state
		state = np.asarray(state)
		action = np.asarray(action)
		q = np.concatenate((state[6:7], state[3:6]))
		omega = state[10:]

		eta = np.dot(self.B0, action)

		f_u = np.array([0, 0, eta[0]])
		tau_u = np.array([eta[1], eta[2], eta[3]])

		# dynamics
		# dot{p} = v
		pos_next = state[0:3] + state[7:10] * self.dt
		# mv = mg + R f_u
		vel_next = state[7:10] + (np.array([0, 0, -self.g]) +
		                         qrotate(q, f_u) / self.mass) * self.dt

		# dot{R} = R S(w)
		# to integrate the dynamics, see
		# https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/, and
		# https://arxiv.org/pdf/1604.08139.pdf
		q_next = qnormalize(qintegrate(q, omega, self.dt))

		# mJ = Jw x w + tau_u
		omega_next = state[10:] + (self.inv_J *
		                           (np.cross(self.J * omega, omega) + tau_u)) * self.dt

		return np.concatenate((pos_next, q_next[1:4], q_next[0:1], vel_next, omega_next))

def create_robot(robot_type):
	if robot_type == "unicycle_first_order_0":
		return RobotUnicycleFirstOrder(-0.5, 0.5, -0.5, 0.5)
	elif robot_type == "unicycle_first_order_1":
		return RobotUnicycleFirstOrder(0.25, 0.5, -0.5, 0.5)
	elif robot_type == "unicycle_first_order_2":
		return RobotUnicycleFirstOrder(0.25, 0.5, -0.25, 0.5)
	elif robot_type == "unicycle_second_order_0":
		return RobotUnicycleSecondOrder(0.5, 0.5, 0.25, 0.25)
	elif robot_type == "car_first_order_0":
		return RobotCarFirstOrderWithTrailers(-0.1, 0.5, -np.pi/3, np.pi/3, 0.25, [])
	elif robot_type == "car_first_order_with_1_trailers_0":
		return RobotCarFirstOrderWithTrailers(-0.1, 0.5, -np.pi/3, np.pi/3, 0.25, [0.5])
	elif robot_type == "quadrotor_0":
		return Quadrotor()
	else:
		raise Exception("Unknown robot type {}!".format(robot_type))


if __name__ == '__main__':
	r = create_robot("quadrotor_0")

	# state = np.array([1, 1, 0.999959, -0.00603708, 0.00740579, -
	#                  0.000374226, 0.999954, 0, 0, -0.00819401, -2.41403, 2.96305, - 0.149642])
	# action = np.array([0.0755318, 0.0915615, 0.0796134, 0.0853427])

	eta = np.dot(r.B0, np.array([r.max_u[0], 0, r.max_u[0], 0]))

	f_u = np.array([0, 0, eta[0]])
	tau_u = np.array([eta[1], eta[2], eta[3]])

	print(f_u, tau_u)
