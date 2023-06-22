import yaml
import numpy as np

import sys, os
sys.path.append(os.getcwd())
from motionplanningutils import RobotHelper


class UtilsSolutionFile:
	def __init__(self, robot_type: str) -> None:
		self.rh = RobotHelper(robot_type)

	def load(self, filename: str) -> None:
		with open(filename) as f:
			self.file = yaml.safe_load(f)
		self.states = np.array(self.file['result'][0]['states'])
		if 'actions' in self.file['result'][0]:
			self.actions = np.array(self.file['result'][0]['actions'])

	def T(self) -> int:
		return self.states.shape[0] - 1

	def save_rescaled(self, filename:str, T: int) -> None:
		T_orig = self.T()
		state_dim = self.states.shape[1]
		states_interp = np.zeros((T+1, state_dim))
		# t_rescaled = np.linspace(0,1,T+1)
		# t_orig = np.linspace(0, 1, T_orig+1)
		for k in range(T+1):
			t_rescaled = k / T # in range [0,1]
			idx_orig = int(np.floor(t_rescaled * T_orig))
			idx_orig_next = int(np.ceil(t_rescaled * T_orig))
			t_orig = idx_orig / T_orig
			t_orig_next = idx_orig_next / T_orig
			if t_orig_next > t_orig:
				rel_t = (t_rescaled - t_orig) / (t_orig_next - t_orig)
			else:
				rel_t = 0
			# print(idx_orig, idx_orig_next, t_orig, t_orig_next, t_rescaled, rel_t)
			states_interp[k] = self.rh.interpolate(self.states[idx_orig], self.states[idx_orig_next], rel_t)
			# print(self.states[orig_idx], self.states[orig_idx_next], states_interp[t])

		# exit()

		# for k in range(state_dim):
			# states_interp[:,k] = np.interp(np.linspace(0,1,T+1), np.linspace(0, 1, T_orig+1), self.states[:,k])
		
		if 'actions' in self.file['result'][0]:
			action_dim = self.actions.shape[1]
			actions_interp = np.empty((T, action_dim))
			for k in range(self.actions.shape[1]):
				actions_interp[:,k] = np.interp(np.linspace(0,1,T), np.linspace(0, 1, T_orig), self.actions[:,k])

		with open(filename, 'w') as f:
			self.file['result'][0]['states'] = states_interp.tolist()
			if 'actions' in self.file['result'][0]:
				self.file['result'][0]['actions'] = actions_interp.tolist()
			yaml.dump(self.file, f)


def main():
	rh = RobotHelper("unicycle_first_order_0")
	a = [0,1,2] #rh.sampleUniform()
	b = [3,4,5] #rh.sampleUniform()
	c = rh.interpolate(a, b, 0.5)
	print(a,b,c)

if __name__ == '__main__':
	main()