import cvxpy as cp
import jax.numpy as np  # Thinly-wrapped numpy
import numpy
from jax import jacfwd, jit

class SCP():
  def __init__(self, robot, collisionChecker=None):
    self.robot = robot
    self.collisionChecker = collisionChecker
    self.constructA = jit(jacfwd(robot.step, 0))
    self.constructB = jit(jacfwd(robot.step, 1))
    self.step = jit(robot.step)

  def min_xf(self,
    initial_x,
    initial_u,
    x0,
    xf,
    num_iterations=10, 
    trust_x = None,
    trust_u = None,
    verbose = False):

    X, U = [initial_x], [initial_u]

    xprev = initial_x
    uprev = initial_u
    T = xprev.shape[0]
    stateDim = xprev.shape[1]
    actionDim = uprev.shape[1]

    for _ in range(num_iterations):
      x = cp.Variable((T, stateDim))
      u = cp.Variable((T-1, actionDim))

      # set initial guesses for warm start
      x.value = xprev
      u.value = uprev

      # this objective helps to keep the control somewhat smooth,
      # otherwise, the output is spiky
      cp_objective = cp.Minimize(cp.sum_squares(u) + 1e6 * cp.norm(x[-1] - xf, "inf"))
      # cp_objective = cp.Minimize(cp.norm(x[-1] - xf, "inf"))

      constraints = [
        x[0] == x0, # initial state constraint
      ]

      # trust region
      if trust_x is not None:
        for t in range(0, T):
          constraints.append(
            cp.abs(x[t] - xprev[t]) <= trust_x
          )
      if trust_u is not None:
        for t in range(0, T-1):
          constraints.append(
            cp.abs(u[t] - uprev[t]) <= trust_u
          )

      # dynamics constraints
      for t in range(0, T-1):
        xbar = xprev[t]
        ubar = uprev[t]

        A = self.constructA(xbar, ubar)
        B = self.constructB(xbar, ubar)
        constraints.append(
          x[t+1] == self.step(xbar, ubar) + A @ (x[t] - xbar) + B @ (u[t] - ubar)
          )

      # bounds on u
      for t in range(0, T-1):
        constraints.extend([
          self.robot.min_u <= u[t],
          u[t] <= self.robot.max_u
          ])

      # bounds on x
      for t in range(0, T):
        constraints.extend([
          self.robot.min_x <= x[t],
          x[t] <= self.robot.max_x
          ])

      prob = cp.Problem(cp_objective, constraints)

      # The optimal objective value is returned by `prob.solve()`.
      try:
        # result = prob.solve(verbose=True, warm_start=True,solver=cp.GUROBI, BarQCPConvTol=1e-9)
        result = prob.solve(verbose=verbose, solver=cp.GUROBI)
        # result = prob.solve(verbose=True, warm_start=True, solver=cp.OSQP, max_iter=1000000)
      except cp.error.SolverError:
        # print("Warning: Solver failed!")
        return X, U, float('inf')
      except KeyError:
        # print("Warning BarQCPConvTol too big?")
        return X, U, float('inf')

      if 'optimal' not in prob.status:
        return X, U, float('inf')

      xprev = numpy.array(x.value, dtype=np.float32)
      uprev = numpy.array(u.value, dtype=np.float32)
      X.append(xprev)
      U.append(uprev)

      # print(xprev)
      # exit()
      max_error_to_goal = np.linalg.norm(xprev[-1] - xf, np.inf)
      # print("max error to goal: ", max_error_to_goal)

      if max_error_to_goal < 1e-5:
        return X, U, prob.value

    return X, U, float('inf')

  def min_u(self, initial_x, initial_u, 
             x0, xf,
             num_iterations=10,
             trust_x=None,
             trust_u=None,
             verbose=False,
             soft_xf=False):

    assert(initial_x.shape[0] == initial_u.shape[0] + 1)
    X, U = [initial_x], [initial_u]

    xprev = initial_x
    uprev = initial_u
    T = xprev.shape[0]
    stateDim = xprev.shape[1]
    actionDim = uprev.shape[1]

    # DEBUG
    if self.collisionChecker is not None:
        for t in range(0, T):
          # See 12a in "Convex optimization for proximity maneuvering of a spacecraft with a robotic manipulator"
          # Also used in GuSTO
          dist_tilde, p_obs, p_robot = self.collisionChecker.distance(
              xprev[t])
          if dist_tilde < 0:
            print("Warning: initial solution distance violation at t={}".format(t))

    for _ in range(num_iterations):
      x = cp.Variable((T, stateDim))
      u = cp.Variable((T-1, actionDim))

      # set initial guesses for warm start
      x.value = xprev
      u.value = uprev

      constraints = [
          x[0] == x0,  # initial state constraint
      ]

      if soft_xf:
        cp_objective = cp.Minimize(cp.sum_squares(u) + 1e6 * cp.norm(x[-1] - xf, "inf"))
      else:
        cp_objective = cp.Minimize(cp.sum_squares(u))
        constraints.append(x[-1] == xf)  # final state constraint

      # trust region
      if trust_x is not None:
        for t in range(0, T):
          constraints.append(
              cp.abs(x[t] - xprev[t]) <= trust_x
          )
      if trust_u is not None:
        for t in range(0, T-1):
          constraints.append(
              cp.abs(u[t] - uprev[t]) <= trust_u
          )

      # dynamics constraints
      for t in range(0, T-1):
        xbar = xprev[t]
        ubar = uprev[t]

        A = self.constructA(xbar, ubar)
        B = self.constructB(xbar, ubar)
        constraints.append(
            x[t+1] == self.step(xbar, ubar) + A @ (x[t] - xbar) + B @ (u[t] - ubar)
        )

      # bounds on u
      for t in range(0, T-1):
        constraints.extend([
            self.robot.min_u <= u[t],
            u[t] <= self.robot.max_u
        ])

      # bounds on x
      for t in range(0, T):
        constraints.extend([
            self.robot.min_x <= x[t],
            x[t] <= self.robot.max_x
        ])

      # collision constraints
      if self.collisionChecker is not None:
        for t in range(0, T):
          # See 12a in "Convex optimization for proximity maneuvering of a spacecraft with a robotic manipulator"
          # Also used in GuSTO
          dist_tilde, p_obs, p_robot = self.collisionChecker.distance(xprev[t])
          if dist_tilde > 0:
            d_tilde = p_robot - p_obs
          else:
            d_tilde = p_obs - p_robot


          norm_d_tilde = numpy.linalg.norm(d_tilde)
          if norm_d_tilde > 0:
            d_hat = d_tilde / norm_d_tilde

            constraints.extend([
              dist_tilde + d_hat[0:2].T @ (x[t,0:2] - xprev[t,0:2]) >= 0.0
            ])

      prob = cp.Problem(cp_objective, constraints)

      # The optimal objective value is returned by `prob.solve()`.
      try:
        # result = prob.solve(verbose=True, warm_start=True,solver=cp.GUROBI, BarQCPConvTol=1e-9)
        result = prob.solve(verbose=verbose, solver=cp.GUROBI)
        # result = prob.solve(verbose=True, warm_start=True, solver=cp.OSQP, max_iter=1000000)
      except cp.error.SolverError:
        # print("Warning: Solver failed!")
        return X, U, float('inf')
      except KeyError:
        # print("Warning BarQCPConvTol too big?")
        return X, U, float('inf')

      if 'optimal' not in prob.status:
        return X, U, prob.value

      # DEBUG
      if self.collisionChecker is not None:
        for t in range(0, T):
          # See 12a in "Convex optimization for proximity maneuvering of a spacecraft with a robotic manipulator"
          # Also used in GuSTO
          dist_tilde, p_obs, p_robot = self.collisionChecker.distance(x.value[t])
          if dist_tilde < 0:
            print("Warning: distance violation at t={} ({})".format(t, dist_tilde))

      xprev = numpy.array(x.value, dtype=np.float32)
      uprev = numpy.array(u.value, dtype=np.float32)
      X.append(xprev)
      U.append(uprev)

    return X, U, prob.value