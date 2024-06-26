import argparse
import cvxpy as cp
import matplotlib.pyplot as plt
import numpy as np
import osqp
import scipy.sparse as sp
from scipy.optimize import minimize


# Function to check if a point is at least min_distance from all other points
def is_valid_point(x, y, x_points, y_points, min_distance):
    for i in range(len(x_points)):
        if np.sqrt((x - x_points[i])**2 + (y - y_points[i])**2) <= min_distance:
            return False
    return True

def is_in_exclusion_zone(x,y, excluded_xmin, excluded_xmax, excluded_ymin, excluded_ymax):
    return excluded_xmin <= x <= excluded_xmax and excluded_ymin <= y <= excluded_ymax

def genQP(n=3, write=False):
    p0_solutions = []
    plot = True
    codegeneration_osqp = False
    cvxpy = True
    # Define the exclusion square in the middle
    excluded_xmin, excluded_xmax = -0.001, 0.0001
    excluded_ymin, excluded_ymax = -0.001, 0.0001

    # Define the limits
    xmin, xmax = -1.0, 1.0 
    ymin, ymax = -1.0, 1.0
    radius = 0.1
    li = [0.7, 0.2, 0.3]
    min_distance = sum(li)
    # Generate random points outside the exclusion zone
    x_points = []
    y_points = []
    pi = []
    pi = [
        [0.7, 0],
        [-0.2, 0.2],
        [-0.3, -0.2]
    ]
    x_points = [p[0] for p in pi]
    y_points = [p[1] for p in pi]
    ## if pi is not set then this will generate random points
    if len(pi) == 0:
        while len(x_points) < n:
            x = np.random.uniform(xmin, xmax)
            y = np.random.uniform(ymin, ymax)
            
            if not is_in_exclusion_zone(x, y, excluded_xmin, excluded_xmax, excluded_ymin, excluded_ymax):
                if is_valid_point(x, y, x_points, y_points, min_distance):
                    x_points.append(x)
                    y_points.append(y)
                    pi.append([x,y])
    if cvxpy:
        p0 = cp.Variable(2,)
        constraints = []
        pi = np.array(pi)
        cost = 0 
        for k, l in enumerate(li):
            cost += cp.QuadForm(np.array(pi[k]) - p0, np.eye(2,)) - l

        # print(cost)
        problem_cvxpy = cp.Problem(cp.Minimize(0.5*cost))
        solver = "OSQP"
        problem_cvxpy.solve(solver=solver)

        ## print the osqp matrices
        data, chain, inverse_data = problem_cvxpy.get_problem_data(solver)
        for key, values in data.items():
            print(key, "\n", values)
        p0_res = p0.value
        p0_solutions.append(p0_res)
        ## set the problem matrices for osqp
        ## source: https://github.com/cvxgrp/cvxpy/blob/master/cvxpy/reductions/solvers/qp_solvers/osqp_qpif.py
        P = data["P"]
        q = data["q"]
        A = sp.vstack([data["A"], data["F"]]).tocsc()
        uA = np.concatenate((data["b"], data["G"]))
        lA = np.concatenate([data["b"], -np.inf*np.ones(data["G"].shape)])
        problem_osqp = osqp.OSQP()
        settings = {"verbose": False}
        problem_osqp.setup(P=P,A=A, q=q, l=lA, u=uA, **settings)
        res = problem_osqp.solve()

        print("P: \n",P.toarray())
        print("A: \n",A.toarray())
        print("q: \n",A.toarray())
        print("l: \n",lA)
        print("u: \n",uA)
        print(pi.shape)
        print("points: \n", pi.reshape(2*pi.shape[0],))
        print("cvxpy: ",p0.value)
        print("osqp: ",res.x)
        print("objcvxpy: ",problem_cvxpy.value)
        print("objosqp: ",res.x)
        print()
        

    def cost_nn(p0, pi, li, p0_d, mu):
        cost = 0        
        for k, l in enumerate(li):
            cost += (np.linalg.norm(pi[k] - p0) - l)**2
        cost += mu* (np.linalg.norm(p0 - p0_d))**2
        print(cost)
        return cost 
    
    p0_d = np.array([0.0,0.0])
    mu = 0.0
    options= {"maxiter": 10}
    res = minimize(cost_nn, p0_d,args=(pi, li, p0_d, mu), options=options)
    print(res.x)
    p0_res = res.x
    p0_solutions.append(p0_res)
    if write:

        prob.codegen("src/generated",
            project_type='Makefile',
            parameters='matrices',
            python_ext_name='emosqp',
            force_rewrite=True,
            FLOAT=True,
            LONG=False)

    if plot:
    # if plot and p0.value is not None:
        # Plot the points and circles
        fig, ax = plt.subplots()
        i = 0
        for x, y in zip(x_points, y_points):
            # circle1 = plt.Circle((x, y), radius, color='b', alpha=0.2, label=f"robot radius_{i}")
            circle2 = plt.Circle((x, y), radius=li[i], color='g', alpha=0.2, label=f'cable range_{i} = {li[i]}')
            i+=1
            # ax.add_artist(circle1)
            ax.add_artist(circle2)
            ax.plot(x, y, 'bo')  # Plot the center point
        colors = ['pc', 'ro']
        solvers = ["QP","nonlinear"]
        for k,p0_res in enumerate(p0_solutions):
            ax.plot(p0_res[0], p0_res[1], colors[k], label=f"payload_{solvers[k]}", markersize=10)
        ax.legend()
        # Draw the exclusion zone
        plt.axvline(x=excluded_xmin, color='r', linestyle='--')
        plt.axvline(x=excluded_xmax, color='r', linestyle='--')
        plt.axhline(y=excluded_ymin, color='r', linestyle='--')
        plt.axhline(y=excluded_ymax, color='r', linestyle='--')

        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Random Points with Exclusion Zone and Shaded Circles')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

    if codegeneration_osqp: 
        prob.codegen("src/generated",
            project_type='Makefile',
            parameters='matrices',
            python_ext_name='emosqp',
            force_rewrite=True,
            FLOAT=True,
            LONG=False)
        

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robots', type=int, help="number of robots")
    parser.add_argument("-w", "--write", action="store_true") 
    args = parser.parse_args()
    genQP(n=args.robots, write=args.write)

if __name__=="__main__":
    main()
