import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
from cvxpygen import cpg



# Function to check if a point is at least min_distance from all other points
def is_valid_point(x, y, x_points, y_points):
    for i in range(len(x_points)):
        if np.sqrt((x - x_points[i])**2 + (y - y_points[i])**2) <= min_distance:
            return False
    return True

def is_in_exclusion_zone(x,y):
    return exclusion_xmin <= x <= exclusion_xmax and exclusion_ymin <= y <= exclusion_ymax


plot = True
# Define the exclusion square in the middle
exclusion_xmin, exclusion_xmax = -0.5, 0.5
exclusion_ymin, exclusion_ymax = -0.5, 0.5

# Define the limits
xmin, xmax = -2, 2 
ymin, ymax = -2, 2
n = 3
min_distance = 1.0
# li = [0.5,0.5,0.5]
li = 0.4*np.ones(n)
li[2] = 0.7
# li[2] = 1.2

# Generate random points outside the exclusion zone
x_points = []
y_points = []
pi = []
### EXAMPLE TO TRY #### uncomment this
# pi = [
#     [-0.707,  0.707], 
#     [ 0.0, 0.0],
#     [0.707,  -0.707],
#     ]
x_points = [p[0] for p in pi ]
y_points = [p[1] for p in pi ]
if len(pi) == 0:
    while len(x_points) < n:
        x = np.random.uniform(xmin, xmax)
        y = np.random.uniform(ymin, ymax)
        
        if not is_in_exclusion_zone(x, y):
            if is_valid_point(x, y, x_points, y_points):
                x_points.append(x)
                y_points.append(y)
                pi.append([x,y])


p0 = cp.Variable(2,)
constraints = []
pi = np.array(pi)
cost = 0 
for k, l in enumerate(li):
    cost += cp.QuadForm(np.array(pi[k]) - p0, np.eye(2,))#  + cp.norm(np.array(pi[k]) - p0, 2) - l

print(cost)
problem = cp.Problem(cp.Minimize(cost))
problem.solve(solver=cp.SCS)
print(p0.value)



if plot and p0.value is not None:
    # Plot the points and circles
    fig, ax = plt.subplots()
    i = 0
    for x, y in zip(x_points, y_points):
        circle = plt.Circle((x, y), li[i], color='b', alpha=0.3)
        i+=1
        ax.add_artist(circle)
        ax.plot(x, y, 'bo')  # Plot the center point
    ax.plot(p0.value[0], p0.value[1], 'ro')

    # Draw the exclusion zone
    plt.axvline(x=exclusion_xmin, color='r', linestyle='--')
    plt.axvline(x=exclusion_xmax, color='r', linestyle='--')
    plt.axhline(y=exclusion_ymin, color='r', linestyle='--')
    plt.axhline(y=exclusion_ymax, color='r', linestyle='--')

    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Random Points with Exclusion Zone and Shaded Circles')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

cpg.generate_code(problem, code_dir='../codegen', solver='SCS')
