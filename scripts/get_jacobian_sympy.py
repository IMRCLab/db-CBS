import sympy as sp
from sympy.printing  import ccode
# Define the symbolic variable
x1, y1, vx1, vy1 = sp.symbols('x1 y1 vx1 vy1', real=True)
x2, y2, vx2, vy2 = sp.symbols('x2 y2 vx2 vy2', real=True)
ax1, ay1, ax2, ay2 = sp.symbols('ax1 ay1 ax2 ay2', real=True)

alpha = 0.5
# positions
p1 = sp.Matrix([x1, y1])
p2 = sp.Matrix([x2, y2])
# distance, Euclidean norm
d = (p1-p2)
d_norm = d.norm()
# a repulsive
a_repulsive = (alpha / d_norm**3)*(p2-p1)
# x_dot = f(x,u)
x_dot = sp.Matrix([[vx1], 
                   [vy1], 
                   [ax1 + a_repulsive[0]], 
                   [ay1 + a_repulsive[1]], 
                   [vx2], 
                   [vy2], 
                   [ax2 - a_repulsive[0]], 
                   [ay2 - a_repulsive[1]]])
# state
x = sp.Matrix([x1, y1, vx1, vy1, x2, y2, vx2, vy2])
# action
u = sp.Matrix([ax1, ay1, ax2, ay2])
# compute the Jacobians
Jv_x = x_dot.jacobian(x)
Jv_u = x_dot.jacobian(u)
# convert into cpp 
cpp_code_Jvx = ccode(Jv_x)
cpp_code_Jvu = ccode(Jv_u)
# save into file
file_path = 'jacobian.txt'
# Export the C++ code to a file
with open(file_path, 'w') as file:
    file.write(cpp_code_Jvx)
    file.write('\n')
    file.write(cpp_code_Jvu)






