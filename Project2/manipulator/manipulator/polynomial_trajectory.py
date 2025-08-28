import matplotlib.pyplot as plt
import numpy as np

def poly(t, tau):
    s = 126*(t/tau)**5 - 420*(t/tau)**6 + 540*(t/tau)**7 - 315*(t/tau)**8 + 70*(t/tau)**9
    s_dot = (126*5*(t/tau)**4 - 420*6*(t/tau)**5 + 540*7*(t/tau)**6 - 315*8*(t/tau)**7 + 70*9*(t/tau)**8)*(1/tau)
    s_ddot = (126*5*4*(t/tau)**3 - 420*6*5*(t/tau)**4 + 540*7*6*(t/tau)**5 - 315*8*7*(t/tau)**6 + 70*9*8*(t/tau)**7)*(1/tau)**2
    return s, s_dot, s_ddot

def poly_trajectory(q_0, q_f, t_0, t_k, dt):
    tau = t_k - t_0
    t = np.arange(t_0, t_k, dt)
    s, s_dot, s_ddot = poly(t, tau)
    q = (q_f-q_0) * s
    v = (q_f-q_0) * s_dot
    a = (q_f-q_0) * s_ddot
    return q, v, a, t

t_0 = 0
t_k = 2
dt = 0.01
q_f = 10
q_0 = 0
q, v, a, t = poly_trajectory(q_0,q_f,t_0,t_k,dt)
q_f = np.trapezoid(v, t)
v_k = np.trapezoid(a, t)
print(q_f, v_k)
plt.plot(t, q, t, v, t, a)
plt.show()





