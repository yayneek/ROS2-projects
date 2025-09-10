import numpy as np
import matplotlib.pyplot as plt

def poly(t, tau):
    s = 126*(t/tau)**5 - 420*(t/tau)**6 + 540*(t/tau)**7 - 315*(t/tau)**8 + 70*(t/tau)**9
    s_dot = (126*5*(t/tau)**4 - 420*6*(t/tau)**5 + 540*7*(t/tau)**6 - 315*8*(t/tau)**7 + 70*9*(t/tau)**8) * (1/tau)
    s_ddot = (126*5*4*(t/tau)**3 - 420*6*5*(t/tau)**4 + 540*7*6*(t/tau)**5 - 315*8*7*(t/tau)**6 + 70*9*8*(t/tau)**7) * (1/tau)**2
    return s, s_dot, s_ddot

def poly_trajectory(q_0, q_f, t_0, t_k, dt):
    """
    q_0, q_f : array-like, shape (6,) initial and final joint positions
    returns:
      q: (N,6) positions over time
      v: (N,6) velocities
      a: (N,6) accelerations
      t: (N,) time vector
    """
    q0 = np.asarray(q_0).reshape(-1)   # (6,)
    qf = np.asarray(q_f).reshape(-1)   # (6,)
    if q0.shape[0] != 6 or qf.shape[0] != 6:
        raise ValueError("q_0 and q_f must be length 6")

    tau = t_k - t_0
    t = np.arange(t_0, t_k, dt)         # (N,)
    s, s_dot, s_ddot = poly(t, tau)     # each (N,)

    #Shape (1, N)
    s = s[np.newaxis, :]
    s_dot = s_dot[np.newaxis, :]
    s_ddot = s_ddot[np.newaxis, :]

    delta_q = (qf - q0).reshape(6, 1)   # (6,1)

    #Broadcasting -> (6, N), then transpose -> (N,6)
    q = (delta_q @ s).T
    v = (delta_q @ s_dot).T
    a = (delta_q @ s_ddot).T

    #Add offset q0 so trajectory starts at q0
    q = q + q0.reshape(1, 6)

    return q, v, a, t
