import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d as interp
import scipy.fft
from scipy.ndimage import maximum_filter1d as max1d
import control as ctrl
import math

mp = .116
lp = .018
rw = .021
mw = .038
Iw = 8.38e-6
g = 9.81
R = 14
kb = .1343
kt = .0948

utx = lambda t, x0: [0, 0]

def sim(f, tf, dt, x0, utx):
    t = [0]
    x = [x0]
    u = [utx(0, x0)]
    while (t[-1] + dt < tf):
        t.append(t[-1] + dt)
        dx = f(t[-1], x[-1], utx(t[-1], x[-1])) * dt
        x.append(x[-1] + dx)
        u.append(utx(t[-1], x[-1]))
    
    t = np.array(t)
    u = np.array(u)
    x = np.array(x)

    return t, x, u

def func(t, x0, u):
    mass = np.array([[-1*mp*lp**2, mp*lp*np.cos(x0[2])],
    [mp*lp*np.cos(x0[2]), -1*(Iw/rw**2 + mw + mp)]])

    f = np.array([[(kt/R)*(u[1] + x0[3]*kb + x0[1]*kb/rw) - mp*lp*g*np.sin(x0[2])],
    [(kt/(R*rw))*(u[1] + x0[3]*kb + x0[1]*kb/rw) + x0[3]**2*mp*lp*np.sin(x0[2])]])

    mass_inv = np.linalg.inv(mass)
    accels = mass_inv @ f

    return np.array([x0[1], accels[1, 0], x0[3], accels[0, 0]])

x0 = [0, 0, np.pi/2, 0]
print(x0[1])

t, x, u = sim(func, 10, 1e-3, x0, utx)

q = x[:, [0, 2]]
print(x.shape)
print(q.shape)

plt.plot(t, q[:, 0], label='angular position')
plt.plot(t, q[:, 1], label='horizontal displacement')
plt.legend()
plt.xlabel('time')
plt.title('position over time')
plt.grid()

from matplotlib import animation
fig = plt.figure(figsize=(8,4)); plt.axis('equal')

plt.xlim(-8*lp,+8*lp); plt.ylim(-4*lp,+4*lp)
plt.xticks([]); plt.yticks([])
plt.plot([-10*lp,+10*lp],[0,0],'k',lw=10)

line_p = plt.plot([0,0],[0,2*lp],lw=10,color='purple')[0]
line_w = plt.plot(0,0,'.',ms=55,color='gold')[0]

plt.tight_layout()

def update(t_):
    i = (t >= t_).nonzero()[0][0]
    h,a = q[i]
    z_w = h; z_p = z_w + np.exp(1j*a)*(0 + 2j*lp)
    line_p.set_data([z_w.real,z_p.real],[z_w.imag,z_p.imag])
    line_w.set_data([z_w.real],[z_w.imag])

ani = animation.FuncAnimation(fig=fig, func=update,
                frames=np.arange(t[0],t[-1],.02), interval=20)

plt.show()
plt.xlim(-50*lp, +50*lp)

# from IPython.display import HTML
# HTML(ani.to_jshtml())

## ----------- PART 2 -----------

"""
approximate derivative via finite-central-differences
input:
g - function - g : R^n -> R^m
y - n array
(optional)
d - scalar - finite differences displacement parameter
output:
Dg(y) - m x n - approximation of Jacobian of g at y
"""
dt = 1e-4
def ss_sim(tf, x0, mat, K):
    t = 0
    t_arr = [0]
    x_arr = [x0]
    initial_u = (-1 * K @ x_arr[0]).item()
    u_arr = [initial_u]
    while t < tf:
        t += dt
        t_arr.append(t)
        x_ = x_arr[-1] + (mat @ x_arr[-1]) * dt
        x_arr.append(x_)
        u_arr.append((-1 * K @ x_arr[-1]).item())
    x_arr = np.array(x_arr)
    t_arr = np.array(t_arr)
    u_arr = np.array(u_arr)
    return t_arr, x_arr, u_arr

def Jacobian(g,y,d=1e-4):
    e = np.identity(len(y))
    Dyg = []
    for j in range(len(y)):
        Dyg.append((.5/d)*(g(y+d*e[j]) - g(y-d*e[j])))
    return np.array(Dyg).T

f_x = lambda x0: func(0, x0, [0 , 0])

f_v = lambda v0: func(0, [0, 0, 0, 0], v0)

A = Jacobian(f_x, [0, 0, 0, 0], 1e-4)
B = Jacobian(f_v, [0, 0], 1e-4)

print(A)
B = B[:, 1:]
print(B)
Q = np.diag([1, 0, 0, .02]); R = .1
K, S, E = ctrl.lqr(A, B, Q, R)

print(K)

utx = lambda t, x0: [0, (-1 * K @ x0).item()]

mat = A - B @ K
time_linear, linear_data, duty = ss_sim(5, x0, mat, K)
time_nl, nl_data, nl_duty = sim(func, 5, 1e-5, x0, utx)

print(K)
print(-1 * K)

plt.clf()
plt.plot(time_linear, linear_data[:,2], label='Angular Position w/controller in Linearized System')
plt.plot(time_nl, nl_data[:, 2], label='Angular Position w/Controller in non-linear system')
plt.grid()
plt.xlabel('time')
plt.ylabel('angle (rads)')
plt.title('Angular position of controlled feedback loop vs time')
plt.show()




