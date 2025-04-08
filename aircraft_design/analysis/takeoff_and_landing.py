import numpy as np
import matplotlib.pyplot as plt

def tau(theta_f):
    return 1 - (theta_f - np.sin(theta_f))/np.pi


def delta_cl(delta,theta_f):
    return 2 * np.pi * delta * tau(theta_f)

def delta_zero_aoa(delta,theta_f):
    return delta * (1 - (theta_f - np.sin(theta_f)/np.pi))

def delta_cmac(delta,theta_f):
    return delta * (0.5 * np.sin(theta_f)*(np.cos(theta_f)-1))


def theta_f(xf0,delta):
    c = 1 + 2 * xf0**2 + 2 * xf0 * ((1-xf0)*np.cos(delta)-1)
    xf = xf0 * np.sqrt(1-((1-xf0)*np.sin(delta)/c)**2)
    cf_to_c = (c-xf) / c
    return np.arccos(2*cf_to_c-1)


# plot delta_cl vs delta
delta = np.linspace(np.radians(-40), np.radians(80), 50)
delta_cl = delta_cl(delta,theta_f(0.75,delta))

plt.plot(np.degrees(delta),delta_cl)
plt.plot(np.degrees(delta),theta_f(0.75,delta))
plt.plot(np.degrees(delta),tau(theta_f(0.75,delta)))
plt.xlabel(r'$\delta$')
plt.ylabel(r'$\Delta C_L$')
plt.savefig('delta_cl.png')

