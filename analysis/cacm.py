import numpy as np

def cl(weight, rho, v, s=9812.2):
    return 2 * weight / (rho * v**2 * s)

def cd(weight, rho, v, s=9812.2, cd0=0.017, e=0.88, ar=10.112):
    return 0.5 * cd0 + cl(weight, rho, v, s)**2 / (np.pi * e * ar)

def cl_cd_ratio(weight_full, weight_empty, rho, v, s=9812.2, cd0=0.017, e=0.88, ar=10.112):
    return 0.5 * (cl(weight_full, rho, v, s)**(1/2)/cd(weight_full, rho, v, s, cd0, e, ar) + (cl(weight_empty, rho, v, s)**(1/2)/cd(weight_empty, rho, v, s, cd0, e, ar)))


def cruise_range(weight_full, weight_empty, rho, v, s=9812.2, tsfc=0.455, cd0=0.017, e=0.88, ar=10.112):
    return (2/tsfc) * (2/(rho*s))**(1/2) * cl_cd_ratio(weight_full, weight_empty, rho, v, s, cd0, e, ar) * ((weight_full)**(1/2)-(weight_full-weight_empty)**(1/2))


