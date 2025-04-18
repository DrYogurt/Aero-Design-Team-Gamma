import numpy as np
from scipy.signal import lti, step
import matplotlib.pyplot as plt

def aircraft_longitudinal_dynamics(V0, a, b, tpar, eta, tau, ku, kw, kq, ko,
                                   prints = True,
                                   plots = True,
                                   for_optim = False,
                                  ):
    """
    This function replicates the logic of the given MATLAB script for aircraft longitudinal dynamics,
    but takes parameters directly as input arguments rather than reading from files.
    It prints intermediate results, characteristic equations, factorization, transfer functions,
    and produces plots of the responses.

    Parameters
    ----------
    V0 : float
        Trim flight speed (ft/s)
    a : np.ndarray shape (4,4)
        Aerodynamic stability derivatives matrix
        Format (following the MATLAB code indexing):
         a(1,1)=xu a(1,2)=zu a(1,3)=mu a(1,4)=???  
         a(2,1)=xw a(2,2)=zw a(2,3)=mw a(2,4)=???
         a(3,1)=xq a(3,2)=zq a(3,3)=mq a(3,4)=???
         a(4,1)=xo a(4,2)=zo a(4,3)=mo a(4,4)=1.0 (generally)

    b : np.ndarray shape (4,2)
        Control derivatives matrix
        Format:
         b(1,1)=xn b(1,2)=zn
         b(2,1)=xt b(2,2)=zt
         b(3,1)=mn b(3,2)=mt
         b(4,1)=0  b(4,2)=0

    tpar : list or array of length 3
        [t_start, t_step, t_end] for time vector
    eta : float
        Elevator deflection step input (rad)
    tau : float
        Engine thrust increment step input
    ku, kw, kq, ko : array-like of length 2
        Feedback gains for axial velocity, normal velocity, pitch rate, and pitch attitude.
    """
    a = a.T
    b = b.T
    # Unpack the aerodynamic and control derivatives
    xu, zu, mu = a[0,0], a[0,1], a[0,2]
    xw, zw, mw = a[1,0], a[1,1], a[1,2]
    xq, zq, mq = a[2,0], a[2,1], a[2,2]
    xo, zo, mo = a[3,0], a[3,1], a[3,2]

    xn, zn = b[0,0], b[0,1]
    xt, zt = b[1,0], b[1,1]
    mn, mt = b[2,0], b[2,1]
    # b[3,:] are zero

    # Print input data as in the MATLAB script style
    if prints:
        print("Aircraft Longitudinal Dynamics\n")
        print(f"  {V0:5.1f} :V0 (ft/s) level cruise")
        print(" --- longitudinal aerodynamic stability derivatives ---")
        for row in a:
            print(f"   {row[0]:10.5f} {row[1]:10.5f} {row[2]:10.5f} {row[3]:10.5f}")
        print(" --- control derivatives ---")
        print(f"   {b[0,0]:10.5f} {b[0,1]:10.5f}")
        print(f"   {b[1,0]:10.5f} {b[1,1]:10.5f}")
        print(f"   {b[2,0]:10.5f} {b[2,1]:10.5f}")
        print(f"   {b[3,0]:10.5f} {b[3,1]:10.5f}")
        print(" --- time array --- ")
        print(f"  {tpar[0]} {tpar[1]} {tpar[2]} :i.e. t=({tpar[0]}:{tpar[1]}:{tpar[2]})")
        print(" --- step inputs ---")
        print(f"   {eta:10.4f} :elevator deflection, eta (rad) [1 deg = 0.0175 rad]")
        print(f"   {tau:10.4f} :engine thrust increment, tau")
        print(" --- feedback control ---")
        print(f"   {ku[0]:4.1f}    {ku[1]:4.1f}  :axial velocity gains (eta tau)")
        print(f"   {kw[0]:4.1f}    {kw[1]:4.1f}  :normal velocity gains")
        print(f"   {kq[0]:4.1f}    {kq[1]:4.1f}  :pitch rate gains")
        print(f"   {ko[0]:4.1f}    {ko[1]:4.1f}  :pitch attitude gains")
        print(" --- end ---\n")

    
    # Apply feedback modifications
    xu = xu - ku[0]*xn - ku[1]*xt
    zu = zu - ku[0]*zn - ku[1]*zt
    mu = mu - ku[0]*mn - ku[1]*mt

    xw = xw - kw[0]*xn - kw[1]*xt
    zw = zw - kw[0]*zn - kw[1]*zt
    mw = mw - kw[0]*mn - kw[1]*mt

    xq = xq - kq[0]*xn - kq[1]*xt
    zq = zq - kq[0]*zn - kq[1]*zt
    mq = mq - kq[0]*mn - kq[1]*mt

    xo = xo - ko[0]*xn - ko[1]*xt
    zo = zo - ko[0]*zn - ko[1]*zt
    mo = mo - ko[0]*mn - ko[1]*mt

    # Print modified [A] and [B]
    if prints:
        print("[A]:")
        print(f"{xu:12.4e}{zu:12.4e}{mu:12.4e}{xo:12.4e}")
        print(f"{xw:12.4e}{zw:12.4e}{mw:12.4e}{zo:12.4e}")
        print(f"{xq:12.4e}{zq:12.4e}{mq:12.4e}{mo:12.4e}")
        print(f"{0.0:12.4e}{0.0:12.4e}{1.0:12.4e}{0.0:12.4e}")
        print("[B]:")
        print(f"{xn:12.4e}{zn:12.4e}")
        print(f"{xt:12.4e}{zt:12.4e}")
        print(f"{mn:12.4e}{mt:12.4e}")
        print(f"{0.0:12.4e}{0.0:12.4e}")
    
    # Characteristic equation
    a0 = 1.0
    b0 = -(mq + xu + zw)
    c0 = (mq*zw - mw*zq) + (mq*xu - mu*xq) + (xu*zw - xw*zu) - mo
    d0 = (mo*xu - mu*xo) + (mo*zw - mw*zo) + mq*(xw*zu - xu*zw) \
         + xq*(mu*zw - mw*zu) + zq*(mw*xu - mu*xw)
    e0 = mo*(xw*zu - xu*zw) + xo*(mu*zw - mw*zu) + zo*(mw*xu - mu*xw)

    den = [a0, b0, c0, d0, e0]
    if prints:
        print("\nCharacteristic Equation: as^4+bs^3+cs^2+ds+e=0")
        print(f"{a0:12.4e}s^4{b0:+12.4e}s^3{c0:+12.4e}s^2{d0:+12.4e}s{e0:+12.4e} =0\n")

    # Approximate factorization
    b11 = (c0*d0 - b0*e0)/(c0*c0)
    c11 = e0/c0
    b12 = b0/a0
    c12 = c0/a0
    if prints:
        print("Approximate Factorization: (as^2+bs+c)(as^2+bs+c)=0")
        print(f"\t(s^2{b11:+12.4e}s{c11:+12.4e})(s^2{b12:+12.4e}s{c12:+12.4e})=0")

    pnfa = np.sqrt(c11)
    pdra = 0.5*b11/pnfa
    snfa = np.sqrt(c12)
    sdra = 0.5*b12/snfa
    if prints:
        print("Phugoid Stability Mode:")
        print(f"{pdra:12.4e} :damping ratio")
        print(f"{pnfa:12.4e} :undamped natural frequency")
        print("Short-Period Stability Mode:")
        print(f"{sdra:12.4e} :damping ratio")
        print(f"{snfa:12.4e} :undamped natural frequency\n")

    # Exact roots
    rd = np.roots(den)

    def cfmt(z):
        # Complex root formatting: real+imag i
        return f"{z.real:+.5f}{z.imag:+.5f}i" if abs(z.imag)>1e-14 else f"{z.real:+.5f}"
    if prints:
        print("Exact Roots of Characteristic Equation:")
        print(" " + "       ".join([cfmt(z) for z in rd]))

    # Based on the original code logic, assume first two roots form one pair, last two form another pair.
    # Sort by magnitude of imaginary part to identify short-period (high freq) vs phugoid (low freq)
    rd_sp = sorted(rd, key=lambda x: abs(x.imag), reverse=True)[:2]
    rd_ph = sorted(rd, key=lambda x: abs(x.imag))[:2]

    # Compute exact damping ratios and frequencies from factored roots
    r_sp1, r_sp2 = rd_sp
    r_ph1, r_ph2 = rd_ph

    snf = np.sqrt(r_sp1*r_sp2)
    sdr = -0.5*(r_sp1+r_sp2)/snf
    pnf = np.sqrt(r_ph1*r_ph2)
    pdr = -0.5*(r_ph1+r_ph2)/pnf
    if prints:
        print("Factored Characteristic Equation:")
        print(f"\t(s^2{2*pdr*pnf:+12.4e}s{(pnf*pnf):+12.4e})(s^2{2*sdr*snf:+12.4e}s{(snf*snf):+12.4e})=0")
        print("Phugoid Stability Mode:")
        print(f"{pdr:12.4e} :damping ratio")
        print(f"{pnf:12.4e} :undamped natural frequency")
        print("Short-Period Stability Mode:")
        print(f"{sdr:12.4e} :damping ratio")
        print(f"{snf:12.4e} :undamped natural frequency\n")
    
        print("Aircraft Response Transfer Functions: =N(s)/D(s)")
        print(f"D(s)={a0:12.4e}s^4{b0:+12.4e}s^3{c0:+12.4e}s^2{d0:+12.4e}s{e0:+12.4e}\n")

    # Elevator response transfer functions
    a1 = xn
    b1 = mn*xq - xn*(mq+zw) + zn*xw
    c1 = mn*(xw*zq - xq*zw + xo) + xn*(mq*zw - mw*zq - mo) + zn*(mw*xq - mq*xw)
    d1 = mn*(xw*zo - xo*zw) + xn*(mo*zw - mw*zo) + zn*(mw*xo - mo*xw)
    num1 = [a1,b1,c1,d1]
    
    if prints:
        print("Axial Velocity Response to Elevator Deflection:")
        print(f"N(s)={num1[0]:12.4e}s^3{num1[1]:+12.4e}s^2{num1[2]:+12.4e}s{num1[3]:+12.4e}")
        rn1 = np.roots(num1)
        print("    roots= " + "    ".join([cfmt(r) for r in rn1]))

    a2 = zn
    b2 = mn*zq + xn*zu - zn*(mq+xu)
    c2 = mn*(xq*zu - xu*zq + zo) + xn*(mu*zq - mq*zu) + zn*(mq*xu - mu*xq - mo)
    d2 = mn*(xo*zu - xu*zo) + xn*(mu*zo - mo*zu) + zn*(mo*xu - mu*xo)
    num2 = [a2,b2,c2,d2]

    if prints:
    
        print("Normal Velocity Response to Elevator Deflection:")
        print(f"N(s)={num2[0]:12.4e}s^3{num2[1]:+12.4e}s^2{num2[2]:+12.4e}s{num2[3]:+12.4e}")
        rn2 = np.roots(num2)
        print("    roots= " + "    ".join([cfmt(r) for r in rn2]))

    a3 = mn
    b3 = -mn*(xu+zw) + xn*mu + zn*mw
    c3 = mn*(xu*zw - xw*zu) + xn*(mw*zu - mu*zw) + zn*(mu*xw - mw*xu)
    num3 = [a3,b3,c3,0.0]
    
    if prints:
        print("Pitch Rate Response to Elevator Deflection:")
        print(f"N(s)={num3[0]:12.4e}s^3{num3[1]:+12.4e}s^2{num3[2]:+12.4e}s{0.0:+12.4e}")
        rn3 = np.roots(num3)
        print("    roots= " + "    ".join([cfmt(r) for r in rn3]))

    a4 = mn
    b4 = -mn*(xu+zw) + xn*mu + zn*mw
    c4 = mn*(xu*zw - xw*zu) + xn*(mw*zu - mu*zw) + zn*(mu*xw - mw*xu)
    num4 = [a4,b4,c4]

    if prints:
        print("Pitch Angle Response to Elevator Deflection:")
        print(f"N(s)={num4[0]:12.4e}s^2{num4[1]:+12.4e}s{num4[2]:+12.4e}")
        rn4 = np.roots(num4)
        print("    roots= " + "    ".join([cfmt(r) for r in rn4]))

    # Engine thrust responses (if tau=0, these may be zero, but we still compute)
    # For demonstration, if not provided, we still compute similarly:
    # If actual derivatives for thrust are zero, they print as zero.
    a5 = xt
    b5 = mt*xq - xt*(mq+zw) + zt*xw
    c5 = mt*(xw*zq - xq*zw + xo) + xt*(mq*zw - mw*zq - mo) + zt*(mw*xq - mq*xw)
    d5 = mt*(xw*zo - xo*zw) + xt*(mo*zw - mw*zo) + zt*(mw*xo - mo*xw)
    num5 = [a5,b5,c5,d5]

    a6 = zt
    b6 = mt*zq + xt*zu - zt*(mq+xu)
    c6 = mt*(xq*zu - xu*zq + zo) + xt*(mu*zq - mq*zu) + zt*(mq*xu - mu*xq - mo)
    d6 = mt*(xo*zu - xu*zo) + xt*(mu*zo - mo*zu) + zt*(mo*xu - mu*xo)
    num6 = [a6,b6,c6,d6]

    a7 = mt
    b7 = -mt*(xu+zw) + xt*mu + zt*mw
    c7 = mt*(xu*zw - xw*zu) + xt*(mw*zu - mu*zw) + zt*(mu*xw - mw*xu)
    num7 = [a7,b7,c7,0.0]

    a8 = mt
    b8 = -mt*(xu+zw) + xt*mu + zt*mw
    c8 = mt*(xu*zw - xw*zu) + xt*(mw*zu - mu*zw) + zt*(mu*xw - mw*xu)
    num8 = [a8,b8,c8]
    
    if prints:
        print("\nAxial Velocity Response to Engine Thrust:")
        print(f"N(s)={num5[0]:12.4e}s^3{num5[1]:+12.4e}s^2{num5[2]:+12.4e}s{num5[3]:+12.4e}")
        rn5 = np.roots(num5) if any(num5) else []
        print("    roots= " + "    ".join([cfmt(r) for r in rn5]))
    
        print("Normal Velocity Response to Engine Thrust:")
        print(f"N(s)={num6[0]:12.4e}s^3{num6[1]:+12.4e}s^2{num6[2]:+12.4e}s{num6[3]:+12.4e}")
        rn6 = np.roots(num6) if any(num6) else []
        print("    roots= " + "    ".join([cfmt(r) for r in rn6]))
    
        print("Pitch Rate Response to Engine Thrust:")
        print(f"N(s)={num7[0]:12.4e}s^3{num7[1]:+12.4e}s^2{num7[2]:+12.4e}s{0.0:+12.4e}")
        rn7 = np.roots(num7) if any(num7) else []
        print("    roots= " + "    ".join([cfmt(r) for r in rn7]))
    
        print("Pitch Angle Response to Engine Thrust:")
        print(f"N(s)={num8[0]:12.4e}s^2{num8[1]:+12.4e}s{num8[2]:+12.4e}")
        rn8 = np.roots(num8) if any(num8) else []
        print("    roots= " + "    ".join([cfmt(r) for r in rn8]))
        print()

    # Time array
    t = np.arange(tpar[0], tpar[2] + tpar[1]*0.9999999, tpar[1])

    # Step responses
    def step_response(num, den, t):
        sys = lti(num, den)
        _, y = step(sys, T=t)
        return y

    u_eta = step_response([eta*n for n in num1], den, t)
    u_tau = step_response([tau*n for n in num5], den, t)
    u = u_eta + u_tau

    w_eta = step_response([eta*n for n in num2], den, t)
    w_tau = step_response([tau*n for n in num6], den, t)
    w = w_eta + w_tau

    q_eta = step_response([eta*n for n in num3], den, t)
    q_tau = step_response([tau*n for n in num7], den, t)
    q = q_eta + q_tau

    o_eta = step_response([eta*n for n in num4], den, t)
    o_tau = step_response([tau*n for n in num8], den, t)
    o = o_eta + o_tau

    alpha = w / V0
    gamma = -w / V0 + o

    # Plotting
    if plots:
        # Individual plots
        fig1 = plt.figure(); plt.plot(t,u); plt.ylabel('Axial Velocity'); plt.xlabel('Seconds'); plt.grid(True)
        plt.savefig('assets/longitudinal/axial_velocity.png')
        
        fig2 = plt.figure(); plt.plot(t,w); plt.ylabel('Normal Velocity'); plt.xlabel('Seconds'); plt.grid(True)
        plt.savefig('assets/longitudinal/normal_velocity.png')
        
        fig3 = plt.figure(); plt.plot(t,q); plt.ylabel('Pitch Rate'); plt.xlabel('Seconds'); plt.grid(True)
        plt.savefig('assets/longitudinal/pitch_rate.png')
        
        fig4 = plt.figure(); plt.plot(t,o); plt.ylabel('Pitch Angle'); plt.xlabel('Seconds'); plt.grid(True)
        plt.savefig('assets/longitudinal/pitch_angle.png')
        
        fig5 = plt.figure(); plt.plot(t,alpha); plt.ylabel('Angle-of-Attack'); plt.xlabel('Seconds'); plt.grid(True)
        plt.savefig('assets/longitudinal/angle_of_attack.png')
        
        fig6 = plt.figure(); plt.plot(t,gamma); plt.ylabel('Flight Path Angle'); plt.xlabel('Seconds'); plt.grid(True)
        plt.savefig('assets/longitudinal/flight_path_angle.png')
        
        # Combined plots
        fig7 = plt.figure()
        plt.subplot(3,1,1); plt.plot(t,u); plt.ylabel('Axial Velocity'); plt.grid(True)
        plt.subplot(3,1,2); plt.plot(t,w); plt.ylabel('Normal Velocity'); plt.grid(True)
        plt.subplot(3,1,3); plt.plot(t,q); plt.ylabel('Pitch Rate'); plt.grid(True)
        plt.savefig('assets/longitudinal/combined_velocities.png')
        
        fig8 = plt.figure()
        plt.subplot(3,1,1); plt.plot(t,o); plt.ylabel('Pitch Angle'); plt.grid(True)
        plt.subplot(3,1,2); plt.plot(t,alpha); plt.ylabel('Angle-of-Attack'); plt.grid(True)
        plt.subplot(3,1,3); plt.plot(t,gamma); plt.ylabel('Flight Path Angle'); plt.grid(True)
        plt.savefig('assets/longitudinal/combined_angles.png')

    # Steady-state responses
    # uss=eta*d1/e0+tau*d5/e0, etc.
    uss = (eta*d1 + tau*d5)/e0
    wss = (eta*d2 + tau*d6)/e0
    qss = 0.0
    oss = (eta*c4 + tau*c8)/e0
    alphass = wss/V0
    gammass = -wss/V0 + oss
    if prints:
        print("Steady-State Respones:")
        print(f"{uss:12.4e} :Axial Velocity")
        print(f"{wss:12.4e} :Normal Velocity")
        print(f"{qss:12.4e} :Pitch Rate")
        print(f"{oss:12.4e} :Pitch Angle")
        print(f"{alphass:12.4e} :Angle-of-Attack")
        print(f"{gammass:12.4e} :Flight Path Angle")

    if for_optim:
        return np.real(snf), np.real(sdr), np.real(pnf), np.real(pdr)