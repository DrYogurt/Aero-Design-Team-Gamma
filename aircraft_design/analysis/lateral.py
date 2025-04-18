import numpy
from scipy.signal import lti, step
import matplotlib.pyplot as plt

def aircraft_lateral_dynamics(V0, a, b, tpar, xi, zeta, kv, kp, kr, ko,
                             prints=True,
                             plots=True,
                             for_optim=False,
                            ):
    """
    This function replicates the logic of the lateral.m MATLAB script for aircraft lateral-directional dynamics,
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
         a(1,1)=yv a(1,2)=lv a(1,3)=nv a(1,4)=???  
         a(2,1)=yp a(2,2)=lp a(2,3)=np a(2,4)=???
         a(3,1)=yr a(3,2)=lr a(3,3)=nr a(3,4)=???
         a(4,1)=yo a(4,2)=lo a(4,3)=no a(4,4)=1.0 (generally)

    b : np.ndarray shape (4,2)
        Control derivatives matrix
        Format:
         b(1,1)=ye b(1,2)=yz
         b(2,1)=le b(2,2)=lz
         b(3,1)=ne b(3,2)=nz
         b(4,1)=0  b(4,2)=0

    tpar : list or array of length 3
        [t_start, t_step, t_end] for time vector
    xi : float
        Aileron deflection step input (rad)
    zeta : float
        Rudder deflection step input (rad)
    kv, kp, kr, ko : array-like of length 2
        Feedback gains for lateral velocity, roll rate, yaw rate, and roll attitude.
    """
    a = a.T
    b = b.T
    # Unpack the aerodynamic and control derivatives
    yv, lv, nv = a[0,0], a[0,1], a[0,2]
    yp, lp, np = a[1,0], a[1,1], a[1,2]
    yr, lr, nr = a[2,0], a[2,1], a[2,2]
    yo, lo, no = a[3,0], a[3,1], a[3,2]

    ye, yz = b[0,0], b[0,1]
    le, lz = b[1,0], b[1,1]
    ne, nz = b[2,0], b[2,1]
    # b[3,:] are zero

    # Print input data as in the MATLAB script style
    if prints:
        print("Aircraft Lateral-Directional Dynamics\n")
        print(f"  {V0:5.1f} :V0 (ft/s) level cruise")
        print(" --- lateral-directional aerodynamic stability derivatives ---")
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
        print(f"   {xi:10.4f} :aileron deflection, xi (rad) [1 deg = 0.0175 rad]")
        print(f"   {zeta:10.4f} :rudder deflection, zeta (rad)")
        print(" --- feedback control ---")
        print(f"   {kv[0]:4.1f}    {kv[1]:4.1f}  :lateral velocity gains (aileron rudder)")
        print(f"   {kp[0]:4.1f}    {kp[1]:4.1f}  :roll rate gains")
        print(f"   {kr[0]:4.1f}    {kr[1]:4.1f}  :yaw rate gains")
        print(f"   {ko[0]:4.1f}    {ko[1]:4.1f}  :roll attitude gains")
        print(" --- end ---\n")

    
    # Apply feedback modifications
    yv = yv - kv[0]*ye - kv[1]*yz
    lv = lv - kv[0]*le - kv[1]*lz
    nv = nv - kv[0]*ne - kv[1]*nz

    yp = yp - kp[0]*ye - kp[1]*yz
    lp = lp - kp[0]*le - kp[1]*lz
    np = np - kp[0]*ne - kp[1]*nz

    yr = yr - kr[0]*ye - kr[1]*yz
    lr = lr - kr[0]*le - kr[1]*lz
    nr = nr - kr[0]*ne - kr[1]*nz

    yo = yo - ko[0]*ye - ko[1]*yz
    lo = lo - ko[0]*le - ko[1]*lz
    no = no - ko[0]*ne - ko[1]*nz

    # Print modified [A] and [B]
    if prints:
        print("[A]:")
        print(f"{yv:12.4e}{lv:12.4e}{nv:12.4e}{yo:12.4e}")
        print(f"{yp:12.4e}{lp:12.4e}{np:12.4e}{lo:12.4e}")
        print(f"{yr:12.4e}{lr:12.4e}{nr:12.4e}{no:12.4e}")
        print(f"{0.0:12.4e}{0.0:12.4e}{1.0:12.4e}{0.0:12.4e}")
        print("[B]:")
        print(f"{ye:12.4e}{yz:12.4e}")
        print(f"{le:12.4e}{lz:12.4e}")
        print(f"{ne:12.4e}{nz:12.4e}")
        print(f"{0.0:12.4e}{0.0:12.4e}")
    
    # Characteristic equation
    a0 = 1.0#np.float64(1.0)
    b0 = float(-(lp + nr + yv))
    c0 = float((lp*nr - lr*np) + (nr*yv - nv*yr) + (lp*yv - lv*yp) - lo)
    d0 = float((lo*nr - lr*no) + lv*(nr*yp - np*yr - yo) + nv*(lp*yr - lr*yp) + yv*(lr*np - lp*nr + lo))
    e0 = float(lv*(nr*yo - no*yr) + nv*(lo*yr - lr*yo) + yv*(lr*no - lo*nr))

    den = [a0, b0, c0, d0, e0]

    if prints:
        print("\nCharacteristic Equation: as^4+bs^3+cs^2+ds+e=0")
        print(f"{a0:12.4e}s^4{b0:+12.4e}s^3{c0:+12.4e}s^2{d0:+12.4e}s{e0:+12.4e} =0\n")

    # Exact roots - Need to handle complex roots and sort them
    rd = numpy.roots(den)
    
    # In MATLAB, cplxpair sorts complex conjugate pairs together
    # We'll implement a simple version here
    def complex_sort(roots):
        # First separate real and complex roots
        real_roots = [r for r in roots if abs(r.imag) < 1e-10]
        complex_roots = [r for r in roots if abs(r.imag) >= 1e-10]
        
        # Sort complex roots by absolute value of imaginary part
        complex_roots = sorted(complex_roots, key=lambda x: abs(x.imag), reverse=True)
        
        # For true pairs, ensure conjugates are next to each other
        paired_complex = []
        skip_next = False
        for i in range(len(complex_roots)):
            if skip_next:
                skip_next = False
                continue
                
            if i < len(complex_roots) - 1 and abs(complex_roots[i] - numpy.conjugate(complex_roots[i+1])) < 1e-10:
                paired_complex.append(complex_roots[i])
                paired_complex.append(complex_roots[i+1])
                skip_next = True
            else:
                paired_complex.append(complex_roots[i])
                
        # Combine and return
        return paired_complex + real_roots

    rd = complex_sort(rd)
    
    # MATLAB script assumes specific ordering to match Dutch roll, spiral, roll modes
    # For compatibility, we'll use similar logic, but more explicit
    
    # Sort by real part (most negative first) to identify roll mode
    sorted_by_real = sorted(rd, key=lambda x: x.real)
    
    # Find complex conjugate pair for dutch roll
    complex_roots = [r for r in rd if abs(r.imag) > 1e-10]
    if len(complex_roots) >= 2:
        r1, r2 = complex_roots[0], complex_roots[1]  # Dutch roll mode
        
        # Find real roots for roll subsidence and spiral
        real_roots = sorted([r for r in rd if abs(r.imag) <= 1e-10], key=lambda x: x.real)
        if len(real_roots) >= 2:
            r3, r4 = real_roots[0], real_roots[1]  # Roll subsidence, Spiral
        else:
            r3, r4 = numpy.nan, numpy.nan
            #raise ValueError(f"Not enough real roots to identify roll subsidence and spiral modes.\n complex roots: {complex_roots}\n real roots: {real_roots}")
    else:
        # If no complex roots (unusual), just use the sorted roots
        r1, r2, r3, r4 = sorted_by_real
        
    def cfmt(z):
        # Complex root formatting: real+imag i
        return f"{z.real:+.5f}{z.imag:+.5f}i" if abs(z.imag)>1e-14 else f"{z.real:+.5f}"
    
    if prints:
        print("Exact Roots of Characteristic Equation:")
        print(" " + "       ".join([cfmt(z) for z in rd]))

    # Compute stability parameters
    # Dutch roll frequency and damping
    dnf = numpy.sqrt(r1 * r2).real  # Using real part to handle any numerical errors
    ddr = -0.5 * (r1 + r2).real / dnf
    
    # Time constants
    Ts = -1 / r4.real  # Spiral time constant
    Tr = -1 / r3.real  # Roll time constant
    
    if prints:
        print("Factored Characteristic Equation:")
        print(f"\t(s{1/Ts:+12.4e})(s{1/Tr:+12.4e})(s^2{2*ddr*dnf:+12.4e}s{dnf*dnf:+12.4e})=0")
        print("Spiral Stability Mode:")
        print(f"{Ts:12.4e} :time constant")
        print("Roll Subsidence Stability Mode:")
        print(f"{Tr:12.4e} :time constant")
        print("Oscillatory Dutch Roll Stability Mode:")
        print(f"{ddr:12.4e} :damping ratio")
        print(f"{dnf:12.4e} :undamped natural frequency\n")

        print("Aircraft Response Transfer Functions: =N(s)/D(s)")
        print(f"D(s)={a0:12.4e}s^4{b0:+12.4e}s^3{c0:+12.4e}s^2{d0:+12.4e}s{e0:+12.4e}\n")

    # Transfer functions for aileron response
    b1 = le*yp + ne*yr - ye*(lp+nr)
    c1 = le*(np*yr - nr*yp + yo) + ne*(lr*yp - lp*yr) + ye*(lp*nr - lr*np - lo)
    d1 = le*(no*yr - nr*yo) + ne*(lr*yo - lo*yr) + ye*(lo*nr - lr*no)
    num1 = [b1, c1, d1]
    
    if prints:
        print("Lateral Velocity Response to Aileron Deflection:")
        print(f"N(s)={num1[0]:12.4e}s^2{num1[1]:+12.4e}s{num1[2]:+12.4e}")
        rn1 = numpy.roots(num1)
        print("    roots= " + "    ".join([cfmt(r) for r in rn1]))

    a2 = le
    b2 = -le*(nr+yv) + ne*lr + ye*lv
    c2 = le*(nr*yv - nv*yr) + ne*(lv*yr - lr*yv) + ye*(lr*nv - lv*nr)
    d2 = 0.0
    num2 = [a2, b2, c2, d2]
    
    if prints:
        print("Roll Rate Response to Aileron Deflection:")
        print(f"N(s)={num2[0]:12.4e}s^3{num2[1]:+12.4e}s^2{num2[2]:+12.4e}s{num2[3]:+12.4e}")
        rn2 = numpy.roots(num2[:3])  # Ignore last zero coefficient for roots
        print("    roots= " + "    ".join([cfmt(r) for r in rn2]))

    a3 = ne
    b3 = le*np - ne*(lp+yv) + ye*nv
    c3 = le*(nv*yp - np*yv + no) + ne*(lp*yv - lv*yp - lo) + ye*(lv*np - lp*nv)
    d3 = le*(nv*yo - no*yv) + ne*(lo*yv - lv*yo) + ye*(lv*no - lo*nv)
    num3 = [a3, b3, c3, d3]
    
    if prints:
        print("Yaw Rate Response to Aileron Deflection:")
        print(f"N(s)={num3[0]:12.4e}s^3{num3[1]:+12.4e}s^2{num3[2]:+12.4e}s{num3[3]:+12.4e}")
        rn3 = numpy.roots(num3)
        print("    roots= " + "    ".join([cfmt(r) for r in rn3]))

    a4 = le
    b4 = -le*(nr+yv) + ne*lr + ye*lv
    c4 = le*(nr*yv - nv*yr) + ne*(lv*yr - lr*yv) + ye*(lr*nv - lv*nr)
    num4 = [a4, b4, c4]
    
    if prints:
        print("Roll Angle Response to Aileron Deflection:")
        print(f"N(s)={num4[0]:12.4e}s^2{num4[1]:+12.4e}s{num4[2]:+12.4e}")
        rn4 = numpy.roots(num4)
        print("    roots= " + "    ".join([cfmt(r) for r in rn4]))
        print()

    # Transfer functions for rudder response
    a5 = yz
    b5 = lz*yp + nz*yr - yz*(lp+nr)
    c5 = lz*(np*yr - nr*yp + yo) + nz*(lr*yp - lp*yr) + yz*(lp*nr - lr*np - lo)
    d5 = lz*(no*yr - nr*yo) + nz*(lr*yo - lo*yr) + yz*(lo*nr - lr*no)
    num5 = [a5, b5, c5, d5]
    
    if prints:
        print("Lateral Velocity Response to Rudder Deflection:")
        print(f"N(s)={num5[0]:12.4e}s^3{num5[1]:+12.4e}s^2{num5[2]:+12.4e}s{num5[3]:+12.4e}")
        rn5 = numpy.roots(num5)
        print("    roots= " + "    ".join([cfmt(r) for r in rn5]))

    a6 = lz
    b6 = -lz*(nr+yv) + nz*lr + yz*lv
    c6 = lz*(nr*yv - nv*yr) + nz*(lv*yr - lr*yv) + yz*(lr*nv - lv*nr)
    d6 = 0.0
    num6 = [a6, b6, c6, d6]
    
    if prints:
        print("Roll Rate Response to Rudder Deflection:")
        print(f"N(s)={num6[0]:12.4e}s^3{num6[1]:+12.4e}s^2{num6[2]:+12.4e}s{num6[3]:+12.4e}")
        rn6 = numpy.roots(num6[:3])  # Ignore last zero coefficient for roots
        print("    roots= " + "    ".join([cfmt(r) for r in rn6]))

    a7 = nz
    b7 = lz*np - nz*(lp+yv) + yz*nv
    c7 = lz*(nv*yp - np*yv + no) + nz*(lp*yv - lv*yp - lo) + yz*(lv*np - lp*nv)
    d7 = lz*(nv*yo - no*yv) + nz*(lo*yv - lv*yo) + yz*(lv*no - lo*nv)
    num7 = [a7, b7, c7, d7]
    
    if prints:
        print("Yaw Rate Response to Rudder Deflection:")
        print(f"N(s)={num7[0]:12.4e}s^3{num7[1]:+12.4e}s^2{num7[2]:+12.4e}s{num7[3]:+12.4e}")
        rn7 = numpy.roots(num7)
        print("    roots= " + "    ".join([cfmt(r) for r in rn7]))

    a8 = lz
    b8 = -lz*(nr+yv) + nz*lr + yz*lv
    c8 = lz*(nr*yv - nv*yr) + nz*(lv*yr - lr*yv) + yz*(lr*nv - lv*nr)
    num8 = [a8, b8, c8]
    
    if prints:
        print("Roll Angle Response to Rudder Deflection:")
        print(f"N(s)={num8[0]:12.4e}s^2{num8[1]:+12.4e}s{num8[2]:+12.4e}")
        rn8 = numpy.roots(num8)
        print("    roots= " + "    ".join([cfmt(r) for r in rn8]))
        print()

    # Time array
    t = numpy.arange(tpar[0], tpar[2] + tpar[1]*0.9999999, tpar[1])

    # Step responses
    def step_response(num, den, t):
        sys = lti(num, den)
        _, y = step(sys, T=t)
        return y

    v_xi = step_response([xi*n for n in num1], den, t)
    v_zeta = step_response([zeta*n for n in num5], den, t)
    v = v_xi + v_zeta

    p_xi = step_response([xi*n for n in num2], den, t)
    p_zeta = step_response([zeta*n for n in num6], den, t)
    p = p_xi + p_zeta

    r_xi = step_response([xi*n for n in num3], den, t)
    r_zeta = step_response([zeta*n for n in num7], den, t)
    r = r_xi + r_zeta

    o_xi = step_response([xi*n for n in num4], den, t)
    o_zeta = step_response([zeta*n for n in num8], den, t)
    o = o_xi + o_zeta

    beta = v / V0

    # Write data to file (optional)
    if False:
        # Save to numpy .npy file instead of a flat file
        data = numpy.column_stack((t, v, p, r, o, beta))
        numpy.save('lateral.npy', data)
        
        # Also write a text file similar to the MATLAB version
        with open('lateral.dat', 'w') as fd:
            for i in range(len(t)):
                fd.write(f"{t[i]:12.4e}{v[i]:12.4e}{p[i]:12.4e}{r[i]:12.4e}{o[i]:12.4e}{beta[i]:12.4e}\n")

    # Plotting
    if plots:
        # Individual plots
        fig1 = plt.figure(); plt.plot(t,v); plt.ylabel('Lateral Velocity'); plt.xlabel('Seconds'); plt.grid(True)
        plt.savefig('assets/lateral/lateral_velocity.png')
        
        fig2 = plt.figure(); plt.plot(t,p); plt.ylabel('Roll Rate'); plt.xlabel('Seconds'); plt.grid(True)
        plt.savefig('assets/lateral/roll_rate.png')
        
        fig3 = plt.figure(); plt.plot(t,r); plt.ylabel('Yaw Rate'); plt.xlabel('Seconds'); plt.grid(True)
        plt.savefig('assets/lateral/yaw_rate.png')
        
        fig4 = plt.figure(); plt.plot(t,o); plt.ylabel('Roll Angle'); plt.xlabel('Seconds'); plt.grid(True)
        plt.savefig('assets/lateral/roll_angle.png')
        
        fig5 = plt.figure(); plt.plot(t,beta); plt.ylabel('Sideslip Angle'); plt.xlabel('Seconds'); plt.grid(True)
        plt.savefig('assets/lateral/sideslip_angle.png')
        
        # Combined plots
        fig6 = plt.figure()
        plt.subplot(3,1,1); plt.plot(t,v); plt.ylabel('Lateral Velocity'); plt.grid(True)
        plt.subplot(3,1,2); plt.plot(t,p); plt.ylabel('Roll Rate'); plt.grid(True)
        plt.subplot(3,1,3); plt.plot(t,r); plt.ylabel('Yaw Rate'); plt.grid(True)
        plt.savefig('assets/lateral/lateral_combined1.png')
        
        fig7 = plt.figure()
        plt.subplot(3,1,1); plt.plot(t,o); plt.ylabel('Roll Angle'); plt.grid(True)
        plt.subplot(3,1,2); plt.plot(t,beta); plt.ylabel('Sideslip Angle'); plt.grid(True)
        plt.savefig('assets/lateral/lateral_combined2.png')

    # Steady-state responses
    vss = (xi*d1 + zeta*d5)/e0
    pss = 0.0
    rss = (xi*d3 + zeta*d7)/e0
    oss = (xi*c4 + zeta*c8)/e0
    betass = vss/V0
    
    if prints:
        print("Steady-State Responses:")
        print(f"{vss:12.4e} :Lateral Velocity")
        print(f"{pss:12.4e} :Roll Rate")
        print(f"{rss:12.4e} :Yaw Rate")
        print(f"{oss:12.4e} :Roll Angle")
        print(f"{betass:12.4e} :Sideslip Angle")


    if for_optim:
        return dnf, ddr, Tr, Ts

# Example usage with dummy data
if __name__ == "__main__":
    # These are placeholder values - replace with actual aircraft data
    V0 = 585.0  # ft/s
    
    # Example stability derivatives matrix (4x4)
    a = numpy.array([
        [-0.0682, 0.0, 0.0, 0.0],
        [0.0, -1.936, 0.546, 0.0],
        [0.0, -0.272, -0.397, 0.0],
        [0.0, 0.0, 1.0, 0.0]
    ])
    
    # Example control derivatives matrix (2x4)
    b = numpy.array([
        [0.0, 0.0234],
        [0.138, 0.0043],
        [0.0037, -0.0536],
        [0.0, 0.0]
    ])
    
    # Time parameters [start, step, end]
    tpar = [0.0, 0.05, 10.0]
    
    # Step inputs
    xi = 0.0175    # aileron deflection (rad) - approximately 1 degree
    zeta = 0.0     # rudder deflection (rad)
    
    # Feedback control gains (all zeros for open-loop response)
    kv = [0.0, 0.0]  # lateral velocity gains
    kp = [0.0, 0.0]  # roll rate gains
    kr = [0.0, 0.0]  # yaw rate gains
    ko = [0.0, 0.0]  # roll attitude gains
    
    # Call the function
    aircraft_lateral_dynamics(V0, a, b, tpar, xi, zeta, kv, kp, kr, ko)