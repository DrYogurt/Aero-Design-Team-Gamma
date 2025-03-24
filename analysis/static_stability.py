import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad

#####################################################
### Longitudinal Static Stability Functions ##########
#####################################################

def longitudinal_static_stability(M_ac, lt, Lt, lw=None, Lw=None):
    """
    Calculate pitching moment for longitudinal static stability
    
    Parameters:
    M_ac: Aerodynamic center moment
    lt: Tail arm (distance from CG to tail aerodynamic center)
    Lt: Tail lift force
    lw: Wing arm (distance from CG to wing aerodynamic center)
    Lw: Wing lift force
    
    Returns:
    M: Total pitching moment
    """
    # For zero angle of attack
    M = M_ac - lt * Lt
    
    # For nonzero angle of attack (if wing parameters provided)
    if lw is not None and Lw is not None:
        M_nonzero = M_ac + lw * Lw - lt * Lt
        return M_nonzero
    
    return M

def total_wing_lift(q, S, aw, alpha):
    """
    Calculate total lift for the wing
    
    Parameters:
    q: Dynamic pressure (1/2 * rho * V^2)
    S: Wing area
    aw: Slope of wing lift curve
    alpha: Angle of attack
    
    Returns:
    Lw: Wing lift
    """
    Lw = q * S * aw * alpha
    return Lw

def tail_lift(q, St, at, alpha_t, eta_t):
    """
    Calculate tail lift
    
    Parameters:
    q: Dynamic pressure
    St: Tail area
    at: Slope of tail lift curve
    alpha_t: Tail angle of attack
    eta_t: Tail efficiency factor
    
    Returns:
    Lt: Tail lift
    """
    Lt = eta_t * q * St * at * alpha_t
    return Lt

def tail_angle_of_attack(alpha_w, it, epsilon):
    """
    Calculate tail angle of attack
    
    Parameters:
    alpha_w: Wing angle of attack
    it: Tail incidence angle
    epsilon: Downwash angle
    
    Returns:
    alpha_t: Tail angle of attack
    """
    alpha_t = alpha_w - it - epsilon
    return alpha_t

def downwash_angle(epsilon_0, d_epsilon_d_alpha, alpha):
    """
    Calculate downwash angle
    
    Parameters:
    epsilon_0: Zero-angle downwash
    d_epsilon_d_alpha: Rate of change of downwash with angle of attack
    alpha: Angle of attack
    
    Returns:
    epsilon: Downwash angle
    """
    epsilon = epsilon_0 + d_epsilon_d_alpha * alpha
    return epsilon

def lift_coefficient(q, S, L):
    """
    Calculate lift coefficient from lift force
    
    Parameters:
    q: Dynamic pressure
    S: Reference area
    L: Lift force
    
    Returns:
    CL: Lift coefficient
    """
    CL = L / (q * S)
    return CL

def total_airplane_lift(q, S, aw, at, St, alpha, it, epsilon_alpha, eta_t):
    """
    Calculate total airplane lift
    
    Parameters:
    q: Dynamic pressure
    S: Wing area
    aw: Wing lift curve slope
    at: Tail lift curve slope
    St: Tail area
    alpha: Angle of attack
    it: Tail incidence
    epsilon_alpha: Rate of change of downwash with alpha
    eta_t: Tail efficiency
    
    Returns:
    L: Total lift
    """
    # Calculate downwash at the tail
    epsilon = epsilon_alpha * alpha
    
    # Calculate wing lift
    L_wing = q * S * aw * alpha
    
    # Calculate tail lift (considering downwash)
    alpha_t = alpha - it - epsilon
    L_tail = -eta_t * q * St * at * it  # Tail contribution from incidence
    
    # Total lift equation from page 2
    L = q * S * (aw + at * (St/S) * (1 - epsilon_alpha)) * alpha - eta_t * q * at * St * it
    
    return L

def lift_coefficient_alpha(aw, eta_t, at, St, S, epsilon_alpha):
    """
    Calculate lift coefficient slope
    
    Parameters:
    aw: Wing lift curve slope
    eta_t: Tail efficiency
    at: Tail lift curve slope
    St: Tail area
    S: Wing area
    epsilon_alpha: Rate of change of downwash with alpha
    
    Returns:
    CL_alpha: Lift coefficient slope
    """
    CL_alpha = aw + eta_t * at * (St/S) * (1 - epsilon_alpha)
    return CL_alpha

def lift_coefficient_zero(eta_t, at, St, S, it):
    """
    Calculate zero-alpha lift coefficient
    
    Parameters:
    eta_t: Tail efficiency
    at: Tail lift curve slope
    St: Tail area
    S: Wing area
    it: Tail incidence
    
    Returns:
    CL_0: Zero-alpha lift coefficient
    """
    CL_0 = -eta_t * at * (St/S) * it
    return CL_0

def airplane_pitching_moment(h, h_nw, c, Lw, h_t, Lt, M_ac):
    """
    Calculate airplane pitching moment
    
    Parameters:
    h: CG position (fraction of chord)
    h_nw: Wing neutral point
    c: Mean aerodynamic chord
    Lw: Wing lift
    h_t: Tail position
    Lt: Tail lift
    M_ac: Moment about aerodynamic center
    
    Returns:
    M: Pitching moment
    """
    M = (h - h_nw) * c * Lw - (h_t - h) * c * Lt + M_ac
    return M

def moment_coefficient(M, q, S, c):
    """
    Calculate moment coefficient
    
    Parameters:
    M: Moment
    q: Dynamic pressure
    S: Reference area
    c: Reference chord
    
    Returns:
    CM: Moment coefficient
    """
    CM = M / (q * S * c)
    return CM

def horizontal_tail_volume_ratio(St, lt, S, c):
    """
    Calculate horizontal tail volume ratio
    
    Parameters:
    St: Tail area
    lt: Tail arm
    S: Wing area
    c: Reference chord
    
    Returns:
    VH: Horizontal tail volume ratio
    """
    VH = (St * lt) / (S * c)
    return VHq


def moment_coefficient_formula_tail(h, h_nw, CL_w, eta_t, VH, CL_t, CM_ac):
    """
    Calculate moment coefficient using horizontal tail volume ratio
    
    Parameters:
    h: CG position
    h_nw: Wing neutral point
    CL_w: Wing lift coefficient
    eta_t: Tail efficiency
    VH: Horizontal tail volume ratio
    CL_t: Tail lift coefficient
    CM_ac: Moment coefficient about aerodynamic center
    
    Returns:
    CM: Total moment coefficient
    """
    CM_tail = (h - h_nw) * CL_w - eta_t * VH * CL_t + CM_ac
    return CM_tail

def moment_coefficient_alpha(h, h_nw, aw, h_t, eta_t, St, S, at, epsilon_alpha):
    """
    Calculate rate of change of moment coefficient with angle of attack
    
    Parameters:
    h: CG position
    h_nw: Wing neutral point
    aw: Wing lift curve slope
    h_t: Tail position
    eta_t: Tail efficiency
    St: Tail area
    S: Wing area
    at: Tail lift curve slope
    epsilon_alpha: Rate of change of downwash
    
    Returns:
    CM_alpha: Rate of change of moment with alpha
    """
    CM_alpha = (h - h_nw) * aw - (h_t - h) * eta_t * (St/S) * at * (1 - epsilon_alpha)
    return CM_alpha

def neutral_point(h_nw, h_t, eta_t, St, S, at, aw, epsilon_alpha):
    """
    Calculate neutral point position
    
    Parameters:
    h_nw: Wing-body neutral point
    h_t: Tail aerodynamic center position
    eta_t: Tail efficiency factor
    St: Tail area
    S: Wing area
    at: Tail lift curve slope
    aw: Wing lift curve slope
    epsilon_alpha: Rate of change of downwash angle with angle of attack
    
    Returns:
    h_n: Neutral point position
    """
    numerator = h_nw + h_t * eta_t * (St/S) * (at/aw) * (1 - epsilon_alpha)
    denominator = 1 + eta_t * (St/S) * (at/aw) * (1 - epsilon_alpha)
    
    h_n = numerator / denominator
    
    return h_n

def moment_coefficient_zero(h_t, h, eta_t, St, S, at, it, CM_ac):
    """
    Calculate zero-alpha moment coefficient
    
    Parameters:
    h_t: Tail position
    h: CG position
    eta_t: Tail efficiency
    St: Tail area
    S: Wing area
    at: Tail lift curve slope
    it: Tail incidence
    CM_ac: Moment coefficient about aerodynamic center
    
    Returns:
    CM_0: Zero-alpha moment coefficient
    """
    CM_0 = (h_t - h) * eta_t * (St/S) * at * it + CM_ac
    return CM_0

def trim_angle_of_attack(alpha_1, V1, V2):
    """
    Calculate trim angle of attack at a different airspeed
    
    Parameters:
    alpha_1: Initial trim angle of attack
    V1: Initial airspeed
    V2: New airspeed
    
    Returns:
    alpha_2: New trim angle of attack
    """
    alpha_2 = alpha_1 * (V1/V2)**2
    return alpha_2

def elevator_deflection_for_trim(CM_alpha, V1, V2, alpha_1, V_H):
    """
    Calculate change in elevator deflection required for trim
    
    Parameters:
    CM_alpha: Rate of change of moment coefficient with angle of attack
    V1: Initial airspeed
    V2: New airspeed
    alpha_1: Initial trim angle of attack
    V_H: Horizontal tail volume ratio
    
    Returns:
    delta_it: Change in elevator deflection
    """
    delta_it = -(CM_alpha/V_H) * ((V1/V2)**2 - 1) * alpha_1
    return delta_it

def tail_incidence_for_trim(CM_ac, CL, alpha, CM_alpha, CL_alpha, CM_i, CL_i):
    """
    Calculate tail incidence required for trim
    
    Parameters:
    CM_ac: Moment coefficient about aerodynamic center
    CL: Lift coefficient
    alpha: Angle of attack
    CM_alpha: Rate of change of moment coefficient with alpha
    CL_alpha: Rate of change of lift coefficient with alpha
    CM_i: Rate of change of moment coefficient with incidence
    CL_i: Rate of change of lift coefficient with incidence
    
    Returns:
    it: Tail incidence
    """
    numerator = -CM_ac * CL_alpha + CM_alpha * CL
    denominator = CL_alpha * CM_i - CM_alpha * CL_i
    
    it = numerator / denominator
    
    return it

def tail_incidence_for_trim_simplified(CM_ac, CL, VH):
    """
    Calculate tail incidence for trim (simplified)
    
    Parameters:
    CM_ac: Moment coefficient about aerodynamic center
    CL: Lift coefficient
    VH: Horizontal tail volume ratio
    
    Returns:
    it: Tail incidence angle
    """
    it = -(CM_ac + VH * CL) / (eta_t * at * VH)
    return it

def moment_due_to_pitch_rate(eta_t, VH, at, lt, c, Q, V):
    """
    Calculate moment due to pitch rate
    
    Parameters:
    eta_t: Tail efficiency
    VH: Horizontal tail volume ratio
    at: Tail lift curve slope
    lt: Tail arm
    c: Reference chord
    Q: Pitch rate
    V: Airspeed
    
    Returns:
    delta_CM: Change in moment coefficient
    """
    q_bar = Q * c / (2 * V)
    delta_CM = -2 * eta_t * VH * at * (lt/c) * q_bar
    return delta_CM

def pitch_damping_coefficient(eta_t, VH, at, lt, c):
    """
    Calculate pitch damping coefficient
    
    Parameters:
    eta_t: Tail efficiency
    VH: Horizontal tail volume ratio
    at: Tail lift curve slope
    lt: Tail arm
    c: Reference chord
    
    Returns:
    CM_q: Pitch damping coefficient
    """
    CM_q = -2 * eta_t * VH * at * (lt/c)
    return CM_q

def static_margin(h_n, h):
    """
    Calculate static margin
    
    Parameters:
    h_n: Neutral point
    h: CG position
    
    Returns:
    SM: Static margin
    """
    SM = h_n - h
    return SM

def trim_angle_from_cm(CM_0, dCM_dAlpha):
    """
    Calculate trim angle from moment coefficient components
    
    Parameters:
    CM_0: Zero-alpha moment coefficient
    dCM_dAlpha: Rate of change of moment coefficient with alpha
    
    Returns:
    alpha_trim: Trim angle of attack
    """
    alpha_trim = -CM_0 / dCM_dAlpha
    return alpha_trim

#####################################################
### Directional Stability Functions #################
#####################################################

def directional_stability_moment(q_t, S_v, a_v, beta, d_epsilon_d_beta, l_v):
    """
    Calculate yawing moment due to sideslip
    
    Parameters:
    q_t: Dynamic pressure at tail
    S_v: Vertical tail area
    a_v: Vertical tail lift curve slope
    beta: Sideslip angle
    d_epsilon_d_beta: Change in sidewash with sideslip
    l_v: Vertical tail moment arm
    
    Returns:
    N_v: Yawing moment due to sideslip
    """
    N_v = q_t * S_v * a_v * beta * (1 - d_epsilon_d_beta) * l_v
    return N_v

def yaw_moment_coefficient(N, q, S, b):
    """
    Calculate yaw moment coefficient
    
    Parameters:
    N: Yaw moment
    q: Dynamic pressure
    S: Reference area
    b: Wingspan
    
    Returns:
    CN: Yaw moment coefficient
    """
    CN = N / (q * S * b)
    return CN

def yaw_moment_due_to_beta(eta_t, S_v, a_v, S, b, beta, d_epsilon_d_beta):
    """
    Calculate yaw moment coefficient due to sideslip
    
    Parameters:
    eta_t: Tail efficiency
    S_v: Vertical tail area
    a_v: Vertical tail lift curve slope
    S: Reference area
    b: Wingspan
    beta: Sideslip angle
    d_epsilon_d_beta: Rate of change of sidewash with sideslip
    
    Returns:
    CN_beta: Yaw moment coefficient derivative with respect to beta
    """
    CN_beta = eta_t * (S_v/S) * a_v * (l_v/b) * (1 - d_epsilon_d_beta)
    return CN_beta

def vertical_tail_volume_ratio(S_v, l_v, S, b):
    """
    Calculate vertical tail volume ratio
    
    Parameters:
    S_v: Vertical tail area
    l_v: Vertical tail arm
    S: Wing area
    b: Wingspan
    
    Returns:
    V_v: Vertical tail volume ratio
    """
    V_v = (S_v * l_v) / (S * b)
    return V_v

def yaw_moment_due_to_rudder(q_t, S_v, a_v, tau, eta, delta_r):
    """
    Calculate yaw moment due to rudder deflection
    
    Parameters:
    q_t: Dynamic pressure at tail
    S_v: Vertical tail area
    a_v: Vertical tail lift curve slope
    tau: Rudder effectiveness
    eta: Efficiency factor
    delta_r: Rudder deflection
    
    Returns:
    Delta_N: Yaw moment due to rudder
    """
    Delta_N = -q_t * S_v * a_v * tau * eta * delta_r
    return Delta_N

def rudder_deflection_for_engine_out(T_ye, CN_delta_r, q, S, b):
    """
    Calculate rudder deflection required for engine-out condition
    
    Parameters:
    T_ye: Engine-out yawing moment
    CN_delta_r: Yaw moment coefficient due to rudder
    q: Dynamic pressure
    S: Reference area
    b: Wingspan
    
    Returns:
    delta_r: Required rudder deflection
    """
    # Calculate engine-out yaw coefficient
    CN_ye = T_ye / (q * S * b)
    
    # Calculate required rudder deflection
    delta_r = CN_ye / CN_delta_r
    
    return delta_r

def effective_aspect_ratio(AR_geo):
    """
    Calculate effective aspect ratio for vertical tail
    
    Parameters:
    AR_geo: Geometric aspect ratio
    
    Returns:
    AR_eff: Effective aspect ratio
    """
    AR_eff = 1.9 * AR_geo
    return AR_eff

#####################################################
### Lateral Control Functions #######################
#####################################################

def roll_moment_due_to_aileron(q, a_0, tau, eta, delta_a, y1, y2):
    """
    Calculate rolling moment due to aileron deflection
    
    Parameters:
    q: Dynamic pressure
    a_0: Section lift curve slope
    tau: Aileron effectiveness parameter
    eta: Span efficiency factor
    delta_a: Aileron deflection
    y1, y2: Spanwise positions of aileron
    
    Returns:
    L_aileron: Rolling moment due to aileron deflection
    """
    # Integration along the span where the aileron is located
    y_values = np.linspace(y1, y2, 100)
    integrand = y_values
    
    # Trapezoidal integration
    L_aileron = -q * a_0 * tau * eta * delta_a * np.trapz(integrand, y_values)
    return L_aileron

def roll_coefficient_due_to_aileron(q, S, b, a_0, tau, eta, delta_a, y1, y2):
    """
    Calculate roll coefficient due to aileron deflection
    
    Parameters:
    q: Dynamic pressure
    S: Wing area
    b: Wingspan
    a_0: Section lift curve slope
    tau: Aileron effectiveness
    eta: Efficiency factor
    delta_a: Aileron deflection
    y1, y2: Spanwise aileron positions
    
    Returns:
    Cl_delta_a: Roll coefficient due to aileron
    """
    # Roll moment from aileron
    L_aileron = roll_moment_due_to_aileron(q, a_0, tau, eta, delta_a, y1, y2)
    
    # Convert to coefficient
    Cl_delta_a = L_aileron / (q * S * b)
    
    return Cl_delta_a

def roll_coefficient_due_to_roll_rate(a_0, AR, p, V, lambda_ratio=1):
    """
    Calculate roll damping coefficient
    
    Parameters:
    a_0: Section lift curve slope
    AR: Aspect ratio
    p: Roll rate
    V: Airspeed
    lambda_ratio: Taper ratio
    
    Returns:
    Cl_p: Roll damping coefficient
    """
    p_bar = p * b / (2 * V)
    
    # For linearly tapered wing
    Cl_p = -a_0 * p_bar * (1 + 3*lambda_ratio) / (12 * (1 + lambda_ratio))
    
    return Cl_p

def steady_roll_rate(Cl_delta_a, Cl_p, delta_a):
    """
    Calculate steady roll rate due to aileron deflection
    
    Parameters:
    Cl_delta_a: Roll moment coefficient due to aileron deflection
    Cl_p: Roll damping coefficient
    delta_a: Aileron deflection
    
    Returns:
    p_bar: Normalized steady roll rate
    """
    p_bar = -Cl_delta_a * delta_a / Cl_p
    return p_bar

def roll_moment_due_to_rudder(q_t, S_v, a_v, tau, eta, delta_r, z_v):
    """
    Calculate roll moment due to rudder deflection
    
    Parameters:
    q_t: Dynamic pressure at tail
    S_v: Vertical tail area
    a_v: Vertical tail lift curve slope
    tau: Rudder effectiveness
    eta: Efficiency factor
    delta_r: Rudder deflection
    z_v: Vertical distance from CG to vertical tail
    
    Returns:
    Delta_L: Roll moment due to rudder
    """
    Delta_L = q_t * S_v * a_v * tau * eta * delta_r * z_v
    return Delta_L

def roll_coefficient_due_to_rudder(q, S, b, q_t, S_v, a_v, tau, eta, delta_r, z_v):
    """
    Calculate roll coefficient due to rudder deflection
    
    Parameters:
    q: Dynamic pressure
    S: Wing area
    b: Wingspan
    q_t: Dynamic pressure at tail
    S_v: Vertical tail area
    a_v: Vertical tail lift curve slope
    tau: Rudder effectiveness
    eta: Efficiency factor
    delta_r: Rudder deflection
    z_v: Vertical distance from CG to vertical tail
    
    Returns:
    Cl_delta_r: Roll coefficient due to rudder
    """
    # Roll moment from rudder
    Delta_L = roll_moment_due_to_rudder(q_t, S_v, a_v, tau, eta, delta_r, z_v)
    
    # Convert to coefficient
    Cl_delta_r = Delta_L / (q * S * b)
    
    return Cl_delta_r

def roll_coefficient_due_to_yaw_rate(a_0, alpha, r, V, b, lambda_ratio=1):
    """
    Calculate roll coefficient due to yaw rate
    
    Parameters:
    a_0: Section lift curve slope
    alpha: Angle of attack
    r: Yaw rate
    V: Airspeed
    b: Wingspan
    lambda_ratio: Taper ratio
    
    Returns:
    Cl_r: Roll coefficient due to yaw rate
    """
    r_bar = r * b / (2 * V)
    
    # For linearly tapered wing
    Cl_r = a_0 * alpha * r_bar * (1 + 3*lambda_ratio) / (6 * (1 + lambda_ratio))
    
    return Cl_r

#####################################################
### Wing and Aircraft Geometry Functions ############
#####################################################

def mean_aerodynamic_chord(c_0, lambda_ratio):
    """
    Calculate mean aerodynamic chord for linearly tapered wing
    
    Parameters:
    c_0: Root chord
    lambda_ratio: Taper ratio (c_t/c_0)
    
    Returns:
    c_bar: Mean aerodynamic chord
    """
    c_bar = (2/3) * c_0 * (1 + lambda_ratio + lambda_ratio**2) / (1 + lambda_ratio)
    return c_bar

def wing_area(b, c_0, lambda_ratio):
    """
    Calculate wing area for linearly tapered wing
    
    Parameters:
    b: Wingspan
    c_0: Root chord
    lambda_ratio: Taper ratio
    
    Returns:
    S: Wing area
    """
    S = b * c_0 * (1 + lambda_ratio) / 2
    return S

def aspect_ratio(b, S):
    """
    Calculate aspect ratio
    
    Parameters:
    b: Wingspan
    S: Wing area
    
    Returns:
    AR: Aspect ratio
    """
    AR = b**2 / S
    return AR

def taper_ratio(c_t, c_0):
    """
    Calculate taper ratio
    
    Parameters:
    c_t: Tip chord
    c_0: Root chord
    
    Returns:
    lambda_ratio: Taper ratio
    """
    lambda_ratio = c_t / c_0
    return lambda_ratio

def wing_lift_curve_slope(CL_alpha, AR):
    """
    Calculate finite wing lift curve slope
    
    Parameters:
    CL_alpha: Section lift curve slope (typically 2π)
    AR: Aspect ratio
    
    Returns:
    a_w: Wing lift curve slope
    """
    a_w = CL_alpha / (1 + CL_alpha/(np.pi * AR))
    return a_w

def chord_distribution(y, b, c_0, lambda_ratio):
    """
    Calculate chord length at spanwise location for linearly tapered wing
    
    Parameters:
    y: Spanwise position
    b: Wingspan
    c_0: Root chord
    lambda_ratio: Taper ratio
    
    Returns:
    c: Chord at position y
    """
    # Normalize y to x (x = 0 at root, x = 1 at tip)
    x = 2 * abs(y) / b
    
    # Chord distribution for linearly tapered wing
    c = c_0 * (1 - (1 - lambda_ratio) * x)
    
    return c

def distance_AC_behind_quarter_chord(lambda_ratio, b, sweep_angle):
    """
    Calculate AC location as distance behind quarter-chord at root

    Parameters:
    lambda_ratio: Taper ratio
    b: Wingspan
    sweep_angle = Sweep Angle (Degrees)

    Returns:
    X_A_swept
    """
    X_A = ((1+2*lambda_ratio) / (1+lambda_ratio)) * (b/6) * np.tan(np.radians(sweep_angle))

    return X_A

#####################################################
### Swept Wing Functions ############################
#####################################################

def effective_moment_coefficient(M_0r, sweep_angle):
    """
    Calculate effective moment coefficient for swept wing
    
    Parameters:
    M_0r: Normal moment coefficient
    cos_sweep: Cosine of sweep angle
    
    Returns:
    M_0: Effective moment coefficient
    """
    cos_sweep = np.cos(np.radians(sweep_angle))
    M_0 = M_0r / cos_sweep
    return M_0

def wing_sweep_correction(X_A, sweep_angle):
    """
    Calculate aerodynamic center shift due to sweep
    
    Parameters:
    X_A: Standard aerodynamic center
    sweep_angle: Wing sweep angle
    
    Returns:
    X_A_swept: Swept wing aerodynamic center
    """
    X_A_swept = X_A * np.tan(np.radians(sweep_angle))
    return X_A_swept

def aerodynamic_center_moment(q, S, c, CM_ac):
    """
    Calculate moment about aerodynamic center
    
    Parameters:
    q: Dynamic pressure
    S: Reference area
    c: Mean aerodynamic chord
    CM_ac: Moment coefficient about AC
    
    Returns:
    M_ac: Moment about aerodynamic center
    """
    M_ac = CM_ac * q * S * c
    return M_ac

def wing_body_lift_coefficient(alpha, alpha_L0, CL_alpha):
    """
    Calculate lift coefficient for wing-body combination
    
    Parameters:
    alpha: Angle of attack
    alpha_L0: Zero-lift angle of attack
    CL_alpha: Lift curve slope
    
    Returns:
    CL_wb: Wing-body lift coefficient
    """
    CL_wb = CL_alpha * (alpha - alpha_L0)
    return CL_wb

def mach_correction_factor(tmax, c):
    """
    Calculate Mach number correction for moment coefficient
    
    Parameters:
    tmax: Maximum thickness
    c: Chord
    
    Returns:
    M_cr: Mach correction factor
    """
    # Formula from page 3: M_cr = 1 - 0.065(100*tmax/c)^0.9
    thickness_percent = 100 * tmax / c
    M_cr = 1 - 0.065 * (thickness_percent)**0.9
    return M_cr

#####################################################
### Combined Analysis Functions #####################
#####################################################

def longitudinal_stability_analysis(aircraft_params, flight_conditions):
    """
    Perform comprehensive longitudinal stability analysis
    
    Parameters:
    aircraft_params: Dictionary of aircraft parameters
    flight_conditions: Dictionary of flight conditions
    
    Returns:
    results: Dictionary of stability analysis results
    """
    # Extract aircraft parameters
    S = aircraft_params.wing_area
    c = aircraft_params.mac
    St = aircraft_params.tail_area
    lt = aircraft_params.tail_arm
    h = aircraft_params.cg_position
    h_ac = aircraft_params.ac_position
    h_t = aircraft_params.tail_ac_position
    it = aircraft_params.tail_incidence
    eta_t = aircraft_params.tail_efficiency
    aw = aircraft_params.wing_lift_slope
    at = aircraft_params.tail_lift_slope
    CM_ac = aircraft_params.cm_ac
    epsilon_0 = aircraft_params.get('epsilon_0', 0)
    d_epsilon_d_alpha = aircraft_params.get('d_epsilon_d_alpha', 0.4)  # Typical value
    
    # Extract flight conditions
    V = flight_conditions['airspeed']
    rho = flight_conditions['density']
    alpha = flight_conditions.get('alpha', 0)
    
    # Calculate dynamic pressure
    q = 0.5 * rho * V**2
    
    # Calculate horizontal tail volume ratio
    VH = horizontal_tail_volume_ratio(St, lt, S, c)
    
    # Calculate downwash at the tail
    epsilon = downwash_angle(epsilon_0, d_epsilon_d_alpha, alpha)
    
    # Calculate tail angle of attack
    alpha_t = tail_angle_of_attack(alpha, it, epsilon)
    
    # Calculate lift components
    CL_wb = wing_body_lift_coefficient(alpha, 0, aw)  # Assuming zero-lift alpha is 0
    L_wing = q * S * CL_wb
    L_tail = tail_lift(q, St, at, alpha_t, eta_t)
    
    # Calculate total lift coefficient
    CL_total = lift_coefficient_alpha(aw, eta_t, at, St, S, d_epsilon_d_alpha) * alpha + \
               lift_coefficient_zero(eta_t, at, St, S, it)
    
    # Calculate pitching moment components
    M_ac_contribution = aerodynamic_center_moment(q, S, c, CM_ac)
    M_wing = (h - h_ac) * c * L_wing
    M_tail = -(h_t - h) * c * L_tail
    
    # Calculate total pitching moment
    M_total = M_ac_contribution + M_wing + M_tail
    
    # Calculate moment coefficient
    CM = moment_coefficient(M_total, q, S, c)
    
    # Calculate CM_alpha
    CM_alpha = moment_coefficient_alpha(h, h_ac, aw, h_t, h, eta_t, St, S, at, d_epsilon_d_alpha)
    
    # Calculate neutral point
    h_n = neutral_point(h_ac, h_t, eta_t, St, S, at, aw, d_epsilon_d_alpha)
    
    # Calculate static margin
    SM = static_margin(h_n, h)
    
    # Calculate trim angle of attack
    CM_0 = moment_coefficient_zero(h_t, h, eta_t, St, S, at, it, CM_ac)
    alpha_trim = trim_angle_from_cm(CM_0, CM_alpha)
    
    # Prepare results
    results = {
        'CL': CL_total,
        'CM': CM,
        'CM_alpha': CM_alpha,
        'h_n': h_n,
        'static_margin': SM,
        'alpha_trim': alpha_trim,
        'VH': VH
    }
    
    return results

def directional_stability_analysis(aircraft_params, flight_conditions):
    """
    Perform comprehensive directional stability analysis
    
    Parameters:
    aircraft_params: Dictionary of aircraft parameters
    flight_conditions: Dictionary of flight conditions
    
    Returns:
    results: Dictionary of stability analysis results
    """
    # Extract aircraft parameters
    S = aircraft_params['wing_area']
    b = aircraft_params['wingspan']
    S_v = aircraft_params['vertical_tail_area']
    l_v = aircraft_params['vertical_tail_arm']
    eta_v = aircraft_params.get('vertical_tail_efficiency', 0.9)  # Typical value
    a_v = aircraft_params['vertical_tail_lift_slope']
    d_epsilon_d_beta = aircraft_params.get('d_epsilon_d_beta', 0.1)  # Typical value
    
    # Extract flight conditions
    V = flight_conditions['airspeed']
    rho = flight_conditions['density']
    beta = flight_conditions.get('sideslip', 0)
    
    # Calculate dynamic pressure
    q = 0.5 * rho * V**2
    
    # Calculate vertical tail volume ratio
    V_v = vertical_tail_volume_ratio(S_v, l_v, S, b)
    
    # Calculate yaw moment due to sideslip
    N_v = directional_stability_moment(q, S_v, a_v, beta, d_epsilon_d_beta, l_v)
    
    # Calculate yaw moment coefficient due to sideslip
    CN_beta = yaw_moment_due_to_beta(eta_v, S_v, a_v, S, b, 1.0, d_epsilon_d_beta)  # Using beta=1.0 to get derivative
    
    # Calculate weathercock stability
    weathercock_stability = CN_beta > 0
    
    # Prepare results
    results = {
        'CN_beta': CN_beta,
        'V_v': V_v,
        'weathercock_stability': weathercock_stability
    }
    
    return results

def lateral_control_analysis(aircraft_params, flight_conditions, control_inputs):
    """
    Perform comprehensive lateral control analysis
    
    Parameters:
    aircraft_params: Dictionary of aircraft parameters
    flight_conditions: Dictionary of flight conditions
    control_inputs: Dictionary of control inputs
    
    Returns:
    results: Dictionary of lateral control analysis results
    """
    # Extract aircraft parameters
    S = aircraft_params['wing_area']
    b = aircraft_params['wingspan']
    c_0 = aircraft_params['root_chord']
    lambda_ratio = aircraft_params['taper_ratio']
    a_0 = aircraft_params['section_lift_slope']
    tau_a = aircraft_params.get('aileron_effectiveness', 0.6)  # Typical value
    eta_a = aircraft_params.get('aileron_efficiency', 0.9)  # Typical value
    y1_a = aircraft_params['aileron_inner_location'] * b/2
    y2_a = aircraft_params['aileron_outer_location'] * b/2
    AR = aircraft_params.get('aspect_ratio', b**2/S)
    
    # Extract flight conditions
    V = flight_conditions['airspeed']
    rho = flight_conditions['density']
    
    # Extract control inputs
    delta_a = control_inputs.get('aileron_deflection', 0)
    
    # Calculate dynamic pressure
    q = 0.5 * rho * V**2
    
    # Calculate roll moment due to aileron
    L_aileron = roll_moment_due_to_aileron(q, a_0, tau_a, eta_a, delta_a, y1_a, y2_a)
    
    # Calculate roll coefficient due to aileron
    Cl_delta_a = roll_coefficient_due_to_aileron(q, S, b, a_0, tau_a, eta_a, delta_a, y1_a, y2_a)
    
    # Calculate roll damping coefficient
    p_normalized = 1.0  # Using normalized value to get derivative
    Cl_p = roll_coefficient_due_to_roll_rate(a_0, AR, p_normalized, V, lambda_ratio)
    
    # Calculate steady roll rate
    p_bar = steady_roll_rate(Cl_delta_a/delta_a, Cl_p/p_normalized, delta_a)  # Using derivatives
    p = p_bar * 2 * V / b  # Convert to dimensional roll rate
    
    # Prepare results
    results = {
        'Cl_delta_a': Cl_delta_a/delta_a,  # Derivative
        'Cl_p': Cl_p/p_normalized,  # Derivative
        'steady_roll_rate': p,
        'roll_performance': abs(p) > 0.07  # Typical minimum roll performance requirement
    }
    
    return results

def trim_and_control_analysis(aircraft_params, flight_conditions_1, flight_conditions_2):
    """
    Analyze trim and control requirements for different flight conditions
    
    Parameters:
    aircraft_params: Dictionary of aircraft parameters
    flight_conditions_1: Dictionary of initial flight conditions
    flight_conditions_2: Dictionary of new flight conditions
    
    Returns:
    results: Dictionary of trim and control analysis results
    """
    # Extract aircraft parameters
    S = aircraft_params['wing_area']
    c = aircraft_params['mac']
    St = aircraft_params['tail_area']
    lt = aircraft_params['tail_arm']
    h = aircraft_params['cg_position']
    h_ac = aircraft_params['ac_position']
    h_t = aircraft_params['tail_ac_position']
    it_1 = aircraft_params.get('initial_tail_incidence', 0)
    eta_t = aircraft_params['tail_efficiency']
    aw = aircraft_params['wing_lift_slope']
    at = aircraft_params['tail_lift_slope']
    CM_ac = aircraft_params['cm_ac']
    d_epsilon_d_alpha = aircraft_params.get('d_epsilon_d_alpha', 0.4)  # Typical value
    
    # Extract flight conditions
    V1 = flight_conditions_1['airspeed']
    rho1 = flight_conditions_1['density']
    alpha1 = flight_conditions_1.get('alpha', 0)
    
    V2 = flight_conditions_2['airspeed']
    rho2 = flight_conditions_2.get('density', rho1)  # Use initial density if not specified
    
    # Calculate horizontal tail volume ratio
    VH = horizontal_tail_volume_ratio(St, lt, S, c)
    
    # Calculate CM_alpha
    CM_alpha = moment_coefficient_alpha(h, h_ac, aw, h_t, h, eta_t, St, S, at, d_epsilon_d_alpha)
    
    # Calculate neutral point
    h_n = neutral_point(h_ac, h_t, eta_t, St, S, at, aw, d_epsilon_d_alpha)
    
    # Calculate static margin
    SM = static_margin(h_n, h)
    
    # Calculate new trim angle of attack
    alpha2 = trim_angle_of_attack(alpha1, V1, V2)
    
    # Calculate change in elevator deflection required for trim
    delta_it = elevator_deflection_for_trim(CM_alpha, V1, V2, alpha1, VH)
    
    # Calculate new tail incidence
    it_2 = it_1 + delta_it
    
    # Prepare results
    results = {
        'h_n': h_n,
        'static_margin': SM,
        'alpha_trim_1': alpha1,
        'alpha_trim_2': alpha2,
        'tail_incidence_1': it_1,
        'tail_incidence_2': it_2,
        'delta_tail_incidence': delta_it
    }
    
    return results

def engine_out_analysis(aircraft_params, flight_conditions, engine_params):
    """
    Analyze directional control requirements for engine-out condition
    
    Parameters:
    aircraft_params: Dictionary of aircraft parameters
    flight_conditions: Dictionary of flight conditions
    engine_params: Dictionary of engine parameters
    
    Returns:
    results: Dictionary of engine-out analysis results
    """
    # Extract aircraft parameters
    S = aircraft_params['wing_area']
    b = aircraft_params['wingspan']
    S_v = aircraft_params['vertical_tail_area']
    l_v = aircraft_params['vertical_tail_arm']
    eta_v = aircraft_params.get('vertical_tail_efficiency', 0.9)  # Typical value
    a_v = aircraft_params['vertical_tail_lift_slope']
    tau_r = aircraft_params.get('rudder_effectiveness', 0.6)  # Typical value
    d_epsilon_d_beta = aircraft_params.get('d_epsilon_d_beta', 0.1)  # Typical value
    
    # Extract flight conditions
    V = flight_conditions['airspeed']
    rho = flight_conditions['density']
    
    # Extract engine parameters
    T = engine_params['thrust']
    y_e = engine_params['engine_offset']  # Distance from centerline
    
    # Calculate dynamic pressure
    q = 0.5 * rho * V**2
    
    # Calculate engine-out yawing moment
    T_ye = T * y_e
    
    # Calculate yaw moment coefficient due to rudder
    CN_delta_r = -eta_v * (S_v/S) * a_v * tau_r * (l_v/b)
    
    # Calculate required rudder deflection
    delta_r = rudder_deflection_for_engine_out(T_ye, CN_delta_r, q, S, b)
    
    # Check if rudder is sufficient
    max_rudder = aircraft_params.get('max_rudder_deflection', np.radians(25))  # Typical max deflection
    rudder_sufficient = abs(delta_r) <= max_rudder
    
    # Prepare results
    results = {
        'engine_out_yaw_moment': T_ye,
        'CN_delta_r': CN_delta_r,
        'required_rudder_deflection': delta_r,
        'rudder_sufficient': rudder_sufficient,
        'rudder_margin': (max_rudder - abs(delta_r)) / max_rudder * 100  # Percentage
    }
    
    return results

def plot_stability_derivatives(aircraft_params, flight_conditions, alpha_range):
    """
    Plot stability derivatives over a range of angles of attack
    
    Parameters:
    aircraft_params: Dictionary of aircraft parameters
    flight_conditions: Dictionary of flight conditions
    alpha_range: Array of angle of attack values to analyze
    
    Returns:
    fig: Matplotlib figure object with stability plots
    """
    # Initialize arrays to store results
    CL_values = []
    CM_values = []
    CM_alpha_values = []
    h_n_values = []
    SM_values = []
    
    # Loop through alpha range
    for alpha in alpha_range:
        # Update flight conditions with current alpha
        current_conditions = flight_conditions.copy()
        current_conditions['alpha'] = alpha
        
        # Perform stability analysis
        results = longitudinal_stability_analysis(aircraft_params, current_conditions)
        
        # Store results
        CL_values.append(results['CL'])
        CM_values.append(results['CM'])
        CM_alpha_values.append(results['CM_alpha'])
        h_n_values.append(results['h_n'])
        SM_values.append(results['static_margin'])
    
    # Create figure for plotting
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))
    
    # Plot CL vs alpha
    axs[0, 0].plot(alpha_range * 180/np.pi, CL_values)
    axs[0, 0].set_xlabel('Angle of Attack (degrees)')
    axs[0, 0].set_ylabel('CL')
    axs[0, 0].set_title('Lift Coefficient vs Alpha')
    axs[0, 0].grid(True)
    
    # Plot CM vs alpha
    axs[0, 1].plot(alpha_range * 180/np.pi, CM_values)
    axs[0, 1].set_xlabel('Angle of Attack (degrees)')
    axs[0, 1].set_ylabel('CM')
    axs[0, 1].set_title('Moment Coefficient vs Alpha')
    axs[0, 1].grid(True)
    
    # Plot CM vs CL
    axs[1, 0].plot(CL_values, CM_values)
    axs[1, 0].set_xlabel('CL')
    axs[1, 0].set_ylabel('CM')
    axs[1, 0].set_title('Moment Coefficient vs Lift Coefficient')
    axs[1, 0].grid(True)
    
    # Plot Static Margin vs alpha
    axs[1, 1].plot(alpha_range * 180/np.pi, SM_values)
    axs[1, 1].set_xlabel('Angle of Attack (degrees)')
    axs[1, 1].set_ylabel('Static Margin')
    axs[1, 1].set_title('Static Margin vs Alpha')
    axs[1, 1].grid(True)
    
    plt.tight_layout()
    
    return fig

def analyze_aircraft_stability(aircraft_name, aircraft_params, flight_conditions):
    """
    Perform a comprehensive stability and control analysis for an aircraft
    
    Parameters:
    aircraft_name: Name of the aircraft
    aircraft_params: Dictionary of aircraft parameters
    flight_conditions: Dictionary of flight conditions
    
    Returns:
    report: Dictionary with comprehensive stability analysis results
    """
    # Perform longitudinal stability analysis
    long_stability = longitudinal_stability_analysis(aircraft_params, flight_conditions)
    
    # Perform directional stability analysis
    dir_stability = directional_stability_analysis(aircraft_params, flight_conditions)
    
    # Set up control inputs for lateral analysis
    control_inputs = {
        'aileron_deflection': np.radians(20)  # 20 degrees aileron deflection for analysis
    }
    
    # Perform lateral control analysis
    lat_control = lateral_control_analysis(aircraft_params, flight_conditions, control_inputs)
    
    # Set up engine parameters for engine-out analysis
    engine_params = {
        'thrust': aircraft_params.get('max_thrust', 0),
        'engine_offset': aircraft_params.get('engine_offset', 0)
    }
    
    # Perform engine-out analysis
    eng_out = engine_out_analysis(aircraft_params, flight_conditions, engine_params)
    
    # Create a second flight condition at 1.3*Vs for trim analysis
    flight_conditions_2 = flight_conditions.copy()
    flight_conditions_2['airspeed'] = flight_conditions['airspeed'] * 1.3
    
    # Perform trim and control analysis
    trim_control = trim_and_control_analysis(aircraft_params, flight_conditions, flight_conditions_2)
    
    # Create comprehensive report
    report = {
        'aircraft_name': aircraft_name,
        'longitudinal_stability': long_stability,
        'directional_stability': dir_stability,
        'lateral_control': lat_control,
        'engine_out': eng_out,
        'trim_control': trim_control
    }
    
    # Add stability assessment
    report['stability_assessment'] = {
        'longitudinally_stable': long_stability['CM_alpha'] < 0,
        'directionally_stable': dir_stability['CN_beta'] > 0,
        'sufficient_roll_control': lat_control['roll_performance'],
        'sufficient_rudder_control': eng_out['rudder_sufficient']
    }
    
    # Overall stability assessment
    report['overall_stable'] = all(report['stability_assessment'].values())
    
    return report

# Additional utility functions for complete aircraft analysis

def finite_wing_correction(a_0, AR, sweep_angle=0, k_e=0.9):
    """
    Calculate finite wing lift curve slope from section lift curve slope
    
    Parameters:
    a_0: Section lift curve slope (typically 2π)
    AR: Aspect ratio
    sweep_angle: Wing sweep angle (radians)
    k_e: Empirical factor (typically 0.9)
    
    Returns:
    a: Finite wing lift curve slope
    """
    # Corrected for sweep
    AR_eff = AR * np.cos(sweep_angle)
    
    # Finite wing correction
    a = a_0 / (1 + a_0/(np.pi * AR_eff * k_e))
    
    return a

def downwash_gradient(AR, sweep_angle=0, k_e=0.3):
    """
    Calculate downwash gradient at the tail
    
    Parameters:
    AR: Wing aspect ratio
    sweep_angle: Wing sweep angle (radians)
    k_e: Empirical factor (typically 0.3)
    
    Returns:
    d_epsilon_d_alpha: Downwash gradient
    """
    # Corrected for sweep
    AR_eff = AR * np.cos(np.radians(sweep_angle))
    
    # Calculate downwash gradient
    d_epsilon_d_alpha = k_e * (2 / (2 + AR_eff))
    
    return d_epsilon_d_alpha

def dynamic_presure_ratio(x_t, wing_height, h_t):
    """
    Calculate dynamic pressure ratio at the tail
    
    Parameters:
    x_t: Tail distance from wing trailing edge
    wing_height: Wing height above tail
    h_t: Tail height
    
    Returns:
    eta_t: Dynamic pressure ratio at tail
    """
    # Simplified model for propeller aircraft
    # For detailed analysis, would need wake and boundary layer effects
    wake_effect = np.exp(-0.1 * x_t / wing_height)
    height_effect = 1.0  # Could model ground effect if needed
    
    eta_t = (1 - wake_effect) * height_effect
    
    return eta_t

def calculate_control_surface_effectiveness(cf_c, seal_type="sealed"):
    """
    Calculate control surface effectiveness parameter
    
    Parameters:
    cf_c: Ratio of control surface chord to airfoil chord
    seal_type: Type of hinge seal ("sealed", "unsealed")
    
    Returns:
    tau: Control surface effectiveness parameter
    """
    # Empirical formulation based on NACA data
    if seal_type == "sealed":
        tau = 1.0 - 0.8 * (1 - cf_c)
    else:  # unsealed
        tau = 0.8 - 0.7 * (1 - cf_c)
    
    return tau

aircraft_params = dict(
    # Linear dimensions given in Feet
    wing_area = 9360,
    wingspan = 315,
    #aspect_ratio = aspect_ratio(wingspan, wing_area),
    root_chord = 49.53,
    tip_chord = 9.91,
    #lambda_ratio = taper_ratio(tip_chord, root_chord),
    #taper_ratio = lambda_ratio,
    sweep_angle = 33, # Degrees
    #mac = mean_aerodynamic_chord(root_chord,lambda_ratio),
    tail_area = 9360.8, 
    #tail_arm = , # Definition
    #cg_position = , # User-defined
    #ac_position = , # Definition
    #tail_ac_position = ,
    #initial_tail_incidence = , # Default 0
    #tail_incidence = ,
    #tail_efficiency = ,
    #wing_lift_slope = .33, 
    #tail_lift_slope = 0.18,
    #cm_ac = ,
    #d_epsilon_d_alpha = , # Necessary to split for htail & wing
    #section_lift_slope = ,
    #aileron_effectiveness = ,
    #aileron_efficiency = ,
    #aileron_inner_location = ,
    #aileron_outer_location = ,
    #vertical_tail_area = 1861,
    #vertical_tail_arm = ,
    #vertical_tail_efficiency = ,
    #vertical_tail_lift_slope = 0.11,
    #epsilon_0 = ,
    #max_rudder_deflection = 25, # Degrees
)

class Aircraft_params:
    def __init__(self, wing_area, wingspan, root_chord, tip_chord, tail_area, tail_arm, cg_position, sweep_angle):
        self.wing_area = wing_area
        self.wingspan = wingspan
        self.root_chord = root_chord
        self.tip_chord = tip_chord
        self.tail_area = tail_area
        self.lambda_ratio = taper_ratio(tip_chord, root_chord)
        self.aspect_ratio = aspect_ratio(wingspan, wing_area)
        self.lambda_ratio = taper_ratio(tip_chord, root_chord)
        self.mac = mean_aerodynamic_chord(root_chord, self.lambda_ratio)
        self.tail_arm = tail_arm
        self.cg_position = cg_position
        self.sweep_angle = sweep_angle
        self.ac_position = (0.25*root_chord + distance_AC_behind_quarter_chord(self.lambda_ratio, self.wingspan, self.sweep_angle)) / self.mac # Defines Hac, distance of AC behind LE of MAC
        
        
        
aircraft_params = Aircraft_params(
    wing_area = 9360,
    wingspan = 315,
    root_chord = 49.53,
    tip_chord = 9.91,
    tail_area = 2239,
    tail_arm = None, # Distance Tail-to-CG, design-defined
    cg_position = None, # Design-defined
    sweep_angle = 34, # Degrees
    
    
)


flight_conditions = dict(
    airspeed = .9 * 968.7,
    density = 0.0007103,
    alpha = 3,
    sideslip = 0,
)
    

if __name__ == "__main__":


    print(longitudinal_stability_analysis(aircraft_params,flight_conditions))
    print("yay")