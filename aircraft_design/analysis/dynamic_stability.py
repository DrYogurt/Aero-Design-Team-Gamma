import numpy as np




def calculate_derivatives(plane):
    """
    Calculate all thirty aerodynamic stability and control coefficients for an aircraft.
    
    Parameters:
    -----------
    plane : dict
        Dictionary containing aircraft parameters:
        - V0: equilibrium flight velocity [m/s]
        - rho: air density [kg/m^3]
        - S: wing planform area [m^2]
        - b: wingspan [m]
        - c_bar: mean aerodynamic chord [m]
        - CL: aircraft lift coefficient at equilibrium
        - CD: aircraft drag coefficient at equilibrium
        - dCL_da: aircraft lift curve slope [1/rad]
        - dCD_da: change in drag coefficient with angle of attack [1/rad]
        - dCL_dV: change in lift coefficient with velocity [1/(m/s)]
        - dCD_dV: change in drag coefficient with velocity [1/(m/s)]
        - dCM_dV: change in moment coefficient with velocity [1/(m/s)]
        - dCM_da: change in moment coefficient with angle of attack [1/rad]
        - de_da: downwash factor
        - h_n: neutral point location as fraction of c_bar
        - h: center of gravity location as fraction of c_bar
        - St: horizontal tail planform area [m^2]
        - lt: distance from CG to horizontal tail aerodynamic center [m]
        - at: horizontal tail lift curve slope [1/rad]
        - Sv: vertical tail planform area [m^2]
        - lv: distance from CG to vertical tail aerodynamic center [m]
        - zv: vertical distance from CG to vertical tail [m]
        - av: vertical tail lift curve slope [1/rad]
        - bv: vertical tail span/height [m]
        - Gamma: wing dihedral angle [rad]
        - Lambda: wing sweep angle [rad]
        - Tau_f: flap effectiveness factor
        - eta_f: flap correction factor
        
    Returns:
    --------
    dict
        Dictionary containing all thirty stability and control coefficients
    """
    # Extract parameters
    V0 = plane.get('V0',0) # v0 only matters if velocity derivatives are non-zero
    S = plane['S'] # from aircraft params
    b = plane['b'] # from aircraft params
    c_bar = plane['c_bar'] # from aircraft params
    c_0 = plane['c_0'] # from aircraft params
    St = plane['St'] # from aircraft params
    lt = plane['lt'] # from aircraft params
    at = plane['at'] # from aircraft params
    Sv = plane['Sv'] # from aircraft params
    lv = plane['lv'] # from aircraft params
    zv = plane['zv'] # from aircraft params
    av = plane['av'] # from aircraft params
    bv = plane['bv'] # from aircraft params
    taper_ratio = plane['taper_ratio'] # from aircraft params
    Gamma = plane['Gamma'] # from aircraft params
    Lambda = plane['Lambda'] # from aircraft params
    Tau_f = plane['Tau_f'] # from aircraft params
    eta_f = plane['eta_f'] # from aircraft params
    CL = plane['CL'] # from static analysis
    CD = plane['CD'] # from static analysis
    dCL_da = plane['dCL_da'] # from static analysis
    dCD_da = plane['dCD_da'] # from static analysis
    static_margin = plane['static_margin']

    
    # Calculate volume ratios
    VH = (St * lt) / (S * c_bar)  # Horizontal tail volume ratio
    Vv = (Sv * lv) / (S * b)      # Vertical tail volume ratio
    
    # Calculate coefficients
    coefficients = {}
    
    # Longitudinal Stability Coefficients
    
    # X_u - Axial force due to axial velocity
    coefficients['C_X_u'] = -CD - V0 * plane.get('dCD_dV', 0)  # Include derivative term
    
    # Z_u - Normal force due to axial velocity
    coefficients['C_Z_u'] = -CL - V0 * plane.get('dCL_dV', 0)
    
    # X_w - Axial force due to normal velocity
    coefficients['C_X_alpha'] = CL - dCD_da
    
    # Z_w - Normal force due to normal velocity
    coefficients['C_Z_alpha'] = -dCL_da - CD
    
    # M_u - Pitching moment due to axial velocity
    coefficients['C_M_u'] = V0 * plane.get('dCM_dV', 0)  # Not negligible if specified
    
    # M_w - Pitching moment due to normal velocity
    coefficients['C_M_alpha'] = plane.get('dCM_da', -dCL_da * (static_margin))
    
    # X_q - Axial force due to pitch rate
    # Not negligible if tail drag is significant
    dCD_t_da_t = plane.get('dCD_t_da_t', 0)
    if dCD_t_da_t != 0:
        coefficients['C_X_q'] = -VH * dCD_t_da_t * 2
    else:
        coefficients['C_X_q'] = 0
    
    # Z_q - Normal force due to pitch rate
    coefficients['C_Z_q'] = -VH * at * 2
    
    # M_q - Pitching moment due to pitch rate
    coefficients['C_M_q'] = -VH * (lt / c_bar) * at * 2
    
    # X_w_dot - Axial force due to rate of change of normal velocity
    de_da = plane.get('de_da', 0.4)  # Get downwash factor from input or use typical value
    
    # Not negligible if tail drag and downwash are significant
    if dCD_t_da_t != 0 and de_da != 0:
        coefficients['C_X_alpha_dot'] = -VH * dCD_t_da_t * de_da * 2
    else:
        coefficients['C_X_alpha_dot'] = 0
    
    # Z_w_dot - Normal force due to rate of change of normal velocity
    coefficients['C_Z_alpha_dot'] = -VH * at * de_da * 2
    
    # M_w_dot - Pitching moment due to rate of change of normal velocity
    coefficients['C_M_alpha_dot'] = -VH * (lt / c_bar) * at * de_da * 2
    
    # Lateral-Directional Stability Coefficients
    
    # Y_v - Side force due to sideslip
    coefficients['C_Y_beta'] = -Sv/S * av
    
    # L_v - Rolling moment due to sideslip
    # Combined effects of dihedral, sweep, and vertical tail
    C_l_beta_dihedral = 0
    C_l_beta_sweep = 0
    C_l_beta_vtail = 0
    if Gamma > 0:
        C_l_beta_dihedral = dCL_da * Gamma * c_0 * b * (1-(1-taper_ratio)/3) / S
    if Lambda > 0:
        C_l_beta_sweep = -2*CL*np.tan(Lambda)* b *c_0 / S * (1-(1-taper_ratio)/3)
    C_l_beta_vtail = -Vv * zv/lv * av
    
    coefficients['C_l_beta'] = C_l_beta_dihedral + C_l_beta_sweep + C_l_beta_vtail
    
    # N_v - Yawing moment due to sideslip
    coefficients['C_N_beta'] = Vv * av
    
    # Y_p - Side force due to roll rate
    c_v = plane.get('c_v', 0)
    if bv > 0:
        coefficients['C_Y_p'] = -c_v/(S*b) * av * bv**2 * (1-(1-taper_ratio)/3) # getting rid of a factor of 1/2 for the 1/4 in the coefficient conversion
    else:
        coefficients['C_Y_p'] = 0  # Often negligible
    
    # L_p - Rolling moment due to roll rate
    # Simplified integration for wing contribution
    
    coefficients['C_l_p'] = -0.5 * (dCL_da + CD) * c_0 * b * (1-(1-taper_ratio)/4) / S
    
    # N_p - Yawing moment due to roll rate
    # Simplified integration for wing contribution
    coefficients['C_N_p'] = -0.5 * (CL - dCD_da) * c_0 * b * (1-(1-taper_ratio)/4) / S
    
    # Y_r - Side force due to yaw rate
    coefficients['C_Y_r'] = Vv * av
    
    # L_r - Rolling moment due to yaw rate
    # Wing and vertical tail contributions
    C_l_r_wing = 2 * CL * c_0 * b * (1-(1-taper_ratio)/4) / S
    C_l_r_vtail = Vv * zv/b * av
    coefficients['C_l_r'] = C_l_r_wing + C_l_r_vtail
    
    # N_r - Yawing moment due to yaw rate
    # Wing and vertical tail contributions
    C_N_r_wing = -2 * CD * c_0 * b * (1-(1-taper_ratio)/4) / S
    C_N_r_vtail = -Vv * lv/b * av
    coefficients['C_N_r'] = C_N_r_wing + C_N_r_vtail
    
    # Aerodynamic Control Coefficients
    
    # X_eta - Axial force due to elevator
    # Not negligible if tail produces significant drag
    C_L_t = plane.get('CL_t', 0)
    e_t = plane.get('e_t', 0.8)  # Typical tail efficiency factor
    A_t = plane.get('A_t', 4)    # Typical tail aspect ratio
    
    if C_L_t != 0:
        coefficients['C_X_delta'] = -2 * St/S * C_L_t/(np.pi*e_t*A_t) * at * Tau_f * eta_f
    else:
        coefficients['C_X_delta'] = 0
    
    # Z_eta - Normal force due to elevator
    coefficients['C_Z_delta'] = -St/S * at * Tau_f * eta_f
    
    # M_eta - Pitching moment due to elevator
    coefficients['C_M_delta'] = -VH * at * Tau_f * eta_f
    
    # Y_xi - Side force due to aileron
    coefficients['C_Y_delta_a'] = 0  # Negligible for conventional aircraft
    
    # L_xi - Rolling moment due to aileron
    # Simplified integration for aileron effect
    y1 = plane.get('aileron_inner_location', 0.5 * b/2)  # Aileron start location
    y2 = plane.get('aileron_outer_location', 0.9 * b/2)  # Aileron end location
    y_aileron = np.linspace(y1, y2, 50)
    c_int = lambda y: c_0 * (y**2/2-(1-taper_ratio)* y**3 / (b/2) / 3)
    coefficients['C_l_delta_a'] = -2/(S*b) * dCL_da * Tau_f * eta_f * (c_int(y2) - c_int(y1))
    
    # N_xi - Yawing moment due to aileron (adverse yaw)
    # Modeled more accurately using partial derivative of CD with respect to aileron deflection
    dCD_dxi = plane.get('dCD_dxi', 0.1 * dCL_da * Tau_f * eta_f)  # Typical estimation
    
    coefficients['C_N_delta_a'] = 2/(S*b) * dCD_dxi * (c_int(y2) - c_int(y1))
    
    # Y_zeta - Side force due to rudder
    coefficients['C_Y_delta_r'] = Sv/S * av * Tau_f * eta_f
    
    # L_zeta - Rolling moment due to rudder
    coefficients['C_l_delta_r'] = Vv * zv/lv * av * Tau_f * eta_f
    
    # N_zeta - Yawing moment due to rudder
    coefficients['C_N_delta_r'] = -Vv * av * Tau_f * eta_f
    
    return coefficients