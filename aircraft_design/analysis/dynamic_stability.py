import numpy as np
from aircraft_design.final_design.final_construction import Aircraft



def calculate_derivatives(plane_dict, aircraft: Aircraft):
    """
    Calculate all thirty aerodynamic stability and control coefficients for an aircraft.
    
    Parameters:
    -----------
    plane_dict : dict
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
    V0 = plane_dict.get('V0',0) # v0 only matters if velocity derivatives are non-zero
    S = plane_dict['S'] # from aircraft params
    b = plane_dict['b'] # from aircraft params
    c_bar = plane_dict['c_bar'] # from aircraft params
    c_0 = plane_dict['c_0'] # from aircraft params
    St = plane_dict['St'] # from aircraft params
    lt = plane_dict['lt'] # from aircraft params
    at = plane_dict['at'] # from aircraft params
    Sv = plane_dict['Sv'] # from aircraft params
    lv = plane_dict['lv'] # from aircraft params
    zv = plane_dict['zv'] # from aircraft params
    av = plane_dict['av'] # from aircraft params
    bv = plane_dict['bv'] # from aircraft params
    taper_ratio = plane_dict['taper_ratio'] # from aircraft params
    Gamma = plane_dict['Gamma'] # from aircraft params
    Lambda = plane_dict['Lambda'] # from aircraft params
  
    Tau_e = plane_dict['Tau_e'] # from aircraft params
    eta_e = plane_dict['eta_e'] # from aircraft params
    Tau_r = plane_dict['Tau_r'] # from aircraft params
    eta_r = plane_dict['eta_r'] # from aircraft params
    Tau_a = plane_dict['Tau_a'] # from aircraft params
    eta_a = plane_dict['eta_a'] # from aircraft params
    CL = plane_dict['CL'] # from static analysis
    CD = plane_dict['CD'] # from static analysis
    dCL_da = plane_dict['dCL_da'] # from static analysis
    dCD_da = plane_dict['dCD_da'] # from static analysis
    static_margin = plane_dict['static_margin']

    
    # Calculate volume ratios
    VH = (St * lt) / (S * c_bar)  # Horizontal tail volume ratio
    Vv = (Sv * lv) / (S * b)      # Vertical tail volume ratio
    
    # Calculate coefficients
    coefficients = {}
    
    # Longitudinal Stability Coefficients
    
    # X_u - Axial force due to axial velocity
    coefficients['C_X_u'] = -CD - V0 * plane_dict.get('dCD_dV', 0)  # Include derivative term
    
    # Z_u - Normal force due to axial velocity
    coefficients['C_Z_u'] = -CL - V0 * plane_dict.get('dCL_dV', 0)
    
    # X_w - Axial force due to normal velocity
    coefficients['C_X_alpha'] = CL - dCD_da
    
    # Z_w - Normal force due to normal velocity
    coefficients['C_Z_alpha'] = -dCL_da - CD
    
    # M_u - Pitching moment due to axial velocity
    coefficients['C_M_u'] = V0 * plane_dict.get('dCM_dV', 0)  # Not negligible if specified
    
    # M_w - Pitching moment due to normal velocity
    coefficients['C_M_alpha'] = plane_dict.get('dCM_da', -dCL_da * (static_margin))
    
    # X_q - Axial force due to pitch rate
    # Not negligible if tail drag is significant
    dCD_t_da_t = plane_dict.get('dCD_t_da_t', 0)
    if dCD_t_da_t != 0:
        coefficients['C_X_q'] = -VH * dCD_t_da_t * 2
    else:
        coefficients['C_X_q'] = 0
    
    # Z_q - Normal force due to pitch rate
    coefficients['C_Z_q'] = -VH * at * 2
    
    # M_q - Pitching moment due to pitch rate
    coefficients['C_M_q'] = -VH * (lt / c_bar) * at * 2
    
    # X_w_dot - Axial force due to rate of change of normal velocity
    de_da = plane_dict.get('de_da', 0.4)  # Get downwash factor from input or use typical value
    
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
    trap_y = np.linspace(0, b/2, 50)
    trap_c = aircraft.wing.geometry.get_chord_at_span(trap_y/b) * trap_y

    if Gamma > 0:
        C_l_beta_dihedral = -2 / (b*S) * dCL_da * Gamma * np.trapz(trap_c, trap_y)
    if Lambda > 0:
        C_l_beta_sweep = -4*CL*np.tan(Lambda) / (b*S) * np.trapz(trap_c, trap_y)
    C_l_beta_vtail = -Vv * zv/lv * av
    
    coefficients['C_l_beta'] = C_l_beta_dihedral + C_l_beta_sweep + C_l_beta_vtail
    
    # N_v - Yawing moment due to sideslip
    coefficients['C_N_beta'] = Vv * av
    
    # Y_p - Side force due to roll rate
    c_v = plane_dict.get('c_v', 0)
    if bv > 0: # vertical tail is linearly tapered, so we can use the formula for a tapered wing
        coefficients['C_Y_p'] = -c_v/(S*b) * av * bv**2 * (1-(1-taper_ratio)/3) # getting rid of a factor of 1/2 for the 1/4 in the coefficient conversion
    else:
        coefficients['C_Y_p'] = 0  # Often negligible
    
    # L_p - Rolling moment due to roll rate
    # Simplified integration for wing contribution
    trap_c2 = trap_c * trap_y
    coefficients['C_l_p'] = -4 / (S*b**2) * (dCL_da + CD) * np.trapz(trap_c2, trap_y)
    
    # N_p - Yawing moment due to roll rate
    # Simplified integration for wing contribution
    coefficients['C_N_p'] = -4 / (S*b**2) * (CL - dCD_da) * np.trapz(trap_c2, trap_y)
    
    # Y_r - Side force due to yaw rate
    coefficients['C_Y_r'] = Vv * av
    
    # L_r - Rolling moment due to yaw rate
    # Wing and vertical tail contributions

    C_l_r_wing = 8 / (S*b**2) * np.trapz(trap_c2, trap_y)
    C_l_r_vtail = Vv * zv/b * av
    coefficients['C_l_r'] = C_l_r_wing + C_l_r_vtail
    
    # N_r - Yawing moment due to yaw rate
    # Wing and vertical tail contributions
    C_N_r_wing = -8 / (S*b**2) * np.trapz(trap_c2, trap_y)
    C_N_r_vtail = -Vv * lv/b * av
    coefficients['C_N_r'] = C_N_r_wing + C_N_r_vtail
    
    # Aerodynamic Control Coefficients
    
    # X_eta - Axial force due to elevator
    # Not negligible if tail produces significant drag
    C_L_t = plane_dict.get('CL_t', 0)
    e_t = plane_dict.get('e_t', 0.8)  # Typical tail efficiency factor
    A_t = plane_dict.get('A_t', 4)    # Typical tail aspect ratio
    
    if C_L_t != 0:
        coefficients['C_X_delta'] = -2 * St/S * C_L_t/(np.pi*e_t*A_t) * at * Tau_e * eta_e
    else:
        coefficients['C_X_delta'] = 0
    
    # Z_eta - Normal force due to elevator
    coefficients['C_Z_delta'] = -St/S * at * Tau_e * eta_e
    
    # M_eta - Pitching moment due to elevator
    coefficients['C_M_delta'] = -VH * at * Tau_e * eta_e
    
    # Y_xi - Side force due to aileron
    coefficients['C_Y_delta_a'] = 0  # Negligible for conventional aircraft
    
    # L_xi - Rolling moment due to aileron
    # Simplified integration for aileron effect
    y1 = plane_dict.get('aileron_inner_location', aircraft.wing.aileron_start * b/2)  # Aileron start location
    y2 = plane_dict.get('aileron_outer_location', aircraft.wing.aileron_end * b/2)  # Aileron end location
    y_aileron = np.linspace(y1, y2, 50)
    trap_c_a = aircraft.wing.geometry.get_chord_at_span(y_aileron/b) * y_aileron
    coefficients['C_l_delta_a'] = -2/(S*b) * dCL_da * Tau_a * eta_a * np.trapz(trap_c_a, y_aileron)
    
    # N_xi - Yawing moment due to aileron (adverse yaw)
    # Modeled more accurately using partial derivative of CD with respect to aileron deflection
    dCD_dxi = plane_dict.get('dCD_dxi', 0.1 * dCL_da * Tau_a * eta_a)  # Typical estimation
    
    coefficients['C_N_delta_a'] = 2/(S*b) * dCD_dxi * np.trapz(trap_c_a, y_aileron)
    
    # Y_zeta - Side force due to rudder
    coefficients['C_Y_delta_r'] = Sv/S * av * Tau_r * eta_r
    
    # L_zeta - Rolling moment due to rudder
    coefficients['C_l_delta_r'] = Vv * zv/lv * av * Tau_r * eta_r
    
    # N_zeta - Yawing moment due to rudder
    coefficients['C_N_delta_r'] = -Vv * av * Tau_r * eta_r
    
    return coefficients