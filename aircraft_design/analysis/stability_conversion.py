import numpy as np
def convert_stability_to_aero_derivatives(stability_derivatives, S, c_bar, b, rho, V0):
    """
    Convert stability derivatives to aerodynamic derivatives.
    
    Parameters:
        stability_derivatives: dict
            Stability derivatives.

    Returns:
        aerodynamic_derivatives: dict
            Aerodynamic derivatives.
    """
    q_dyn = 0.5 * rho * V0

    dimensional_params = {
    "X_u": stability_derivatives["C_X_u"] * q_dyn * S,
    "Z_u": stability_derivatives["C_Z_u"] * q_dyn * S,
    "M_u": stability_derivatives["C_M_u"] * q_dyn * S * c_bar,
    "X_w": stability_derivatives["C_X_alpha"] * q_dyn * S,
    "Z_w": stability_derivatives["C_Z_alpha"] * q_dyn * S,
    "M_w": stability_derivatives["C_M_alpha"] * q_dyn * S * c_bar,
    "X_q": stability_derivatives["C_X_q"] * q_dyn * S * c_bar * 0.5,
    "Z_q": stability_derivatives["C_Z_q"] * q_dyn * S * c_bar *0.5,
    "M_q": stability_derivatives["C_M_q"] * q_dyn * S * c_bar**2 * 0.5,
    "X_wdot": stability_derivatives["C_X_alpha_dot"] * 0.5*rho * S * c_bar * 0.5,
    "Z_wdot": stability_derivatives["C_Z_alpha_dot"] * 0.5*rho * S * c_bar * 0.5,
    "M_wdot": stability_derivatives["C_M_alpha_dot"] * 0.5*rho * S * c_bar**2 * 0.5,
    "Y_v": stability_derivatives["C_Y_beta"] * q_dyn * S,
    "L_v": stability_derivatives["C_l_beta"] * q_dyn * S * b,
    "N_v": stability_derivatives["C_N_beta"] * q_dyn * S * b,
    "Y_p": stability_derivatives["C_Y_p"] * q_dyn * S * b * 0.5,
    "L_p": stability_derivatives["C_l_p"] * q_dyn * S * b**2 * 0.5,
    "N_p": stability_derivatives["C_N_p"] * q_dyn * S * b**2 * 0.5,
    "Y_r": stability_derivatives["C_Y_r"] * q_dyn * S * b * 0.5,
    "L_r": stability_derivatives["C_l_r"] * q_dyn * S * b**2 * 0.5,
    "N_r": stability_derivatives["C_N_r"] * q_dyn * S * b**2 * 0.5,
    "X_eta": stability_derivatives["C_X_delta"] * q_dyn * S * V0,
    "Z_eta": stability_derivatives["C_Z_delta"] * q_dyn * S * V0,
    "M_eta": stability_derivatives["C_M_delta"] * q_dyn * S * c_bar * V0,
    "Y_xi": stability_derivatives["C_Y_delta_a"] * q_dyn * S * V0,
    "L_xi": stability_derivatives["C_l_delta_a"] * q_dyn * S * V0 * b,
    "N_xi": stability_derivatives["C_N_delta_a"] * q_dyn * S * b,
    "Y_zeta": stability_derivatives["C_Y_delta_r"] * q_dyn * S * V0,
    "L_zeta": stability_derivatives["C_l_delta_r"] * q_dyn * S  * b * V0,
    "N_zeta": stability_derivatives["C_N_delta_r"] * q_dyn * S  * b * V0,    
}

    return dimensional_params

def longitudinal_matrices(dimensional_params, aircraft_data):
    """
    Convert dimensional parameters to arrays.
        """
    # Auxiliary terms for stability derivatives
    g = 32.174  # ft/s^2
    mass = aircraft_data['mass']  # aircraft mass
    Iy = aircraft_data['Iy']  # moment of inertia around y-axis
    aux_denominator = mass - dimensional_params["Z_wdot"]
    # A matrix for longitudinal dynamics
    a_mat = np.array([
        [
            dimensional_params["X_u"] / mass + dimensional_params["X_wdot"] * dimensional_params["Z_u"] / mass / aux_denominator,
            dimensional_params["X_w"] / mass + dimensional_params["X_wdot"] * dimensional_params["Z_w"] / mass / aux_denominator,
            (dimensional_params["X_q"] - mass*aircraft_data['We']) / mass + (dimensional_params['Z_q'] + mass * aircraft_data['Ue']) * dimensional_params['X_wdot'] / mass / aux_denominator,
            -g*np.cos(aircraft_data['theta_e']) - dimensional_params['X_wdot'] * g * np.sin(aircraft_data['theta_e']) / aux_denominator
        ],
        [
            dimensional_params["Z_u"] / aux_denominator,
            dimensional_params["Z_w"] / aux_denominator,
            (dimensional_params["Z_q"] + mass * aircraft_data['Ue']) / aux_denominator,
            -mass * g *np.sin(aircraft_data['theta_e']) / aux_denominator
        ],
        [
            (dimensional_params["M_u"] + dimensional_params["M_wdot"] * dimensional_params["Z_u"] / aux_denominator) / Iy,
            (dimensional_params["M_w"] + dimensional_params["M_wdot"] * dimensional_params["Z_w"] / aux_denominator) / Iy,
            (dimensional_params["M_q"] + dimensional_params["M_wdot"] * (dimensional_params["Z_q"] + mass * aircraft_data['Ue']) / aux_denominator) / Iy,
            dimensional_params['M_wdot'] * mass * g *np.sin(aircraft_data['theta_e']) / aux_denominator / Iy
            
        ],
        [0, 0, 1, 0]
    ])

    # B matrix for longitudinal control dynamics
    b_mat = np.array([
        [
            (dimensional_params["X_eta"] + dimensional_params["X_wdot"] * dimensional_params["Z_eta"] / aux_denominator) / mass,
            0
        ],
        [
            dimensional_params["Z_eta"] / aux_denominator,
            0
        ],
        [
            (dimensional_params["M_eta"] + dimensional_params["M_wdot"] * dimensional_params["Z_eta"] / aux_denominator) / Iy,
            0
        ],
        [0, 0]
    ]).T

    # Time parameters for simulation
    eta = 0.0175  # Elevator deflection in radians
    tau = 0.0  # Thrust variation
    ku = [0.0, 0.0]
    kw = [0.0, 0.0]
    kq = [0.0, 0.0]
    ko = [0.0, 0.0]

    return a_mat, b_mat, eta, tau, ku, kw, kq, ko







def lateral_directional_matrices(dimensional_params, aircraft_data):
    """
    Convert dimensional parameters to arrays.
    """

    # Assuming you've already calculated dimensional_params using convert_stability_to_aero_derivatives

    # Auxiliary calculations
    g = 32.174  # gravity acceleration, m/s²
    mass = aircraft_data['mass']  # aircraft mass
    Ix = aircraft_data['Ix']  # moment of inertia around x-axis
    Iz = aircraft_data['Iz']  # moment of inertia around z-axis
    Ixz = aircraft_data['Ixz']  # product of inertia

    # A matrix for lateral-directional dynamics
    a_mat = np.array([
        [
            dimensional_params["Y_v"] / mass,
            dimensional_params["Y_p"] / mass,
            (dimensional_params["Y_r"] - mass*aircraft_data['Ue']) / mass,
            g*np.cos(aircraft_data['theta_e'])
        ],
        [
            dimensional_params["L_v"] / Ix + dimensional_params["N_v"] * Ixz / (Ix * Iz),
            dimensional_params["L_p"] / Ix + dimensional_params["N_p"] * Ixz / (Ix * Iz),
            dimensional_params["L_r"] / Ix + dimensional_params["N_r"] * Ixz / (Ix * Iz),
            0
        ],
        [
            dimensional_params["N_v"] / Iz + dimensional_params["L_v"] * Ixz / (Ix * Iz),
            dimensional_params["N_p"] / Iz + dimensional_params["L_p"] * Ixz / (Ix * Iz),
            dimensional_params["N_r"] / Iz + dimensional_params["L_r"] * Ixz / (Ix * Iz),
            0
        ],
        [0, 1, np.tan(aircraft_data['theta_e']), 0]
    ])

    # B matrix for lateral control dynamics (aileron and rudder)
    b_mat = np.array([
        [
            dimensional_params["Y_xi"] / mass,
            dimensional_params["Y_zeta"] / mass
        ],
        [
            dimensional_params["L_xi"] / Ix + dimensional_params["N_xi"] * Ixz / (Ix * Iz),
            dimensional_params["L_zeta"] / Ix + dimensional_params["N_zeta"] * Ixz / (Ix * Iz)
        ],
        [
            dimensional_params["N_xi"] / Iz + dimensional_params["L_xi"] * Ixz / (Ix * Iz),
            dimensional_params["N_zeta"] / Iz + dimensional_params["L_zeta"] * Ixz / (Ix * Iz)
        ],
        [0, 0]
    ]).T


    # Control inputs
    xi = 0.0175  # Aileron deflection in radians (approximately 1 degree)
    zeta = 0.0   # Rudder deflection in radians

    return a_mat, b_mat, xi, zeta