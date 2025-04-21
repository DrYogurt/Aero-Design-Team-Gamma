from aircraft_design.final_design.final_construction import Aircraft
from aircraft_design.analysis.dynamic_stability import calculate_derivatives
from aircraft_design.final_design.final_trade_studies.static_stability_utils import aircraft_to_parameters
from aircraft_design.final_design.final_trade_studies.static_stability_trade_study import analyze_aircraft_static_stability
from aircraft_design.analysis.stability_conversion import convert_stability_to_aero_derivatives, lateral_directional_matrices, longitudinal_matrices
from aircraft_design.analysis.longitudinal import aircraft_longitudinal_dynamics
from aircraft_design.analysis.lateral import aircraft_lateral_dynamics

import copy
import numpy as np
import matplotlib.pyplot as plt

def create_stability_parameters(aircraft: Aircraft):
    """
    Create the stability parameters dictionary and calculate dimensionless derivatives.
    
    Args:
        aircraft: Aircraft object
        
    Returns:
        tuple: (stability_params, dimensionless_derivatives)
    """
    # run static stability analysis
    aircraft_params, analysis_results = analyze_aircraft_static_stability(aircraft)
    mass_results = aircraft.analysis_results['mass_analysis']
    # Create the parameter dictionary for dynamic stability analysis
    stability_params = {
        # Flight conditions
        'V0': aircraft_params['airspeed'],
        'rho': aircraft_params['density'],
        
        # Wing parameters
        'S': aircraft_params['wing_area'],
        'b': aircraft_params['wingspan'],
        'c_bar': aircraft_params['mac'],
        'c_0': aircraft_params['root_chord'],
        'taper_ratio': aircraft_params['taper_ratio'],
        
        # Aerodynamic coefficients (placeholder values - these should be calculated or estimated)
        'CL': analysis_results['stability_analyses']['longitudinal_stability']['CL_total'],
        'CD': analysis_results['stability_analyses']['longitudinal_stability']['CD_total'],
        'dCL_da': analysis_results['stability_analyses']['longitudinal_stability']['CL_alpha'],
        'dCD_da': analysis_results['stability_analyses']['longitudinal_stability']['CD_alpha'],
        'dCM_da': analysis_results['stability_analyses']['longitudinal_stability']['CM_alpha'],
        
        # Note, velocity derivatives are assumed to be zero: dCL_dV, dCD_dV, dCM_dV

        # Downwash and CG parameters
        'de_da': aircraft_params['d_epsilon_d_alpha'],  # Downwash factor
        'static_margin': analysis_results['stability_analyses']['longitudinal_stability']['static_margin'],  # Neutral point location as fraction of c_bar
        
        # Horizontal tail parameters
        'St': aircraft_params['htail_area'],
        'lt': aircraft_params['htail_arm'],
        'at': aircraft_params['tail_lift_slope'],  # Horizontal tail lift curve slope [1/rad]
        
        # Vertical tail parameters
        'Sv': aircraft_params['vertical_tail_area'],
        'lv': aircraft_params['vertical_tail_arm'],
        'zv': aircraft_params['vertical_tail_zv'],  # Vertical distance from CG to vertical tail
        'av': aircraft_params['tail_lift_slope'],  # Vertical tail lift curve slope [1/rad]
        'bv': aircraft_params['vertical_tail_height'],  # Vertical tail span/height
        
        # Wing geometry parameters
        'Gamma': aircraft_params['Gamma'],  # Wing dihedral angle [rad]
        'Lambda': aircraft_params['Lambda'],  # Wing sweep angle [rad]
        
        # Control surface parameters
        'Tau_e': 0.5,  # Elevator effectiveness factor
        'eta_e': 0.7,  # Elevator correction factor
        'Tau_r': 0.5,  # Rudder effectiveness factor
        'eta_r': 0.7,  # Rudder correction factor
        'Tau_a': 0.5,  # Aileron effectiveness factor
        'eta_a': 0.7,  # Aileron correction factor
        
        # Additional parameters for control derivatives
        'aileron_start': aircraft_params['aileron_inner_location'],
        'y2': aircraft_params['aileron_outer_location'],
        'dCD_dxi': 0.1,  # Change in drag coefficient with aileron deflection
        'CL_t': 0.5,  # Tail lift coefficient
        'e_t': 0.95,  # Tail efficiency factor
        'A_t': aircraft_params['htail_aspect_ratio'],  # Tail aspect ratio
        'c_v': aircraft_params['vertical_tail_chord'],  # Vertical tail chord at root

        'mass': mass_results['total_mass'],
        'Ix': mass_results['total_ixx'],
        'Iy': mass_results['total_iyy'],
        'Iz': mass_results['total_izz'],
        'Ixy': mass_results['total_ixy'],
        'Ixz': mass_results['total_ixz'],
        'Iyz': mass_results['total_iyz'],
        'theta_e': np.radians(aircraft_params['alpha']),
        'Ue': aircraft_params['airspeed'] * np.cos(aircraft_params['alpha']),
        'We': aircraft_params['airspeed'] * np.sin(aircraft_params['alpha']),
    }
    
    dimensionless_derivatives = calculate_derivatives(stability_params, aircraft)

    return aircraft_params, stability_params, dimensionless_derivatives

def analyze_aircraft_dynamic_stability(
    aircraft: Aircraft,
    output_filename=None,
    ku=[0.0, 0.0],
    kw=[0.0, 0.0],
    kq=[0.0, 0.0],
    ko_long=[0.0, 0.0],
    kv=[0.0, 0.0],
    kp=[0.0, 0.0],
    kr=[0.0, 0.0],
    ko_lat=[0.0, 0.0],
    visualize=False
):
    """
    Convert aircraft parameters from final construction to the format needed for dynamic stability analysis.
    
    Args:
        aircraft: Aircraft object
        output_filename: Optional filename to save results
        ku: Longitudinal velocity feedback gains [P, I]
        kw: Vertical velocity feedback gains [P, I]
        kq: Pitch rate feedback gains [P, I]
        ko_long: Longitudinal offset feedback gains [P, I]
        kv: Lateral velocity feedback gains [P, I]
        kp: Roll rate feedback gains [P, I]
        kr: Yaw rate feedback gains [P, I]
        ko_lat: Lateral offset feedback gains [P, I]
    
    Returns:
        tuple: (short_nf, short_df, p_nf, p_df, dnf, ddr, Tr, Ts)
    """
    
    # Get stability parameters and dimensionless derivatives
    _, stability_params, dimensionless_derivatives = create_stability_parameters(aircraft)
    
    aero_derivatives = convert_stability_to_aero_derivatives(
        dimensionless_derivatives,
        S=stability_params['S'],
        c_bar=stability_params['c_bar'],
        b=stability_params['b'],
        rho=stability_params['rho'],
        V0=stability_params['V0']
    )
    
    a_mat_longitudinal, b_mat_longitudinal = longitudinal_matrices(
        aero_derivatives,
        stability_params,
    )
     # Time parameters for simulation
    eta = np.radians(20)  # Elevator deflection in radians
    tau = 10  # Thrust variation
    
    a_mat_lateral, b_mat_lateral = lateral_directional_matrices(aero_derivatives, stability_params)
    
    tpar = [0, 0.01, 100]

    short_nf, short_df, p_nf, p_df = aircraft_longitudinal_dynamics(
        stability_params['V0'],
        a_mat_longitudinal,
        b_mat_longitudinal,
        tpar, eta, tau, ku, kw, kq, ko_long,
        plots=visualize,
        prints=visualize,
        for_optim=True
    )

    # Control inputs
    xi = np.radians(10)  # Aileron deflection in radians (approximately 1 degree)
    zeta = np.radians(10)   # Rudder deflection in radians (approximately 1 degree)

    dnf, ddr, Tr, Ts = aircraft_lateral_dynamics(
        stability_params['V0'],
        a_mat_lateral,
        b_mat_lateral,
        tpar, xi, zeta, kv, kp, kr, ko_lat,
        plots=visualize,
        prints=visualize,
        for_optim=True
    )
    return short_nf, short_df, p_nf, p_df, dnf, ddr, Tr, Ts

def optimize_longitudinal_gains(aircraft: Aircraft, target_values=None, base_gains=None):
    """
    Optimize the longitudinal feedback gains to minimize the MSE between actual and target dynamic stability characteristics.
    
    Args:
        aircraft: Aircraft object
        target_values: Dictionary of target values for each stability characteristic
                      If None, uses default values for a stable aircraft
    
    Returns:
        dict: Optimized longitudinal gain values
    """
    from scipy.optimize import minimize
    
    if target_values is None:
        target_values = {
            'short_nf': (2.5,3.5), # 2.5-3.5 # Target natural frequency for short period mode
            'short_df': (0.3,2.0),   # 0.3-2.0 # Target damping ratio for short period mode
            'p_nf': 1e-9, # positive     # Target natural frequency for phugoid mode
            'p_df': 0.04, # min 0.04     # Target damping ratio for phugoid mode
        }
    
    def cost_function(gains):
        # Unpack the gains
        ku = gains[0:2]
        kw = gains[2:4]
        kq = gains[4:6]
        ko_long = gains[6:8]
        
        # Get the actual values
        short_nf, short_df, p_nf, p_df, _, _, _, _ = analyze_aircraft_dynamic_stability(
            aircraft,
            ku=ku, kw=kw, kq=kq, ko_long=ko_long,
            kv=[0,0], kp=[0,0], kr=[0,0], ko_lat=[0,0],
            visualize=False
        )
        
        # Calculate MSE
        cost = (
            # Short period mode - normalize by range width
            10*(min(0, short_nf - target_values['short_nf'][1]) + max(0, target_values['short_nf'][0] - short_nf))**2 +
            (min(0, short_df - target_values['short_df'][1]) + max(0, target_values['short_df'][0] - short_df))**2 +
            # Phugoid mode - natural frequency should be positive, normalize by target value
            (min(0, p_nf - target_values['p_nf']))**2 / (target_values['p_nf']**2 + 1e-10) +
            # Phugoid damping ratio - normalize by target value
            (min(0, p_df - target_values['p_df']))**2 / (target_values['p_df']**2 + 1e-10)
        )

        if any(np.isnan([short_nf, short_df, p_nf, p_df])):
            cost = 1e10
        print(f"Longitudinal Cost: {cost}, short_nf: {short_nf:.2f}, short_df: {short_df:.2f}, p_nf: {p_nf:.2f}, p_df: {p_df:.2f}")
        print(f"Longitudinal Gains: {gains}")
        return cost
    
    # Initial guess (random values between -1 and 1)
    x0 = np.random.uniform(-1, 1, 8)
    if base_gains is not None:
        x0 = base_gains
    
    # Bounds for all gains (-1 to 1)
    bounds = [(-0.001, 0.001) for _ in range(8)]
    
    # Optimize
    result = minimize(
        cost_function,
        x0,
        bounds=bounds,
        #method='Nelder-Mead',
    )
    
    # Return optimized gains in a dictionary
    optimized_gains = {
        'ku': result.x[0:2],
        'kw': result.x[2:4],
        'kq': result.x[4:6],
        'ko_long': result.x[6:8]
    }
    
    return optimized_gains

def optimize_lateral_gains(aircraft: Aircraft, target_values=None, base_gains=None):
    """
    Optimize the lateral feedback gains to minimize the MSE between actual and target dynamic stability characteristics.
    
    Args:
        aircraft: Aircraft object
        target_values: Dictionary of target values for each stability characteristic
                      If None, uses default values for a stable aircraft
    
    Returns:
        dict: Optimized lateral gain values
    """
    from scipy.optimize import minimize
    
    if target_values is None:
        target_values = {
            'dnf': 0.5, #min 0.5     # Target natural frequency for Dutch roll mode
            'ddr': 0.08, #min 0.08     # Target damping ratio for Dutch roll mode
            'Tr': 2.5, # max 1.4   -  Target time constant for roll subsistence mode
            'Ts': 28.9 # max 28.9  - Target time constant for spiral mode
        }
    
    def cost_function(gains):
        # Unpack the gains
        kv = gains[0:2]
        kp = gains[2:4]
        kr = gains[4:6]
        ko_lat = gains[6:8]
        
        # Get the actual values
        _, _, _, _, dnf, ddr, Tr, Ts = analyze_aircraft_dynamic_stability(
            aircraft,
            ku=[0,0], kw=[0,0], kq=[0,0], ko_long=[0,0],
            kv=kv, kp=kp, kr=kr, ko_lat=ko_lat,
            visualize=False
        )
        
        # Calculate MSE
        cost = (
            # Dutch roll mode - normalize by target value
            (min(0, dnf - target_values['dnf']))**2 / (target_values['dnf']**2 + 1e-10) +
            # Dutch roll damping ratio - normalize by target value
            (min(0, ddr - target_values['ddr']))**2 / (target_values['ddr']**2 + 1e-10) +
            # Roll mode time constant - normalize by target value
            (max(0, Tr - target_values['Tr']))**2 / (target_values['Tr']**2 + 1e-10) +
            # Spiral mode time constant - normalize by target value
            (min(0, Ts - abs(target_values['Ts'])))**2 / (abs(target_values['Ts'])**2 + 1e-10) +
            # Ensure time constants are positive - use fixed normalization factor
            (min(0, Tr))**2
        )
        if any(np.isnan([dnf, ddr, Tr, Ts])):
            cost = 1e10
        print(f"Lateral Cost: {cost}, dnf: {dnf:.2f}, ddr: {ddr:.2f}, Tr: {Tr:.2f}, Ts: {Ts:.2f}")
        print(f"Lateral Gains: {gains}")
        return cost
    
    # Initial guess (all zeros)
    x0 = np.zeros(8)
    if base_gains is not None:
        x0 = base_gains
    
    # Bounds for all gains (-1 to 1)
    #bounds = [(-3, 3) for _ in range(8)]
    
    # Optimize
    result = minimize(
        cost_function,
        x0,
        #bounds=bounds,
        #method='Nelder-Mead',
    )
    
    # Return optimized gains in a dictionary
    optimized_gains = {
        'kv': result.x[0:2],
        'kp': result.x[2:4],
        'kr': result.x[4:6],
        'ko_lat': result.x[6:8]
    }
    
    return optimized_gains

def plot_gain_sensitivity(aircraft: Aircraft, gain_name: str, gain_range=(-2, 2), num_points=10, base_gains=None):
    """
    Plot how each gain affects the stability characteristics using contour plots.
    
    Args:
        aircraft: Aircraft object
        gain_name: Name of the gain to analyze (e.g., 'ku', 'kw', etc.)
        gain_range: Tuple of (min, max) values for the gain
        num_points: Number of points to evaluate in the range
    """
    # Create base gains dictionary with all gains set to zero
    if base_gains is None:
        gains = {
            'ku': [0, 0], 'kw': [0, 0], 'kq': [0, 0], 'ko_long': [0, 0],
            'kv': [0, 0], 'kp': [0, 0], 'kr': [0, 0], 'ko_lat': [0, 0]
        }
    else:
        gains = copy.deepcopy(base_gains)
    # Generate gain values to test
    gain_values = np.linspace(gain_range[0], gain_range[1], num_points)
    
    # Create figure with 4 subplots
    fig, axs = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(f'Effect of {gain_name} on Stability Characteristics', fontsize=16)
    
    # Create meshgrid for contour plots
    X, Y = np.meshgrid(gain_values, gain_values)
    
    # Initialize arrays to store results
    if gain_name in ['ku', 'kw', 'kq', 'ko_long']:
        short_nf = np.zeros((num_points, num_points))
        short_df = np.zeros((num_points, num_points))
        p_nf = np.zeros((num_points, num_points))
        p_df = np.zeros((num_points, num_points))
    else:
        dnf = np.zeros((num_points, num_points))
        ddr = np.zeros((num_points, num_points))
        Tr = np.zeros((num_points, num_points))
        Ts = np.zeros((num_points, num_points))
    
    # Calculate results for each combination of gain values
    for i, g1 in enumerate(gain_values):
        for j, g2 in enumerate(gain_values):
            gains = base_gains.copy()
            gains[gain_name][0] = g1
            gains[gain_name][1] = g2
            results = analyze_aircraft_dynamic_stability(aircraft, **gains)
            
            if gain_name in ['ku', 'kw', 'kq', 'ko_long']:
                short_nf[i,j] = results[0]
                short_df[i,j] = results[1]
                p_nf[i,j] = results[2]
                p_df[i,j] = results[3]
            else:
                dnf[i,j] = results[4]
                ddr[i,j] = results[5]
                Tr[i,j] = results[6]
                Ts[i,j] = results[7]
    
    # Plot longitudinal characteristics
    if gain_name in ['ku', 'kw', 'kq', 'ko_long']:
        # Short period natural frequency
        cs = axs[0,0].contourf(X, Y, short_nf, levels=20, cmap='viridis')
        axs[0,0].set_title('Short Period Natural Frequency')
        axs[0,0].set_xlabel(f'{gain_name}[0]')
        axs[0,0].set_ylabel(f'{gain_name}[1]')
        plt.colorbar(cs, ax=axs[0,0], label='Natural Frequency (rad/s)')
        
        # Short period damping ratio
        cs = axs[0,1].contourf(X, Y, short_df, levels=20, cmap='viridis')
        axs[0,1].set_title('Short Period Damping Ratio')
        axs[0,1].set_xlabel(f'{gain_name}[0]')
        axs[0,1].set_ylabel(f'{gain_name}[1]')
        plt.colorbar(cs, ax=axs[0,1], label='Damping Ratio')
        
        # Phugoid natural frequency
        cs = axs[1,0].contourf(X, Y, p_nf, levels=20, cmap='viridis')
        axs[1,0].set_title('Phugoid Natural Frequency')
        axs[1,0].set_xlabel(f'{gain_name}[0]')
        axs[1,0].set_ylabel(f'{gain_name}[1]')
        plt.colorbar(cs, ax=axs[1,0], label='Natural Frequency (rad/s)')
        
        # Phugoid damping ratio
        cs = axs[1,1].contourf(X, Y, p_df, levels=20, cmap='viridis')
        axs[1,1].set_title('Phugoid Damping Ratio')
        axs[1,1].set_xlabel(f'{gain_name}[0]')
        axs[1,1].set_ylabel(f'{gain_name}[1]')
        plt.colorbar(cs, ax=axs[1,1], label='Damping Ratio')
    
    # Plot lateral characteristics
    else:
        # Dutch roll natural frequency
        cs = axs[0,0].contourf(X, Y, dnf, levels=20, cmap='viridis')
        axs[0,0].set_title('Dutch Roll Natural Frequency')
        axs[0,0].set_xlabel(f'{gain_name}[0]')
        axs[0,0].set_ylabel(f'{gain_name}[1]')
        plt.colorbar(cs, ax=axs[0,0], label='Natural Frequency (rad/s)')
        
        # Dutch roll damping ratio
        cs = axs[0,1].contourf(X, Y, ddr, levels=20, cmap='viridis')
        axs[0,1].set_title('Dutch Roll Damping Ratio')
        axs[0,1].set_xlabel(f'{gain_name}[0]')
        axs[0,1].set_ylabel(f'{gain_name}[1]')
        plt.colorbar(cs, ax=axs[0,1], label='Damping Ratio')
        
        # Roll time constant
        cs = axs[1,0].contourf(X, Y, Tr, levels=20, cmap='viridis')
        axs[1,0].set_title('Roll Time Constant')
        axs[1,0].set_xlabel(f'{gain_name}[0]')
        axs[1,0].set_ylabel(f'{gain_name}[1]')
        plt.colorbar(cs, ax=axs[1,0], label='Time Constant (s)')
        
        # Spiral time constant
        cs = axs[1,1].contourf(X, Y, Ts, levels=20, cmap='viridis')
        axs[1,1].set_title('Spiral Time Constant')
        axs[1,1].set_xlabel(f'{gain_name}[0]')
        axs[1,1].set_ylabel(f'{gain_name}[1]')
        plt.colorbar(cs, ax=axs[1,1], label='Time Constant (s)')
    
    plt.tight_layout()
    plt.savefig(f'assets/dynamic_stability/{gain_name}_sensitivity.png')
    plt.close()

if __name__ == "__main__":
    # Create base aircraft
    aircraft = Aircraft()
    
    # Get passive values
    short_nf, short_df, p_nf, p_df, dnf, ddr, Tr, Ts = analyze_aircraft_dynamic_stability(aircraft, visualize=True)
    print(f"========= Passive Values ==========")
    print(f"short_nf: {short_nf:.2f}, short_df: {short_df:.2f}, p_nf: {p_nf:.2f}, p_df: {p_df:.2f}")
    print(f"dnf: {dnf:.2f}, ddr: {ddr:.2f}, Tr: {Tr:.2f}, Ts: {Ts:.2f}")
    

    test_gains = {
        'ku': [0.5, 1], 'kw': [0, -0.5], 'kq': [0, -0.5], 'ko_long': [0.5, -0.5],
        'kv': [0, 0], 'kp': [-0.25, 0.25], 'kr': [0.5, 0], 'ko_lat': [0.75, -0.25]
    }

    short_nf, short_df, p_nf, p_df, dnf, ddr, Tr, Ts = analyze_aircraft_dynamic_stability(aircraft, **test_gains, visualize=True)
    print(f"========= Augmented Values ==========")
    print(f"short_nf: {short_nf:.2f}, short_df: {short_df:.2f}, p_nf: {p_nf:.2f}, p_df: {p_df:.2f}")
    print(f"dnf: {dnf:.2f}, ddr: {ddr:.2f}, Tr: {Tr:.2f}, Ts: {Ts:.2f}")

    # Plot sensitivity for each gain
    print("\nGenerating gain sensitivity plots...")
    for gain in ['ku', 'kw', 'kq', 'ko_long', 'kv', 'kp', 'kr', 'ko_lat']:
        print(f"Plotting {gain} sensitivity...")
        plot_gain_sensitivity(aircraft, gain, base_gains=test_gains)
    exit()
    
    # Optimize the longitudinal stability characteristics
    print("\nOptimizing longitudinal stability characteristics...")
    longitudinal_gains = optimize_longitudinal_gains(aircraft)
    
    # Optimize the lateral stability characteristics
    print("\nOptimizing lateral stability characteristics...")
    lateral_gains = optimize_lateral_gains(aircraft)
    
    # Combine the gains
    optimized_gains = {**longitudinal_gains, **lateral_gains}
    
    # Print the optimized gains
    print("\nOptimized Gains:")
    for gain_name, gain_values in optimized_gains.items():
        print(f"{gain_name}: {gain_values}")
    
    # Run analysis with optimized gains
    print("\nResults with optimized gains:")
    optimized_results = analyze_aircraft_dynamic_stability(
        aircraft, 
        "optimized_dynamic_stability.json", 
        **optimized_gains,
        visualize=True  
    )
    
    # Print the optimized results
    for key, value in optimized_results.items():
        print(f"{key},\t{value:.4f}")
    print("Optimization completed successfully.")
