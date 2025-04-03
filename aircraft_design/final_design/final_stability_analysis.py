from aircraft_design.analysis.static_stability import *
from aircraft_design.final_design.final_construction import Aircraft
from aircraft_design.core.plotting import plot_orthographic_views
import matplotlib.pyplot as plt
import json
import copy
import warnings
def aircraft_to_parameters(aircraft: Aircraft):
    """
    Convert an Aircraft object to a dictionary of parameters
    """
    # First define basic geometric parameters
    wing = aircraft.wing.aero.geometry
    htail = aircraft.horizontal_tail.geometry
    vtail = aircraft.vertical_tail.geometry
    fuselage = aircraft.fuselage.geometry
    
    wing_zero = wing.position.x

    cg_absolute_position = aircraft.get_mass_properties()['cg_x']
    mac = wing.mean_aerodynamic_chord
    
    # Get tail incidence angle from orientation
    tail_incidence_deg = htail.orientation.pitch
    tail_incidence_rad = np.radians(tail_incidence_deg)
    
    aircraft_params = {
        'wing_area': wing.area,
        'wingspan': wing.parameters['span'],
        'root_chord': wing.parameters['root_chord'],
        'taper_ratio': wing.taper_ratio,
        'aspect_ratio': wing.aspect_ratio,
        'mac': mac,
        # Tail geometry
        'htail_area': htail.area,         # Horizontal tail area (ft^2)
        'htail_arm': htail.position.x - wing_zero,  # Distance from leading edge of wing to horizontal tail AC (ft)
        'vertical_tail_area': vtail.area, # ft^2
        'vertical_tail_arm': vtail.position.x - wing_zero,  # Distance from leading edge of wing to vertical tail AC (ft)
        
        # Position parameters
        'cg_position': (cg_absolute_position - wing_zero) / mac,  # CG position as fraction of MAC
        'ac_position': 0.25,        # Aerodynamic center position as fraction of MAC
        'htail_ac_position': (htail.position.x - wing_zero) / mac,  # Horizontal tail AC position as fraction of MAC
        
        # Basic aerodynamic parameters
        'section_lift_slope': 2*np.pi,  # 2π per radian (theoretical)
        'zero_lift_angle': np.radians(-5),  # Zero lift angle of attack in radians
        
        # Efficiency factors
        'tail_efficiency': 0.95,    # Horizontal tail efficiency factor #TODO: figure out a real value
        'vertical_tail_efficiency': 0.95,  # Vertical tail efficiency factor
        
        # Control surface parameters
        'aileron_inner_location': 0.5,  # Fraction of semi-span
        'aileron_outer_location': 0.9,  # Fraction of semi-span
        'max_rudder_deflection': np.radians(25),  # radians
        
        # Engine parameters
        'max_thrust': aircraft.engines[0].parameters['thrust'],        # lbs
        'engine_offset': 53,        # ft from centerline
        
        # Initial conditions
        'tail_incidence': tail_incidence_rad,  # Using value from orientation
        
        # Weight
        'aircraft_weight': 1.585e6   # lbs
    }
    if htail.aspect_ratio == 0.0:
        raise ValueError("Horizontal tail aspect ratio is 0.0")
    # Calculate derived parameters
    aircraft_params['wing_lift_slope'] = finite_wing_correction(aircraft_params['section_lift_slope'], aircraft_params['aspect_ratio'])
    aircraft_params['tail_lift_slope'] = finite_wing_correction(aircraft_params['section_lift_slope'], htail.aspect_ratio)
    
    # Calculate vertical tail aspect ratio and lift slope
    if vtail.aspect_ratio == 0.0:
        raise ValueError("Vertical tail aspect ratio is 0.0")
    aircraft_params['vertical_tail_aspect_ratio'] = vtail.aspect_ratio
    aircraft_params['vertical_tail_lift_slope'] = finite_wing_correction(aircraft_params['section_lift_slope'], vtail.aspect_ratio)
    
    aircraft_params['d_epsilon_d_alpha'] = downwash_gradient(aircraft_params['wing_lift_slope'],aircraft_params['aspect_ratio'])
    aircraft_params['aileron_effectiveness'] = 1#calculate_control_surface_effectiveness(0.25, "sealed") #TODO: double check this
    aircraft_params['tail_efficiency'] = 1 #dynamic_presure_ratio(aircraft_params['htail_arm'], 5, 0)
    aircraft_params['cm_ac'] = -0.1
    aircraft_params['d_epsilon_d_beta'] = 0.0
    return aircraft_params

def update_aircraft_parameters(aircraft: Aircraft, design_vars):
    """
    Update the aircraft parameters with the new design variables
    
    Parameters:
    aircraft: Aircraft object to update
    design_vars: [wing_position, tail_incidence, h_tail_taper, v_tail_taper]
    
    Returns:
    Updated aircraft object
    """
    # Unpack design variables
    wing_position = design_vars[0]
    tail_incidence_rad = design_vars[1]
    h_tail_taper = design_vars[2]  # Only modify taper ratio for horizontal tail
    v_tail_taper = design_vars[3]  # Only modify taper ratio for vertical tail

    # update the aircraft parameters
    aircraft.wing.aero.geometry.position.x = wing_position
    
    # Set horizontal tail incidence angle in degrees
    aircraft.horizontal_tail.geometry.orientation.pitch = tail_incidence_rad
    
    # Keep span and root chord fixed, adjust tip chord via taper ratio
    h_root_chord = aircraft.horizontal_tail.geometry.parameters['root_chord']
    aircraft.horizontal_tail.geometry.parameters['tip_chord'] = h_tail_taper * h_root_chord
    
    # Keep height and root chord fixed, adjust tip chord via taper ratio
    v_root_chord = aircraft.vertical_tail.geometry.parameters['root_chord']
    aircraft.vertical_tail.geometry.parameters['tip_chord'] = v_tail_taper * v_root_chord
    
    return aircraft


def stability_objective_function(design_vars, aircraft: Aircraft, flight_conditions):
    """
    Simplified objective function for stability optimization
    
    Parameters:
    design_vars: array [wing_position, tail_incidence, h_tail_taper, v_tail_taper]
    aircraft: Aircraft object
    flight_conditions: Flight conditions to analyze
    
    Returns:
    cost: Cost value (lower is better)
    """
    try:
        # Update aircraft with new parameters
        aircraft_copy = copy.deepcopy(aircraft)  # Create a copy to avoid modifying the original
        aircraft_copy = update_aircraft_parameters(aircraft_copy, design_vars)
        current_params = aircraft_to_parameters(aircraft_copy)

        # Create a copy of flight conditions to modify
        current_flight_conditions = flight_conditions.copy()

        # Run stability analysis
        stability_results = analyze_aircraft_stability("optimization", current_params, current_flight_conditions)
        
        # Update flight conditions with trim angle of attack
        current_flight_conditions['alpha'] = stability_results['trim_control']['alpha_trim_1']
        
        # Run stability analysis again with updated flight conditions
        stability_results = analyze_aircraft_stability("optimization", current_params, current_flight_conditions)
        
        # Extract key stability metrics
        static_margin = stability_results['longitudinal_stability']['static_margin']
        trim_angle = stability_results['trim_control']['alpha_trim_1']
        CN_beta = stability_results['directional_stability']['CN_beta']
        
        # Define cost components using squared errors
        target_static_margin = 0.2  # Target 15% static margin
        static_margin_cost = 5.0 * (static_margin - target_static_margin)**2
        
        # Higher penalty for unstable configurations
        if static_margin < 0.05:
            static_margin_cost += 50.0 * (0.05 - static_margin)**2
            
        # Minimize trim angle
        trim_angle_cost = (trim_angle-np.radians(2))**2
        
        # Strongly penalize negative directional stability
        directional_stability_cost = max(0, -CN_beta)**2
        
        # Combine costs with weights
        cost = (static_margin_cost + 
                trim_angle_cost + 
                directional_stability_cost)
        
        # Print current values for debugging
        #print(f"Wing pos: {design_vars[0]:.4f}, Tail inc: {np.degrees(design_vars[1]):.2f}°, "
        #      f"H-taper: {design_vars[2]:.2f}, V-taper: {design_vars[3]:.2f}, "
        #      f"SM: {static_margin:.3f}, Cost: {cost:.4f}")
        
        return cost
    except Exception as e:
        print(f"Error in objective function: {e}")
        return 1e6  # Return high cost for invalid configurations


def optimize_stability(aircraft: Aircraft, flight_conditions):
    """
    Simplified optimization of aircraft stability
    
    Parameters:
    aircraft: Aircraft object
    flight_conditions: Flight conditions to analyze
    
    Returns:
    result: Optimization result object
    """
    from scipy.optimize import minimize
    
    # Initial guess for design variables (simplified set)
    x0 = np.array([
        0.25,  # Wing position (25% MAC)
        np.radians(2),   # Tail incidence (2 degrees)
        # Current taper ratios
        aircraft.horizontal_tail.geometry.parameters['tip_chord'] / 
        aircraft.horizontal_tail.geometry.parameters['root_chord'],  # H-tail taper
        aircraft.vertical_tail.geometry.parameters['tip_chord'] / 
        aircraft.vertical_tail.geometry.parameters['root_chord']     # V-tail taper
    ])
    
    # Add bounds to prevent zero or negative values for critical parameters
    bounds = [
        (25,175),                      # Wing position bounds (15-35% MAC)
        (0, np.radians(10)),  # Tail incidence bounds (-5 to 10 degrees)
        (0.1, 0.9),                        # Horizontal tail taper ratio bounds (0.3-0.9)
        (0.1, 0.9)                         # Vertical tail taper ratio bounds (0.3-0.9)
    ]
    
    # Define objective function
    objective = lambda x: stability_objective_function(x, aircraft, flight_conditions)
    
    print("Starting optimization...")
    print("Initial parameters:")
    print(f"Wing Position: {x0[0]:.3f} MAC")
    print(f"Tail Incidence: {np.degrees(x0[1]):.2f} degrees")
    print(f"Horizontal Tail Taper Ratio: {x0[2]:.3f}")
    print(f"Vertical Tail Taper Ratio: {x0[3]:.3f}")

    result = minimize(
        objective,
        x0,
        method='Nelder-Mead',
        bounds=bounds,
        options={
            'disp': True,
            'maxiter': 200,
            'xatol': 1e-4,
            'fatol': 1e-4
        }
    )
    
    # Display results
    print("\nOptimization complete!")
    print(f"Success: {result.success}")
    print(f"Message: {result.message}")
    print("\nOptimized parameters:")
    print(f"Wing Position: {result.x[0]:.3f} MAC")
    print(f"Tail Incidence: {np.degrees(result.x[1]):.2f} degrees")
    print(f"Horizontal Tail Taper Ratio: {result.x[2]:.3f}")
    print(f"Vertical Tail Taper Ratio: {result.x[3]:.3f}")
    
    # Calculate and print final static margin
    final_aircraft = update_aircraft_parameters(aircraft, result.x)
    final_params = aircraft_to_parameters(final_aircraft)
    stability_results = analyze_aircraft_stability("final", final_params, flight_conditions)
    static_margin = stability_results['longitudinal_stability']['static_margin']
    print(f"Final Static Margin: {static_margin:.3f}")
    
    return result

def analyze_final_parameters(result, aircraft: Aircraft, flight_conditions):
    """
    Analyze and save the final optimized parameters and their effects
    """
    if not result.success:
        print("Optimization was not successful. Cannot analyze final parameters.")
        return {}
    
    # Update aircraft with optimized parameters
    final_aircraft = update_aircraft_parameters(aircraft, result.x)
    final_params = aircraft_to_parameters(final_aircraft)
    
    # Calculate all stability metrics and analyses
    stability_results = analyze_aircraft_stability("optimized", final_params, flight_conditions)
    
    # Perform individual analyses
    longitudinal_results = longitudinal_stability_analysis(final_params, flight_conditions)
    directional_results = directional_stability_analysis(final_params, flight_conditions)
    lateral_results = lateral_control_analysis(final_params, flight_conditions)
    trim_results = trim_and_control_analysis(final_params, flight_conditions)
    engine_out_results = engine_out_analysis(final_params, flight_conditions)
    
    # Calculate tail volume coefficients
    h_tail_volume = final_params['htail_area'] * final_params['htail_arm'] / (
        final_params['wing_area'] * final_params['mac'])
    v_tail_volume = final_params['vertical_tail_area'] * final_params['vertical_tail_arm'] / (
        final_params['wing_area'] * final_params['wingspan'])
    
    # Create comprehensive results dictionary
    final_analysis = {
        'optimization_result': {
            'success': result.success,
            'message': result.message,
            'iterations': result.nit,
            'final_cost': result.fun,
            'optimal_design_variables': {
                'wing_position': result.x[0],
                'tail_incidence': result.x[1],
                'horizontal_tail_taper_ratio': result.x[2],
                'vertical_tail_taper_ratio': result.x[3],
                'horizontal_tail_volume': h_tail_volume,
                'vertical_tail_volume': v_tail_volume
            }
        },
        'final_parameters': final_params,
        'flight_conditions': flight_conditions,
        'raw_analysis_results': {
            'longitudinal_stability_analysis': longitudinal_results,
            'directional_stability_analysis': directional_results,
            'lateral_control_analysis': lateral_results,
            'trim_and_control_analysis': trim_results,
            'engine_out_analysis': engine_out_results
        },
        'comprehensive_stability_results': stability_results
    }
    
    # Save to JSON file
    try:
        with open("final_optimization_analysis.json", "w") as f:
            # Convert any non-JSON serializable values to strings
            def convert_to_serializable(obj):
                if isinstance(obj, (np.float32, np.float64)):
                    return float(obj)
                elif isinstance(obj, (np.int32, np.int64)):
                    return int(obj)
                elif isinstance(obj, np.bool_):
                    return bool(obj)
                elif isinstance(obj, dict):
                    return {k: convert_to_serializable(v) for k, v in obj.items()}
                elif isinstance(obj, (list, tuple)):
                    return [convert_to_serializable(x) for x in obj]
                elif isinstance(obj, (bool, str, int, float)) or obj is None:
                    return obj
                return str(obj)
            
            serializable_results = convert_to_serializable(final_analysis)
            json.dump(serializable_results, f, indent=4)
        print("\nFinal analysis saved to 'final_optimization_analysis.json'")
    except Exception as e:
        print(f"Error saving final analysis: {e}")
        print("\nFinal analysis results:")
        print(json.dumps(final_analysis, indent=4))
    
    return final_analysis

if __name__ == "__main__":
    aircraft = Aircraft()
    # Define flight conditions for optimization
    flight_conditions = {
        'airspeed': 0.9 * 1116.45,    # 90% of design speed
        'density': 0.00237717,       # Air density at altitude
        'alpha': 0,     # Initial angle of attack
        'sideslip': 0               # No sideslip
    }
    aircraft_params = aircraft_to_parameters(aircraft)
    # supress runtime warnings until the optimization is complete
    warnings.filterwarnings("ignore")
    print("Starting stability optimization...")
    print("\nInitial aircraft parameters:")
    print(f"Wing Position: {aircraft_params['ac_position']:.3f} MAC")
    print(f"Tail Incidence: {np.degrees(aircraft_params['tail_incidence']):.2f} degrees")
    print(f"Horizontal Tail Volume Ratio: {aircraft_params['htail_area'] * aircraft_params['htail_arm'] / (aircraft_params['wing_area'] * aircraft_params['mac']):.3f}")
    print(f"Vertical Tail Volume Ratio: {aircraft_params['vertical_tail_area'] * aircraft_params['vertical_tail_arm'] / (aircraft_params['wing_area'] * aircraft_params['wingspan']):.3f}")

    # Run optimization and analyze results
    result = optimize_stability(aircraft, flight_conditions)
    final_analysis = analyze_final_parameters(result, aircraft, flight_conditions)

    # Plot the results using the optimized parameters
    if result.success:

        print("Optimization successful")
        # Create a copy of the final parameters for plotting
        plot_params = final_analysis['final_parameters']
        
        # Generate stability plots
        fig = plot_stability_derivatives(plot_params, flight_conditions, np.radians(np.arange(-10, 10, 0.1)))
        plt.savefig("assets/stability_optimization_results.png")
        plt.close()
        
        print("\nStability plots have been saved to 'stability_optimization_results.png'")

        # save the aircraft to a json file
        with open("assets/aircraft.json", "w") as f:
            json.dump(aircraft.to_dict(), f, indent=4)
        plot_aircraft = False
        if plot_aircraft:
            # plot the aircraft orthographic view
            obj = aircraft.plot()
            fig = plot_orthographic_views(obj)
            plt.savefig("assets/aircraft_orthographic.png")
            plt.close()
    else:
        print("\nOptimization was not successful. No plots were generated.")

    