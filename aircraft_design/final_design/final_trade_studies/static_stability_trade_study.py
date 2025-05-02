import numpy as np
import matplotlib.pyplot as plt
from aircraft_design.final_design.final_construction import Aircraft
from aircraft_design.components.propulsion.fuel_tanks import FuelTank
from aircraft_design.analysis.static_stability import longitudinal_stability_analysis, directional_stability_analysis, lateral_control_analysis, trim_and_control_analysis, engine_out_analysis
from aircraft_design.final_design.final_trade_studies.static_stability_utils import aircraft_to_parameters
from aircraft_design.analysis.dynamic_stability import calculate_derivatives
import copy
from scipy.optimize import minimize
import json
from pprint import pprint
def optimize_margin(desired_margin, total_fuel_percentage, aircraft):
    """
    Find the optimal wing-to-tail fuel ratio to achieve a desired static margin.
    
    Parameters:
    desired_margin: Target static margin
    total_fuel_percentage: Total fuel percentage (0-1)
    aircraft: Base aircraft object
    
    Returns:
    optimal_ratio: Optimal wing-to-tail fuel ratio
    """
    def objective(tail_to_wing_ratio):
        # Create aircraft copy
        aircraft_copy = copy.deepcopy(aircraft)
        
        # Calculate wing and tail fuel masses based on ratio and total fuel
        total_fuel_mass = total_fuel_percentage * (5.9e5)
        tail_fuel_mass = total_fuel_mass * tail_to_wing_ratio
        wing_fuel_mass = total_fuel_mass - tail_fuel_mass
        
        # Set wing tank fill levels
        for tank in aircraft_copy.wing.children:
            if isinstance(tank, FuelTank):
                fill_level = wing_fuel_mass / (4.9e5)  # Wing fuel as fraction of max
                tank.set_fill_level(fill_level)
        
        # Set tail tank fill levels
        for tank in aircraft_copy.horizontal_tail.children + aircraft_copy.vertical_tail.children:
            if isinstance(tank, FuelTank):
                fill_level = tail_fuel_mass / (1e5)  # Tail fuel as fraction of max
                tank.set_fill_level(fill_level)
        
        # Run analysis
        aircraft_copy.run_analysis('mass_analysis', analyze_children=True)
        aircraft_params = aircraft_to_parameters(aircraft_copy)
        stability_results = longitudinal_stability_analysis(aircraft_params)
        
        # Return squared error from desired margin
        return (stability_results['static_margin'] - desired_margin)**2
    
    # Optimize using scipy minimize
    result = minimize(
        objective,
        x0=0.5,  # Initial guess for wing-to-tail ratio
        bounds=[(0.01, 10)],  # Keep ratio between 0.01 and 10
        method='L-BFGS-B'
    )
    
    return result.x[0]

def fuel_distribution_stability_study(aircraft: Aircraft):
    """
    Analyze how fuel distribution between wing and tail tanks affects static margin.
    Creates both scatter and contour plots showing static margin vs fuel volume ratio and total fuel percentage.
    """
    # Define maximum fuel capacities
    WING_MAX_FUEL = 4.83e5  # lbs
    TAIL_MAX_FUEL = 1.12e5  # lbs
    
    # Create arrays for wing and tail fuel percentages
    wing_fuel_percentages = np.linspace(0.01, 1, 10)  # Wing fuel as percentage of WING_MAX_FUEL
    tail_fuel_percentages = np.linspace(0.01, 1, 5)  # Tail fuel as percentage of TAIL_MAX_FUEL

    # Initialize arrays for results
    static_margins = np.zeros((len(wing_fuel_percentages), len(tail_fuel_percentages)))
    total_fuel_percentages = np.zeros_like(static_margins)
    wing_to_tail_ratios = np.zeros_like(static_margins)
    
    #set incidence angle to 1 degree
    #aircraft.horizontal_tail.geometry.orientation.pitch = np.radians(.2)
    # Iterate through all combinations
    for i, wing_percent in enumerate(wing_fuel_percentages):
        for j, tail_percent in enumerate(tail_fuel_percentages):        
            # copy the aircraft
            aircraft_copy = copy.deepcopy(aircraft)
            
            # Set wing tank fill levels
            for tank in aircraft_copy.wing.children:
                if isinstance(tank, FuelTank):
                    fill_level = wing_percent
                    assert fill_level <= 1.0, f"Wing fill level is greater than 100%: {fill_level}"
                    tank.set_fill_level(fill_level)
            
            # Set tail tank fill levels
            for tank in aircraft_copy.horizontal_tail.children + aircraft_copy.vertical_tail.children:
                if isinstance(tank, FuelTank):
                    fill_level = tail_percent
                    assert fill_level <= 1.0, f"Tail fill level is greater than 100%: {fill_level}"
                    tank.set_fill_level(fill_level)
            
            # Run mass analysis
            aircraft_copy.run_analysis('mass_analysis', analyze_children=True)
            wing_fuel_mass = sum(tank.analysis_results['mass_analysis']['total_mass'] for tank in aircraft_copy.wing.children if isinstance(tank, FuelTank))
            tail_fuel_mass = sum(tank.analysis_results['mass_analysis']['total_mass'] for tank in aircraft_copy.horizontal_tail.children + aircraft_copy.vertical_tail.children 
                          if isinstance(tank, FuelTank))
            total_fuel_mass = wing_fuel_mass + tail_fuel_mass
            total_fuel_percentage = total_fuel_mass / (WING_MAX_FUEL + TAIL_MAX_FUEL)
            wing_to_tail_ratio = np.clip(tail_fuel_mass / wing_fuel_mass, 0.01, 10)
            total_fuel_percentages[i, j] = total_fuel_percentage
            wing_to_tail_ratios[i, j] = wing_to_tail_ratio

            # Get aircraft parameters and run stability analysis
            aircraft_params = aircraft_to_parameters(aircraft_copy)
            stability_results = longitudinal_stability_analysis(aircraft_params)
            static_margins[i, j] = stability_results['static_margin']
    
    # Create figure with two subplots
    fig, ax= plt.subplots(1, 1, figsize=(20, 8))

    # Contour plot
    contour = ax.contourf(wing_to_tail_ratios, 
                          total_fuel_percentages * 100,
                          static_margins,
                          levels=20,
                          cmap='viridis',
                          extend='both')
    
    # Add contour lines
    ax.contour(wing_to_tail_ratios,
                total_fuel_percentages * 100,
                static_margins,
                levels=10,
                colors='black',
                linewidths=0.5)
    
    # Add colorbar for contour plot
    cbar = plt.colorbar(contour, ax=ax)
    cbar.set_label('Static Margin (% MAC)')
    
    # Calculate and plot optimal margin line
    total_fuel_percentages_line = np.linspace(0.1, 1, 10)
    optimal_ratios = []
    for fuel_percent in total_fuel_percentages_line:
        optimal_ratio = optimize_margin(0.2, fuel_percent, aircraft)
        optimal_ratios.append(optimal_ratio)
    
    # Plot the optimal margin line
    ax.plot(optimal_ratios, total_fuel_percentages_line * 100, 'r-', linewidth=2, label='0.2 Static Margin')
    
    # log scale for x axis
    ax.set_xscale('log')

    ax.set_xlabel('Tail-to-Wing Fuel Mass Ratio (log scale)')
    ax.set_ylabel('Total Fuel (fraction of 7.5e5 lbs)')
    ax.set_title('Static Margin vs Fuel Distribution (Contour)')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Adjust layout and save
    plt.tight_layout()
    plt.savefig('assets/fuel_distribution_stability.png')
    plt.close()
    
    # Return results as tuples
    results = []
    for i in range(len(wing_fuel_percentages)):
        for j in range(len(tail_fuel_percentages)):
            results.append((
                total_fuel_percentages[i, j],
                wing_to_tail_ratios[i, j],
                static_margins[i, j]
            ))
    
    return results

def analyze_aircraft_static_stability(aircraft: Aircraft, landing_configuration = False, output_filename = None):
    """
    Perform static stability analysis on an aircraft and save results to a JSON file.
    
    Parameters:
    aircraft: Aircraft object to analyze
    output_filename: Name of the output JSON file
    
    Returns:
    dict: Dictionary containing all analysis results
    """
    # set total fuel to 5.9e5 lbs and distribute fuel with optimal ratio
    optimize_fuel = False
    if optimize_fuel: 
        total_fuel_mass = 5.9e5
        optimal_ratio = optimize_margin(0.2, total_fuel_mass / (5e5 + 2.5e5), aircraft)
        wing_fuel_mass = total_fuel_mass / (1 + optimal_ratio)
        tail_fuel_mass = total_fuel_mass - wing_fuel_mass
        
        # set wing tank fill level
        for tank in aircraft.wing.children:
            if isinstance(tank, FuelTank):
                tank.set_fill_level(wing_fuel_mass / 5e5)
        
        # set tail tank fill level
        for tank in aircraft.horizontal_tail.children + aircraft.vertical_tail.children:
            if isinstance(tank, FuelTank):
                tank.set_fill_level(tail_fuel_mass / 2.5e5)
        
    
    # Run mass analysis to get current mass properties
    #aircraft.run_analysis('mass_analysis', analyze_children=True)
    #mass_results = aircraft.analysis_results['mass_analysis']
    
    # Get aircraft parameters
    aircraft_params = aircraft_to_parameters(aircraft)

    if landing_configuration:
        aircraft_params['wing_zero_lift_angle'] = np.radians(-13)
        aircraft_params['wing_area'] += aircraft.wing.flap_area*1.2
        aircraft_params['optimize_tail_incidence'] = False
    
    # Calculate tail volume coefficients
    h_tail_volume = aircraft_params['htail_area'] * aircraft_params['htail_arm'] / (
        aircraft_params['wing_area'] * aircraft_params['mac'])
    v_tail_volume = aircraft_params['vertical_tail_area'] * aircraft_params['vertical_tail_arm'] / (
        aircraft_params['wing_area'] * aircraft_params['wingspan'])
    
    # Perform all stability analyses
    longitudinal_results = longitudinal_stability_analysis(aircraft_params)
    directional_results = directional_stability_analysis(aircraft_params)
    lateral_results = lateral_control_analysis(aircraft_params)
    trim_results = trim_and_control_analysis(aircraft_params, aircraft_params)
    engine_out_results = engine_out_analysis(aircraft_params)
    
    # Create comprehensive results dictionary
    analysis_results = {
        'aircraft_parameters': aircraft_params,
        #'mass_properties': mass_results,
        'tail_volumes': {
            'horizontal_tail_volume': h_tail_volume,
            'vertical_tail_volume': v_tail_volume
        },
        'stability_analyses': {
            'longitudinal_stability': longitudinal_results,
            'directional_stability': directional_results,
            'lateral_control': lateral_results,
            'trim_and_control': trim_results,
            'engine_out': engine_out_results
        }
    }
    
    if output_filename:
        # Save to JSON file
        try:
            with open(output_filename, "w") as f:
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
                
                serializable_results = convert_to_serializable(analysis_results)
                json.dump(serializable_results, f, indent=4)
                print(f"\nStability analysis saved to '{output_filename}'")
        except Exception as e:
            print(f"Error saving stability analysis: {e}")
            print("\nAnalysis results:")
            print(json.dumps(analysis_results, indent=4))
    
    return aircraft_params, analysis_results


if __name__ == "__main__":
    # Create base aircraft
    aircraft = Aircraft()
    
    #fuel_distribution_stability_study(aircraft)
    #print("Fuel distribution stability study completed.")
    
    stability_results = analyze_aircraft_static_stability(aircraft, "static_stability_analysis.json")
    print("Static stability analysis completed.")
    pprint(stability_results)
