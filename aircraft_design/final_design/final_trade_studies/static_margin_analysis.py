from aircraft_design.final_design.final_construction import Aircraft
from aircraft_design.final_design.final_trade_studies.static_stability_utils import aircraft_to_parameters
from aircraft_design.analysis.static_stability import analyze_aircraft_stability
from aircraft_design.components.propulsion.fuel_tanks import FuelTank
import matplotlib.pyplot as plt
import numpy as np

def analyze_static_margins():
    # Create arrays to store results
    wing_tip_positions = np.linspace(25, 125, 50)
    full_cg_positions = []
    full_neutral_points = []
    full_static_margins = []
    empty_cg_positions = []
    empty_neutral_points = []
    empty_static_margins = []
    # Analyze for each wing tip position
    for wing_tip_position in wing_tip_positions:
        # Create aircraft instance with current wing tip position
        aircraft = Aircraft(wing_tip_position=wing_tip_position)
        
        # Analyze full fuel configuration
        aircraft_params = aircraft_to_parameters(aircraft)
        stability_results = analyze_aircraft_stability("full", aircraft_params)
        cg_x = aircraft.get_mass_properties()['cg_x']
        mac = aircraft.wing.geometry.mean_aerodynamic_chord
        static_margin = stability_results['longitudinal_stability']['static_margin']
        # Calculate neutral point position (in feet)
        neutral_point = cg_x + static_margin * mac
        full_cg_positions.append(cg_x)
        full_neutral_points.append(neutral_point)
        full_static_margins.append(static_margin)
        # Analyze drained fuel configuration
        # Empty all fuel tanks
        for child in aircraft.wing.children:
            if isinstance(child, FuelTank):
                child.set_fill_level(0.0)
        # Empty horizontal tail tanks
        for child in aircraft.horizontal_tail.children:
            if isinstance(child, FuelTank):
                child.set_fill_level(0.0)
        # Empty vertical tail tanks
        for child in aircraft.vertical_tail.children:
            if isinstance(child, FuelTank):
                child.set_fill_level(0.0)
        
        aircraft_params = aircraft_to_parameters(aircraft)
        stability_results = analyze_aircraft_stability("drained", aircraft_params)
        cg_x = aircraft.get_mass_properties()['cg_x']
        static_margin = stability_results['longitudinal_stability']['static_margin']
        # Calculate neutral point position (in feet)
        neutral_point = cg_x + static_margin * mac
        empty_cg_positions.append(cg_x)
        empty_neutral_points.append(neutral_point)
        empty_static_margins.append(static_margin)
    # Create plots with twin axes
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))
    
    # Plot full fuel configuration
    ax1_twin = ax1.twinx()
    ax1.plot(wing_tip_positions, full_cg_positions, 'b-', label='CG Position')
    ax1.plot(wing_tip_positions, full_neutral_points, 'r-', label='Neutral Point')
    ax1_twin.plot(wing_tip_positions, full_static_margins, 'g-', label='Static Margin')
    ax1.set_xlabel('Wing Tip Position (ft)')
    ax1.set_ylabel('Position (ft)', color='b')
    ax1.set_title('Full Fuel Configuration')
    ax1.grid(True)
    # Combine legends
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1_twin.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    
    # Plot empty fuel configuration
    ax2_twin = ax2.twinx()
    ax2.plot(wing_tip_positions, empty_cg_positions, 'b-', label='CG Position')
    ax2.plot(wing_tip_positions, empty_neutral_points, 'r-', label='Neutral Point')
    ax2_twin.plot(wing_tip_positions, empty_static_margins, 'g-', label='Static Margin')
    ax2.set_xlabel('Wing Tip Position (ft)')
    ax2.set_ylabel('Position(ft)', color='b')
    ax2.set_title('Empty Fuel Configuration')
    ax2.grid(True)
    # Combine legends
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_twin.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    
    plt.tight_layout()
    plt.savefig('assets/static_margin_analysis.png')
    plt.close()

def static_margin_at_full_and_empty():
    """
    Calculate and return the static margin for both full and empty fuel configurations
    using the optimal wing tip position.
    
    Returns:
        tuple: (full_static_margin, empty_static_margin) - Static margins as decimal values
    """
    # Create aircraft with optimal wing tip position (based on previous analysis)
    #optimal_wing_position = 70  # feet, based on previous analysis
    aircraft = Aircraft()
    
    # Calculate full fuel static margin
    aircraft_params = aircraft_to_parameters(aircraft)
    stability_results = analyze_aircraft_stability("full", aircraft_params)
    full_static_margin = stability_results['longitudinal_stability']['static_margin']
    
    # Get full fuel CG position
    full_cg_position = aircraft.get_mass_properties()['cg_x']
    
    # Empty all fuel tanks
    for component in [aircraft.wing, aircraft.horizontal_tail, aircraft.vertical_tail]:
        for child in component.children:
            if isinstance(child, FuelTank):
                child.set_fill_level(0.0)
    
    # Calculate empty fuel static margin
    aircraft_params = aircraft_to_parameters(aircraft)
    stability_results = analyze_aircraft_stability("empty", aircraft_params)
    empty_static_margin = stability_results['longitudinal_stability']['static_margin']
    
    # Get empty fuel CG position
    empty_cg_position = aircraft.get_mass_properties()['cg_x']
    
    return (full_static_margin, empty_static_margin, full_cg_position, empty_cg_position)

if __name__ == "__main__":
    full_static_margin, empty_static_margin, full_cg_position, empty_cg_position = static_margin_at_full_and_empty()
    print(f"Full fuel static margin: {full_static_margin:.2f}")
    print(f"Empty fuel static margin: {empty_static_margin:.2f}")
    print(f"Full fuel CG position: {full_cg_position:.2f} ft")
    print(f"Empty fuel CG position: {empty_cg_position:.2f} ft")
    analyze_static_margins() 