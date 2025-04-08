from aircraft_design.final_design.final_construction import Aircraft
from aircraft_design.final_design.final_trade_studies.static_stability_utils import aircraft_to_parameters
from aircraft_design.analysis.static_stability import analyze_aircraft_stability
from aircraft_design.components.propulsion.fuel_tanks import FuelTank

def analyze_static_margins():
    # Create aircraft instance
    aircraft = Aircraft()
    
    # Analyze full fuel configuration
    print("\n=== Full Fuel Configuration ===")
    aircraft_params = aircraft_to_parameters(aircraft)
    stability_results = analyze_aircraft_stability("full", aircraft_params)
    full_static_margin = stability_results['longitudinal_stability']['static_margin']
    print(f"Static Margin: {full_static_margin:.3f} MAC")
    
    # Analyze drained fuel configuration
    print("\n=== Drained Fuel Configuration ===")
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
    drained_static_margin = stability_results['longitudinal_stability']['static_margin']
    print(f"Static Margin: {drained_static_margin:.3f} MAC")
    
    # Print difference
    print("\n=== Comparison ===")
    print(f"Static Margin Change: {drained_static_margin - full_static_margin:.3f} MAC")
    print(f"Percentage Change: {((drained_static_margin - full_static_margin) / full_static_margin * 100):.1f}%")

if __name__ == "__main__":
    analyze_static_margins() 