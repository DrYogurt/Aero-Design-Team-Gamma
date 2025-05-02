from aircraft_design.final_design.final_construction import Aircraft
from aircraft_design.final_design.final_trade_studies.static_stability_utils import aircraft_to_parameters
from aircraft_design.analysis.static_stability import analyze_aircraft_stability
from aircraft_design.components.propulsion.fuel_tanks import FuelTank
import matplotlib.pyplot as plt
import numpy as np
import copy

def analyze_fuel_distribution():
    """
    Analyze static margins across different total fuel levels by varying fuel distribution
    between wing and tail tanks.
    """
    # Create base aircraft
    aircraft = Aircraft()
    
    # Get wing and tail fuel tanks
    wing_tanks = [child for child in aircraft.wing.children if isinstance(child, FuelTank)]
    tail_tanks = [child for child in aircraft.horizontal_tail.children if isinstance(child, FuelTank)]
    
    # Calculate maximum capacities based on full_mass
    wing_max_capacity = sum(tank.full_mass for tank in wing_tanks)
    tail_max_capacity = sum(tank.full_mass for tank in tail_tanks)
    total_max_capacity = wing_max_capacity + tail_max_capacity
    
    print(f"\n=== Tank Capacities ===")
    print(f"Wing tanks max capacity: {wing_max_capacity:.1f} lbs")
    print(f"Tail tanks max capacity: {tail_max_capacity:.1f} lbs")
    print(f"Total max capacity: {total_max_capacity:.1f} lbs")
    
    # Create arrays to store results - reduced number of points
    total_fuel_levels = np.linspace(1000, total_max_capacity, 30)  # lbs
    wing_fill_ratios = np.linspace(0, 1, 30)  # Wing fill percentage (0 to 100%)
    
    # Create mesh grid for contour plot
    X, Y = np.meshgrid(total_fuel_levels, wing_fill_ratios)
    Z = np.zeros_like(X)  # Will store static margins
    CGs = np.zeros((len(wing_fill_ratios), len(total_fuel_levels), 3))  # Will store CG positions
    
    # Analyze for each total fuel level and wing fill ratio
    for i, total_fuel in enumerate(total_fuel_levels):
        for j, wing_fill_ratio in enumerate(wing_fill_ratios):
            # Create a copy of the aircraft for this configuration
            test_aircraft = copy.deepcopy(aircraft)
            
            # Calculate fuel amounts, respecting tank capacities
            wing_fuel = min(total_fuel * wing_fill_ratio, wing_max_capacity)
            tail_fuel = min(total_fuel * (1 - wing_fill_ratio), tail_max_capacity)
            
            # Calculate fill percentages
            wing_fill_percentage = wing_fuel / wing_max_capacity
            tail_fill_percentage = tail_fuel / tail_max_capacity
            
            # Distribute fuel among tanks proportionally to their capacity
            for tank in wing_tanks:
                tank.set_fill_level(wing_fill_percentage)
            
            for tank in tail_tanks:
                tank.set_fill_level(tail_fill_percentage)
            
            # Get mass properties
            mass_props = test_aircraft.get_mass_properties()
            CGs[j, i] = [mass_props['cg_x'], mass_props['cg_y'], mass_props['cg_z']]
            
            # Analyze stability
            aircraft_params = aircraft_to_parameters(test_aircraft)
            stability_results = analyze_aircraft_stability("full", aircraft_params)
            static_margin = stability_results['longitudinal_stability']['static_margin']
            
            Z[j, i] = static_margin
    
    # Find the configuration with minimum static margin (farthest back CG)
    min_margin_idx = np.unravel_index(np.argmin(Z), Z.shape)
    farthest_back_cg = CGs[min_margin_idx]
    min_margin_fuel = total_fuel_levels[min_margin_idx[1]]
    min_margin_wing_fill = wing_fill_ratios[min_margin_idx[0]] * 100
    
    # Create plots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Plot static margin contours with optimal ratio overlay
    contour = ax1.contourf(X, Y, Z, levels=20, cmap='viridis')
    plt.colorbar(contour, ax=ax1, label='Static Margin')
    
    # Plot optimal ratio line
    optimal_ratios = wing_fill_ratios[np.argmin(Z, axis=0)]
    ax1.plot(total_fuel_levels, optimal_ratios, 'r-', linewidth=2, label='Optimal Wing Fill Percentage')
    
    ax1.set_xlabel('Total Fuel (lbs)')
    ax1.set_ylabel('Wing Fill Percentage')
    ax1.set_title('Static Margin Contours with Optimal Fill Distribution')
    ax1.grid(True)
    
    # Plot maximum and minimum static margins
    max_margins = np.max(Z, axis=0)
    min_margins = np.min(Z, axis=0)
    ax2.plot(total_fuel_levels, max_margins, 'b-', label='Maximum Static Margin')
    ax2.plot(total_fuel_levels, min_margins, 'r-', label='Minimum Static Margin')
    ax2.set_xlabel('Total Fuel (lbs)')
    ax2.set_ylabel('Static Margin')
    ax2.set_title('Static Margin vs Total Fuel')
    ax2.grid(True)
    ax2.legend()
    
    # Add vertical lines for capacity limits
    for ax in [ax1, ax2]:
        ax.axvline(x=wing_max_capacity, color='r', linestyle='--', alpha=0.5, label='Wing Capacity')
        ax.axvline(x=tail_max_capacity, color='b', linestyle='--', alpha=0.5, label='Tail Capacity')
        ax.axvline(x=total_max_capacity, color='k', linestyle='--', alpha=0.5, label='Total Capacity')
        ax.legend()
    
    plt.tight_layout()
    plt.savefig('assets/fuel_distribution_trade_study.png')
    plt.close()
    
    # Print summary
    print("\n=== Fuel Distribution Trade Study Results ===")
    print(f"Maximum static margin: {np.max(Z):.3f}")
    print(f"Minimum static margin: {np.min(Z):.3f}")
    print(f"Average optimal wing fill percentage: {np.mean(optimal_ratios)*100:.1f}%")
    print(f"\n=== Farthest Back CG Configuration ===")
    print(f"CG Position: ({farthest_back_cg[0]:.2f}, {farthest_back_cg[1]:.2f}, {farthest_back_cg[2]:.2f}) ft")
    print(f"Total Fuel: {min_margin_fuel:.1f} lbs")
    print(f"Wing Fill Percentage: {min_margin_wing_fill:.1f}%")

if __name__ == "__main__":
    analyze_fuel_distribution()
