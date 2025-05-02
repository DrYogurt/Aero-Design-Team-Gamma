# %% [markdown]
# # Stability Trade Studies
# We begin by creating the aircraft



# %%
from aircraft_design.final_design.final_construction import Aircraft
from aircraft_design.analysis.mass_analysis import MassAnalysis, MassFeature
import copy
from pprint import pprint

aircraft = Aircraft()

empty_aircraft = copy.deepcopy(aircraft)
from aircraft_design.components.propulsion.fuel_tanks import FuelTank
# for each of the fuel tanks, set the fill level to 0
for tank in empty_aircraft.wing.children:
    if isinstance(tank, FuelTank):
        tank.set_fill_level(0)
for tank in empty_aircraft.horizontal_tail.children:
    if isinstance(tank, FuelTank):
        tank.set_fill_level(0)
for tank in empty_aircraft.vertical_tail.children:
    if isinstance(tank, FuelTank):
        tank.set_fill_level(0)


# print total aircraft mass
full_mass_test_aircraft = copy.deepcopy(aircraft)
full_mass_test_aircraft.run_analysis('mass_analysis', analyze_children=True)
full_mass_results = full_mass_test_aircraft.analysis_results['mass_analysis']
total_mass = full_mass_results['total_mass']
total_x_cg = full_mass_results['cg_x']
print(f"Total aircraft mass: {total_mass:.2f} lb")
print(f"Total aircraft x CG: {total_x_cg:.2f} ft")

empty_mass_test_aircraft = copy.deepcopy(empty_aircraft)
empty_mass_test_aircraft.run_analysis('mass_analysis', analyze_children=True)
empty_mass_results = empty_mass_test_aircraft.analysis_results['mass_analysis']
total_mass = empty_mass_results['total_mass']
total_x_cg = empty_mass_results['cg_x']
print(f"Total aircraft mass: {total_mass:.2f} lb")
print(f"Total aircraft x CG: {total_x_cg:.2f} ft")

# %%
import matplotlib.pyplot as plt
from aircraft_design.core.plotting import plot_side_view, plot_cg

full_aircraft_mass_props = full_mass_results
empty_aircraft_mass_props = empty_mass_results

for aircraft,mass_props,fuel_config in [(aircraft,full_aircraft_mass_props,'Full'), (empty_aircraft,empty_aircraft_mass_props,'Empty')]:
    obj = aircraft.plot()
    # Side view
    fig_side = plt.figure(figsize=(15, 6))
    ax = fig_side.add_subplot(111)
    _, ax = plot_side_view(obj, fig=fig_side, ax=ax)
    plot_cg(ax, mass_props['cg_x'], mass_props['cg_z'], 
            color='red', markersize=15, label='Aircraft CG')

    ax.set_title(f"{fuel_config} Fuel Configuration - Aircraft - Side View")
    ax.set_aspect('equal')
    ax.legend(loc='upper right')

    plt.tight_layout()
    plt.show()

# %%
# print mass properties
for mass_props in [full_aircraft_mass_props,empty_aircraft_mass_props]:
    pprint(f"Mass properties: {mass_props}")

# %% [markdown]
# ## Static Stability
# This trade study calculates and returns some important static stability values. Since we're manually setting the mass, run this twice with/without fuel in the mass estimates and assume we can keep the CG close enough

# %%
from aircraft_design.final_design.final_trade_studies.static_stability_trade_study import analyze_aircraft_static_stability
from aircraft_design.final_design.final_trade_studies.static_margin_analysis import static_margin_at_full_and_empty
# copy aircraft
aircraft_static_stability_copy = copy.deepcopy(aircraft)

aircraft_params, analysis_results = analyze_aircraft_static_stability(empty_aircraft
                                                                      ,landing_configuration=False)

full_static_margin, empty_static_margin, full_cg_position, empty_cg_position = static_margin_at_full_and_empty(aircraft)
print(f"Full fuel static margin: {full_static_margin:.2f}")
print(f"Empty fuel static margin: {empty_static_margin:.2f}")
print(f"Full fuel CG position: {full_cg_position:.2f} ft")
print(f"Empty fuel CG position: {empty_cg_position:.2f} ft")

pprint(analysis_results['stability_analyses'])

# %% [markdown]
# ## Dynamic Stability - Dimensionless Params
# This trade study calculates the dimensionless derivatives for dynamic stability, and calculates the steady roll rate (which should be at least 0.6.

# %%
from aircraft_design.final_design.final_trade_studies.dynamic_stability_trade_study import create_stability_parameters
from ambiance import Atmosphere
import numbers
flight_conditions = [
    # M, h
    (0.25, 0),
    (0.5,20e3),
    (0.5,36e3),
    (0.9,40e3)
]
flight_conditions_stability_params = []
for M, h in flight_conditions:
    rho = Atmosphere(h).density
    a = Atmosphere(h).speed_of_sound
    V0 = M * a
    flight_conditions_stability_params.append((V0,rho))

aircraft_dynamic_stability_copy = copy.deepcopy(aircraft)
all_dimensionless_derivatives = {}
for rho, V0 in flight_conditions_stability_params:  
    aircraft_params, stability_params, dimensionless_derivatives = create_stability_parameters(aircraft_dynamic_stability_copy, rho=rho, V0=V0)
    for key, value in dimensionless_derivatives.items():
        if key in all_dimensionless_derivatives:
            all_dimensionless_derivatives[key].append(value if isinstance(value, numbers.Number) else value[0])
        else:
            all_dimensionless_derivatives[key] = [value] if isinstance(value, numbers.Number) else list(value)
    dimensionless_roll_rate = dimensionless_derivatives['C_l_delta_a'] / dimensionless_derivatives['C_l_p']
    steady_roll_rate = dimensionless_roll_rate * 2 * aircraft_params['airspeed'] / aircraft_params['wingspan']
    if 'steady_roll_rate' in all_dimensionless_derivatives:
        all_dimensionless_derivatives['steady_roll_rate'].append(steady_roll_rate)
    else:
        all_dimensionless_derivatives['steady_roll_rate'] = [steady_roll_rate]

pprint(all_dimensionless_derivatives)

for key, values in dimensionless_derivatives.items():
    print(f"{key}: {', '.join(f'{v:.3f}' for v in values)}")


# %% [markdown]
# ## Dynamic Stability - Frequencies, Damping Ratios and Time Constants
# This trade study calculates and returns all the values for ensuring first order dynamic stability 

# %%
from aircraft_design.final_design.final_trade_studies.dynamic_stability_trade_study import analyze_aircraft_dynamic_stability, plot_gain_sensitivity

aircraft_dynamic_stability_copy = copy.deepcopy(aircraft)



# find passive values
short_nf, short_df, p_nf, p_df, dnf, ddr, Tr, Ts = analyze_aircraft_dynamic_stability(aircraft_dynamic_stability_copy, visualize=False)

print(f"====== Passive values ====== \n short_nf: {short_nf:.2f}, short_df: {short_df:.2f}, p_nf: {p_nf:.2f}, p_df: {p_df:.2f}, dnf: {dnf:.2f}, ddr: {ddr:.2f}, Tr: {Tr:.2f}, Ts: {Ts:.2f}")

# currently these gains are bad
test_gains = {
        'ku': [0, 0], 'kw': [0, 0], 'kq': [0, 0], 'ko_long': [0, 0],
        'kv': [0, 0], 'kp': [0, 0], 'kr': [0, 0], 'ko_lat': [0, 0]
    }


# plot long gain sensitivity
for gain in []:#['ku', 'kw', 'kq', 'ko_long']:
    plot_gain_sensitivity(aircraft_dynamic_stability_copy, gain, base_gains=test_gains)
    print(f"Finished {gain}")

#plot lat gain sensitivity
for gain in ['kv', 'kp', 'kr', 'ko_lat']:
    plot_gain_sensitivity(aircraft_dynamic_stability_copy, gain, base_gains=test_gains)
    print(f"Finished {gain}")

# %%
import json
import os
from typing import List, Dict, Any
from pprint import pprint

def get_best_solutions(
    solutions_dir: str,
    best_longitudinal: int,
    best_lateral: int
) -> Dict[str, List[Dict[str, Any]]]:
    """
    Extract the n best longitudinal and m best lateral solutions based on cost.
    
    Args:
        solutions_dir: Path to the directory containing solution files
        n_longitudinal: Number of best longitudinal solutions to return
        m_lateral: Number of best lateral solutions to return
        
    Returns:
        Dictionary with two keys:
        - 'longitudinal': List of n best longitudinal solutions
        - 'lateral': List of m best lateral solutions
    """
    longitudinal_solutions = []
    lateral_solutions = []
    
    # Read all solution files
    for filename in os.listdir(solutions_dir):
        if not filename.endswith('.json'):
            continue
            
        filepath = os.path.join(solutions_dir, filename)
        with open(filepath, 'r') as f:
            solution = json.load(f)
            
        # Add solution to appropriate list
        if 'longitudinal' in filename:
            longitudinal_solutions.append(solution)
        elif 'lateral' in filename:
            lateral_solutions.append(solution)
    
    # Filter longitudinal solutions to only include those with short_nf < 5
    longitudinal_solutions = [sol for sol in longitudinal_solutions if sol['results']['short_nf'] < 5]
    
    # Sort solutions by p_nf (descending)
    longitudinal_solutions.sort(key=lambda x: -x['results']['p_nf'])
    lateral_solutions.sort(key=lambda x: x['cost'])
    
    return {
        'longitudinal': longitudinal_solutions[:best_longitudinal],
        'lateral': lateral_solutions[:best_lateral]
    }

best_solutions = get_best_solutions(
    solutions_dir='assets/dynamic_stability/solutions',
    best_longitudinal=25,  # Get 5 best longitudinal solutions
    best_lateral=5        # Get 5 best lateral solutions
)

# Access the solutions
best_longitudinal = best_solutions['longitudinal']
best_lateral = best_solutions['lateral']

for i in range(len(best_longitudinal)):
    pprint(best_longitudinal[i])
for i in range(len(best_lateral)):
    pprint(best_lateral[i])

# %%
from aircraft_design.final_design.final_trade_studies.dynamic_stability_trade_study import analyze_aircraft_dynamic_stability
from aircraft_design.final_design.final_construction import Aircraft
import numpy as np
# prepare for optimization
aircraft = Aircraft()
test_gains = {'ko_long': [0.4253810369570412, -1.0895938907733194],
           'kq': [2.9463312437697606, -0.641640554927314],
           'ku': [-0.1443077700100025, 1.1621887321320818],
           'kw': [0.044401442882121556, -2.2524414223174922],
           }

base_gains_lon = [test_gains['ku'][0], test_gains['ku'][1], test_gains['kw'][0], test_gains['kw'][1], test_gains['kq'][0], test_gains['kq'][1], test_gains['ko_long'][0], test_gains['ko_long'][1]]
base_gains_lat = np.zeros(8) #[test_gains['kv'][0], test_gains['kv'][1], test_gains['kp'][0], test_gains['kp'][1], test_gains['kr'][0], test_gains['kr'][1], test_gains['ko_lat'][0], test_gains['ko_lat'][1]]


analyze_aircraft_dynamic_stability(aircraft, **test_gains, visualize=True)



# %%
import matplotlib.pyplot as plt
aircraft_params, stability_params, dimensionless_derivatives = create_stability_parameters(aircraft_dynamic_stability_copy, rho=rho, V0=V0)

dimensionless_roll_rate = dimensionless_derivatives['C_l_delta_a'] / dimensionless_derivatives['C_l_p']
V0s = np.linspace(200,900,20)
rho = Atmosphere(36000/3.28084).density
steady_roll_rates = []
for V0 in V0s:
    steady_roll_rate = dimensionless_roll_rate * 2 * V0 / 315
    steady_roll_rates.append(steady_roll_rate*6)

plt.plot(V0s, steady_roll_rates)
plt.xlabel('Airspeed (ft/s)')
plt.ylabel('Degrees Deflection Needed for 6 deg/s Roll Rate')
plt.title('Steady Roll Rate vs Airspeed')
#draw a vertical line at 200 ft/s
plt.axvline(x=356.094, color='red', linestyle='--',label='$1.3 V_{stall}$')


plt.legend()
plt.show()

# %%
import numpy as np
# Extract relevant parameters from the aircraft
S = 9812.599#aircraft_params['wing_area']
b = 315 #aircraft_params['wingspan']

# Engine-out yawing moment (from previous calculation)
engine_moment = (48 + 2*48) * 89381.250  # Using the engine thrust defined above

# Calculate rudder deflection needed at different airspeeds
V0s = np.linspace(200, 900, 20)
rudder_deflections = []
rudder_deflections_degrees = []

for V0 in V0s:
    # Dynamic pressure
    q = 0.5 * rho * V0**2
    
    # Yaw moment coefficient due to rudder
    CN_delta_r = dimensionless_derivatives['C_N_delta_r']
    print(CN_delta_r)
    # Required rudder deflection
    delta_r = engine_moment / (q * S * b) / CN_delta_r
    
    rudder_deflections.append(delta_r)
    rudder_deflections_degrees.append(np.degrees(delta_r))

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(V0s, rudder_deflections_degrees)
plt.xlabel('Airspeed (ft/s)')
plt.ylabel('Rudder Deflection Required (degrees)')
plt.title('Rudder Deflection Required for Engine-Out Condition vs Airspeed')
plt.axvline(x=356.094, color='red', linestyle='--', label='$1.3 V_{stall}$')

# Add horizontal line at maximum rudder deflection (if available)
if 'max_rudder_deflection' in aircraft_params:
    max_deflection_deg = np.degrees(aircraft_params['max_rudder_deflection'])
    #plt.axhline(y=max_deflection_deg, color='green', linestyle='--', 
    #            label=f'Max Rudder Deflection ({max_deflection_deg:.1f}°)')

plt.grid(True)
plt.legend()
plt.show()




# %%
import numpy as np
# Extract relevant parameters from the aircraft
S = 9812.599#aircraft_params['wing_area']
b = 315 #aircraft_params['wingspan']
c = aircraft_params['mac']
V0s = np.linspace(200, 900, 20)
elevator_deflections = []
elevator_deflections_degrees = []
# Define a range of pitch rates to analyze (deg/s)
pitch_rates = np.linspace(1, 20, 20)  # From 1 to 20 deg/s
elevator_deflections = []
elevator_deflections_degrees = []

# Use a fixed airspeed (e.g., takeoff speed)
V0 = 328.702

# Get relevant coefficients
C_M_q = dimensionless_derivatives['C_M_q']
C_M_alpha = dimensionless_derivatives['C_M_alpha']
C_M_delta_e = dimensionless_derivatives['C_M_delta']

for pitch_rate_deg in pitch_rates:
    # Convert pitch rate from deg/s to rad/s
    pitch_rate_rad = np.radians(pitch_rate_deg)
    
    # Calculate required elevator deflection
    delta_e = (C_M_q + C_M_alpha) * pitch_rate_rad * c / (2 * V0) / C_M_delta_e
    
    elevator_deflections.append(delta_e)
    elevator_deflections_degrees.append(np.degrees(delta_e))

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(pitch_rates, elevator_deflections_degrees)
plt.xlabel('Pitch Rate (deg/s)')
plt.ylabel('Elevator Deflection Required (degrees)')
plt.title(f'Elevator Deflection Required vs Pitch Rate at {V0} ft/s')

# Add horizontal line at maximum elevator deflection (if available)
if 'max_elevator_deflection' in aircraft_params:
    max_deflection_deg = np.degrees(aircraft_params['max_elevator_deflection'])
    plt.axhline(y=max_deflection_deg, color='green', linestyle='--', 
                label=f'Max Elevator Deflection ({max_deflection_deg:.1f}°)')

plt.grid(True)
plt.legend()
plt.show()




# %%
1400 / 328

# %%



