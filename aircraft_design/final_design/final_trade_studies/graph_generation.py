# %%
from aircraft_design.final_design.final_trade_studies.dynamic_stability_trade_study import analyze_aircraft_dynamic_stability
from aircraft_design.final_design.final_construction import Aircraft
import numpy as np
# prepare for optimization
aircraft = Aircraft()

# good short period
test_gains = {'ko_long': [-1.3032519997683814, -1.3369288062892695],
           'kq': [-1.3742821242820142, -1.0245150979564492],
           'ku': [-0.03911437512047744, -1.0050226930594066],
           'kw': [-0.1747477841102023, -1.167616876420973],
            'kv': [-0.01764108, -0.00454934],
            'kp': [-2.,  0.],
            'kr': [-1.64769301, -2.        ],
            'ko_lat': [-1.43494421e+00, -1.07675308e-05]
           }

deflections = {'eta': np.radians(-10),
               'tau': 10,
               'xi': np.radians(2),
               'zeta': np.radians(0),
               'tpar': [0, 0.01, 100]}

short_nf, short_df, p_nf, p_df, dnf, ddr, Tr, Ts = analyze_aircraft_dynamic_stability(aircraft, **test_gains, **deflections, visualize=True)
print(f"short_nf: {short_nf}, short_df: {short_df}, p_nf: {p_nf}, p_df: {p_df}, dnf: {dnf}, ddr: {ddr}, Tr: {Tr}, Ts: {Ts}")

# %%
from atmosphere import Atmosphere

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

short_nf, short_df, p_nf, p_df, dnf, ddr, Tr, Ts = analyze_aircraft_dynamic_stability(aircraft, **test_gains, **deflections, visualize=True)
print(f"short_nf: {short_nf}, short_df: {short_df}, p_nf: {p_nf}, p_df: {p_df}, dnf: {dnf}, ddr: {ddr}, Tr: {Tr}, Ts: {Ts}")

