aircraft_params = dict(
    # Linear dimensions given in Feet
    wing_area = 9360,
    wingspan = 315,
    #aspect_ratio = aspect_ratio(wingspan, wing_area),
    root_chord = 49.53,
    tip_chord = 9.91,
    #lambda_ratio = taper_ratio(tip_chord, root_chord),
    #taper_ratio = lambda_ratio,
    sweep_angle = 33, # Degrees
    #mac = mean_aerodynamic_chord(root_chord,lambda_ratio),
    tail_area = 9360.8, 
    #tail_arm = , # Definition
    #cg_position = , # User-defined
    #ac_position = , # Definition
    #tail_ac_position = ,
    #initial_tail_incidence = , # Default 0
    #tail_incidence = ,
    #tail_efficiency = ,
    #wing_lift_slope = .33, 
    #tail_lift_slope = 0.18,
    #cm_ac = ,
    #d_epsilon_d_alpha = , # Necessary to split for htail & wing
    #section_lift_slope = ,
    #aileron_effectiveness = ,
    #aileron_efficiency = ,
    #aileron_inner_location = ,
    #aileron_outer_location = ,
    #vertical_tail_area = 1861,
    #vertical_tail_arm = ,
    #vertical_tail_efficiency = ,
    #vertical_tail_lift_slope = 0.11,
    #epsilon_0 = ,
    #max_rudder_deflection = 25, # Degrees
)

print(trim_cruise_lift_coefficient(aircraft_params))
class Aircraft_params:
    def __init__(self, wing_area, wingspan, root_chord, tip_chord, htail_area, htail_mean_chord, htail_arm, htail_ac_position, vtail_mean_chord, vtail_area, cg_position, sweep_angle, aircraft_weight):
        self.wing_area = wing_area
        self.wingspan = wingspan
        self.root_chord = root_chord
        self.tip_chord = tip_chord
        self.htail_mean_chord = htail_mean_chord
        self.htail_area = htail_area
        self.htail_ac_position = htail_ac_position
        self.lambda_ratio = taper_ratio(tip_chord, root_chord)
        self.aspect_ratio = aspect_ratio(wingspan, wing_area)
        self.lambda_ratio = taper_ratio(tip_chord, root_chord)
        self.mac = mean_aerodynamic_chord(root_chord, self.lambda_ratio)
        self.htail_arm = htail_arm
        self.vtail_mean_chord = vtail_mean_chord
        self.vtail_area = vtail_area
        self.cg_position = cg_position
        self.sweep_angle = sweep_angle
        self.ac_position = (0.25*root_chord + distance_AC_behind_quarter_chord(self.lambda_ratio, self.wingspan, self.sweep_angle)) / self.mac # Defines Hac, distance of AC behind LE of MAC
        self.aircraft_weight = aircraft_weight
        
        self.tail_incidence = ((a + eta_t*a_t*S_t/S*(1-depsilon_dalpha))*alpha_e- trim_cruise_lift_coefficient(aircraft_params, flight_conditions) ) / (eta_t*a_t*S_t/S)
        
        
        
aircraft_params = Aircraft_params(
    wing_area = 9360,
    wingspan = 315,
    root_chord = 49.53,
    tip_chord = 9.91,
    htail_area = 2239,
    htail_mean_chord = 24.8,
    htail_arm = None, # Distance Tail-to-CG -- Design defined
    htail_ac_position = None, # Distance of tail above vortex plane simplified to be height of tail above midspan chord of wing -- Design defined
    vtail_mean_chord = 35.192,
    vtail_area = 1861,
    cg_position = None, # Design-defined
    sweep_angle = 34, # Degrees
    aircraft_weight = 1575000, # Lbs.
    
    
)