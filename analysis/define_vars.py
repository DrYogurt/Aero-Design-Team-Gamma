from global_variables.registry import VariableRegistry, Variable

registry = VariableRegistry("aero_vars.yaml")

# Atmospheric properties
registry.atmosphere.density = Variable(
    name="Air density",
    symbol="rho",
    function_str="lambda h: Atmosphere(h).density",
    function_args=['h']
)

registry.atmosphere.pressure = Variable(
    name="Air pressure",
    symbol="P",
    function_str="lambda h: Atmosphere(h).pressure",
    function_args=['h']
)

registry.atmosphere.temperature = Variable(
    name="Air temperature",
    symbol="T",
    function_str="lambda h: Atmosphere(h).temperature",
    function_args=['h']
)

# Weights
registry.aircraft.weights.max = Variable(
    name="Maximum takeoff weight",
    symbol="W_max",
    minimum=0
)

registry.aircraft.weights.empty_fraction = Variable(
    name="Empty weight fraction",
    symbol="We_Wmax",
    minimum=0,
    maximum=1
)

registry.aircraft.weights.empty = Variable(
    name="Empty weight",
    symbol="W_e",
    definitions=["We_Wmax * W_max"]
)

registry.aircraft.weights.passenger.count = Variable(
    name="Number of passengers",
    symbol="n_pax",
    minimum=1200
)

registry.aircraft.weights.passenger.unit = Variable(
    name="Weight per passenger",
    symbol="W_pax",
    minimum=200
)

registry.aircraft.weights.payload = Variable(
    name="Payload weight",
    symbol="W_p",
    definitions=["n_pax * W_pax"]
)

registry.aircraft.weights.fuel = Variable(
    name="Fuel weight",
    symbol="W_f",
    definitions=["W_max - W_p - W_e"]
)

# Wing geometry
registry.aircraft.wing.span = Variable(
    name="Wing span",
    symbol="b",
    minimum=0
)

registry.aircraft.wing.area = Variable(
    name="Wing area",
    symbol="S_",
    minimum=0
)

registry.aircraft.wing.aspect_ratio = Variable(
    name="Aspect ratio",
    symbol="AR",
    definitions=["b**2 / S_"]
)

registry.aircraft.wing.efficiency = Variable(
    name="Oswald efficiency factor",
    symbol="e",
    minimum=0,
    maximum=1
)

# dummy variables
registry.aero.coefficients.dummy.velocity = Variable(
    name="Velocity",
    symbol="V",
    minimum=0
)

registry.aero.coefficients.dummy.weight = Variable(
    name="Weight",
    symbol="W",
    minimum=0
)
registry.aero.coefficients.dummy.pi = Variable(
    name="PI",
    symbol="pi",
    minimum=0
)
registry.aero.coefficients.dummy.gravity = Variable(
    name="Acceleration due to gravity",
    symbol="g",
    definitions=["32.174"]
)

# Aerodynamic coefficients
registry.aero.coefficients.lift.basic = Variable(
    name="Lift coefficient",
    symbol="C_L",
    definitions=["2 * W / (rho * V**2 * S_)"]
)

registry.aero.coefficients.lift_full = Variable(
    name="Lift coefficient at max weight",
    symbol="C_L_full",
    definitions=["2 * W_max / (rho * V**2 * S_)"]
)

registry.aero.coefficients.lift_empty = Variable(
    name="Lift coefficient at empty weight",
    symbol="C_L_empty",
    definitions=["2 * (W_max - W_f) / (rho * V**2 * S_)"]
)

registry.aero.coefficients.lift.max = Variable(
    name="Maximum lift coefficient",
    symbol="C_Lmax",
    minimum=0,
    maximum=1.8
)

registry.aero.coefficients.drag.parasitic = Variable(
    name="Parasitic drag coefficient",
    symbol="C_D0",
    minimum=0
)

registry.aero.coefficients.drag.induced_full = Variable(
    name="Induced drag coefficient",
    symbol="C_Di_full",
    definitions=["C_L_full**2 / (pi * e * AR)"]
)

registry.aero.coefficients.drag.total_drag_full = Variable(
    name="Total drag coefficient",
    symbol="C_D_full",
    definitions=["C_D0 + C_Di_full"]
)
registry.aero.coefficients.drag.induced_empty = Variable(
    name="Induced drag coefficient",
    symbol="C_Di_empty",
    definitions=["C_L_empty**2 / (pi * e * AR)"]
)

registry.aero.coefficients.drag.total_drag_empty = Variable(
    name="Total drag coefficient",
    symbol="C_D_empty",
    definitions=["C_D0 + C_Di_empty"]
)
## Engine parameters
registry.aircraft.engine.thrust.available = Variable(
    name="Available thrust",
    symbol="T_A0",
    minimum=0
)

registry.aircraft.engine.thrust.static = Variable(
    name="Static thrust",
    symbol="T_s",
    minimum=0
)

registry.aircraft.engine.TSFC = Variable(
    name="Thrust specific fuel consumption",
    symbol="TSFC",
    minimum=0
)

# Performance polynomial coefficients
registry.performance.coefficients.A = Variable(
    name="Coefficient A",
    symbol="A",
    definitions=["rho * C_D0 * S_ / 2"]
)

registry.performance.coefficients.B = Variable(
    name="Coefficient B",
    symbol="B",
    definitions=["-T_A0"]
)

registry.performance.coefficients.C = Variable(
    name="Coefficient C",
    symbol="C",
    definitions=["2 * W**2 / (rho * S_ * e * AR * pi)"]
)


# Performance - Velocities
registry.performance.velocity.stall = Variable(
    name="Stall velocity",
    symbol="V_stall",
    definitions=["(2 * W / (rho * S_ * C_Lmax)**(1/2))"]
)

registry.performance.velocity.max = Variable(
    name="Maximum velocity",
    symbol="V_max",
    definitions=["((-B + (B**2 - 4*A*C)**(1/2))/(2*A))**(1/2)"]
)

registry.performance.velocity.climb_max = Variable(
    name="Velocity for maximum climb rate",
    symbol="V_RC_max",
    definitions=["((-B + (B**2 + 4*3*A*C)**(1/2))/(2*3*A))**(1/2)"] #replacing A-> 3A and C-> -C
)

# Performance - Climb
registry.performance.climb.max_rate = Variable(
    name="Maximum rate of climb",
    symbol="RC_max",
    definitions=["(A * V_RC_max**3 + B * V_RC_max + C/V_RC_max) / (-W)"]
)

# Performance - Ceiling
registry.performance.ceiling.density_ratio = Variable(
    name="Maximum ceiling density ratio",
    symbol="sigma_max",
    definitions=["(4 * A * C / B**2)**0.5"]
)

# Performance - Range
registry.performance.range.lift_drag_ratio = Variable(
    name="Average lift-to-drag ratio",
    symbol="CL_CD_ratio",
    definitions=["0.5 * ((C_L_full)**(1/2)/C_D_full + (C_L_empty)**(1/2)/C_D_empty)"]
)

registry.performance.range.cruise = Variable(
    name="Cruise range",
    symbol="R",
    definitions=["(2/TSFC) * (2/(rho*S_)**(1/2)) * CL_CD_ratio * ((W_max)**(1/2)-(W_max-W_f)**(1/2))"]
)

# Takeoff parameters
registry.performance.takeoff.friction = Variable(
    name="Ground friction coefficient",
    symbol="mu",
    minimum=0,
    maximum=1
)

registry.performance.takeoff.obstacle_height = Variable(
    name="Obstacle height",
    symbol="h_obstacle",
    definitions=["35"]
)

registry.performance.takeoff.velocity.rotate = Variable(
    name="Rotation velocity",
    symbol="V_1",
    definitions=["1.2 * V_stall"]
)

registry.performance.takeoff.drag = Variable(
    name="Takeoff drag at rotation",
    symbol="D_1",
    definitions=["C_D0 * rho * V_1**2 * S_ / 2"]
)

registry.performance.takeoff.force.net = Variable(
    name="Net force at rotation",
    symbol="F_1",
    definitions=["T_s - D_1"]
)

registry.performance.takeoff.force.static = Variable(
    name="Static net force",
    symbol="F_s",
    definitions=["T_s - mu * W"]
)

registry.performance.takeoff.power.required = Variable(
    name="Power required",
    symbol="P_R",
    definitions=["A * V_1**3 + C / V_1"]
)

registry.performance.takeoff.power.available = Variable(
    name="Power available",
    symbol="P_A",
    definitions=["V_1 * T_s"]
)

registry.performance.takeoff.climb_rate = Variable(
    name="Climb rate",
    symbol="RC",
    definitions=["(P_A - P_R) / W"]
)

registry.performance.takeoff.distance.ground = Variable(
    name="Ground roll distance",
    symbol="S_1",
    definitions=["W/(2*g) * (V_1**2)/(F_s - F_1) * Log(F_s/F_1)"]
)

registry.performance.takeoff.distance.climb = Variable(
    name="Climb distance",
    symbol="S_2",
    definitions=["h_obstacle / RC * V_1"]
)

registry.performance.takeoff.distance.total = Variable(
    name="Total takeoff distance",
    symbol="S_TO",
    definitions=["S_1 + S_2"]
)


