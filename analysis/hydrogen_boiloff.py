#####
# Implemented following the work done by McFarland and Agarwal:
# https://openscholarship.wustl.edu/cgi/viewcontent.cgi?article=1182&context=mems500
#####

import numpy as np
from scipy.optimize import minimize
from dataclasses import dataclass
from typing import Dict, Tuple, Callable

@dataclass
class FluidProperies:
    density: float  # kg/m3
    dynamic_viscosity: float  # kg/s*m
    heat_capacity: float  # J/kg*K
    thermal_conductivity: float  # W/m*K
    thermal_expansion_coeff: float = None  # K^-1
    prandtl_number: float = None

@dataclass
class MaterialProperties:
    thermal_conductivity: float  # W/m*K
    density: float  # lbs/ft3
    thickness: float = None  # m

@dataclass
class TankGeometry:
    diameter: float  # m
    length: float  # m
    
    @property
    def surface_area(self) -> float:
        """Calculate surface area of cylindrical tank"""
        return np.pi * self.diameter * self.length + 2 * np.pi * (self.diameter/2)**2

def reynolds_number(fluid: FluidProperties, velocity: float, length: float) -> float:
    """Calculate Reynolds number (Equation 5)"""
    return (fluid.density * velocity * length) / fluid.dynamic_viscosity

def prandtl_number(fluid: FluidProperties) -> float:
    """Calculate Prandtl number (Equation 6)"""
    if fluid.prandtl_number is not None:
        return fluid.prandtl_number
    return (fluid.heat_capacity * fluid.dynamic_viscosity) / fluid.thermal_conductivity

def nusselt_number_forced(re: float, pr: float) -> float:
    """Calculate Nusselt number for forced convection (Equation 7)"""
    return 0.644 * (re**0.5) * (pr**(1/3))

def grashof_number(fluid: FluidProperties, t_surface: float, t_ambient: float, 
                  length: float, g: float = 9.81) -> float:
    """Calculate Grashof number (Equation 8)"""
    return (g * fluid.thermal_expansion_coeff * (t_surface - t_ambient) * length**3) / fluid.dynamic_viscosity

def rayleigh_number(gr: float, pr: float) -> float:
    """Calculate Rayleigh number (Equation 9)"""
    return gr * pr

def nusselt_number_natural(ra: float, pr: float) -> float:
    """Calculate Nusselt number for natural convection (Equation 10)"""
    term = 0.387 * ra**(1/6) / (1 + (0.559/pr)**(9/16))**(8/27)
    return (0.60 + term)**2

def convection_coefficient(nu: float, k: float, l: float) -> float:
    """Calculate convection coefficient (Equation 11)"""
    return (nu * k) / l

def heat_transfer_convection(h: float, area: float, t1: float, t2: float) -> float:
    """Calculate convective heat transfer (Equation 12)"""
    return h * area * (t1 - t2)

def thermal_resistance_conduction(thickness: float, k: float, area: float) -> float:
    """Calculate thermal resistance for conduction (Equation 2)"""
    return thickness / (k * area)

def heat_transfer_radiation_solar(absorptivity: float, solar_flux: float, area: float) -> float:
    """Calculate radiative heat transfer from solar radiation (Equation 3)"""
    return absorptivity * solar_flux * area

def heat_transfer_radiation_temp(absorptivity: float, stefan_boltzmann: float, 
                               area: float, t1: float, t2: float) -> float:
    """Calculate radiative heat transfer due to temperature difference (Equation 4)"""
    return absorptivity * stefan_boltzmann * area * (t1**4 - t2**4)

def series_resistance(*resistances: float) -> float:
    """Calculate total resistance for series configuration (Equation 13)"""
    return sum(resistances)

def parallel_resistance(*resistances: float) -> float:
    """Calculate total resistance for parallel configuration (Equation 14)"""
    return 1 / sum(1/r for r in resistances)

def calculate_boiloff_rate(q_total: float, h_vap: float) -> float:
    """Calculate boiloff rate given total heat transfer and heat of vaporization"""
    return q_total / h_vap

def analyze_tank_thermal_performance(
    tank: TankGeometry,
    ambient_fluid: FluidProperties,
    materials: Dict[str, MaterialProperties],
    operating_conditions: dict,
    target_boiloff_rate: float = None,
    beta: float = 1.0  # Tank fullness factor (1.0 = full)
) -> Tuple[float, float]:
    """
    Analyze thermal performance of tank and calculate required insulation thickness
    following the methodology from the paper.
    
    Args:
        tank: Tank geometry parameters
        ambient_fluid: Properties of the ambient fluid (air)
        materials: Dictionary of material properties
        operating_conditions: Operating conditions dictionary
        target_boiloff_rate: Target boiloff rate in lbs/hr (if None, will calculate boiloff for given thickness)
        beta: Tank fullness factor (1.0 = full tank, 0.0 = empty)
    
    Returns:
        Tuple of (boiloff_rate, required_insulation_thickness)
    """
    # Constants
    STEFAN_BOLTZMANN = 5.67e-8  # W/m^2K^4
    
    # Extract operating conditions
    T_ambient = operating_conditions["ambient_temp"]
    T_critical = operating_conditions["critical_temp"]
    solar_flux = operating_conditions["solar_flux"]
    absorptivity = operating_conditions["absorptivity"]
    velocity = operating_conditions.get("cruise_velocity", 0.0)
    
    # Calculate Reynolds number for forced convection
    Re = reynolds_number(ambient_fluid, velocity, tank.diameter)
    
    # Calculate Prandtl number
    Pr = prandtl_number(ambient_fluid)
    
    def calculate_heat_transfer(T_surface: float, insulation_thickness: float) -> float:
        """Calculate total heat transfer for a given surface temperature and insulation thickness"""
        # 1. Solar radiation heat transfer (Q3)
        Q_solar = heat_transfer_radiation_solar(
            absorptivity, solar_flux, tank.surface_area
        )
        
        # 2. Temperature differential radiation (Q1)
        Q_rad = heat_transfer_radiation_temp(
            absorptivity, STEFAN_BOLTZMANN, tank.surface_area,
            T_surface, T_ambient
        )
        
        # 3. Convective heat transfer (Q2)
        if velocity > 0:  # Forced convection
            Nu = nusselt_number_forced(Re, Pr)
        else:  # Natural convection
            Gr = grashof_number(ambient_fluid, T_surface, T_ambient, tank.length)
            Ra = rayleigh_number(Gr, Pr)
            Nu = nusselt_number_natural(Ra, Pr)
            
        h = convection_coefficient(Nu, ambient_fluid.thermal_conductivity, tank.length)
        Q_conv = heat_transfer_convection(h, tank.surface_area, T_surface, T_ambient)
        
        # 4. Calculate conductive resistances through tank layers
        # Outer shell resistance
        R_outer = thermal_resistance_conduction(
            materials["outer_shell"].thickness,
            materials["outer_shell"].thermal_conductivity,
            tank.surface_area
        )
        
        # Mylar resistance
        R_mylar = thermal_resistance_conduction(
            materials["mylar"].thickness,
            materials["mylar"].thermal_conductivity,
            tank.surface_area
        )
        
        # Insulation resistance
        R_insulation = thermal_resistance_conduction(
            insulation_thickness,
            materials["insulation"].thermal_conductivity,
            tank.surface_area
        )
        
        # Inner shell resistance
        R_inner = thermal_resistance_conduction(
            materials["inner_shell"].thickness,
            materials["inner_shell"].thermal_conductivity,
            tank.surface_area
        )
        
        # 5. Calculate lug thermal resistance
        # Note: This is a simplified calculation of the lug resistance
        # TODO: Implement the thermal resistance from figure 4
        R_lug = thermal_resistance_conduction(
            materials["lug"].thickness,
            materials["lug"].thermal_conductivity,
            materials["lug"].thickness * 0.1  # Approximate lug cross-sectional area
        ) / 3  # Three lugs in parallel
        
        # 6. Total heat transfer calculation
        # Main path resistance (through tank walls)
        R_walls = series_resistance(R_outer, R_mylar, R_insulation, R_inner)
        
        # Combine with lug resistance in parallel
        R_total = parallel_resistance(R_walls, R_lug)
        
        # Calculate total heat transfer
        Q_total = Q_solar + Q_rad + Q_conv
        
        return Q_total
    
    if target_boiloff_rate is not None:
        # Convert target boiloff rate to heat transfer rate
        target_Q = target_boiloff_rate * operating_conditions["h_vap"]
        
        def heat_balance(x: np.ndarray) -> np.ndarray:
            thickness, T_surface = x
            Q = calculate_heat_transfer(T_surface, thickness)
            return np.sum(np.array([Q - target_Q, T_surface - T_ambient + Q * 0.01]))**2
            
        # Initial guess
        x0 = np.array([0.1, T_ambient])  # [thickness, T_surface]
        solution = minimize(heat_balance, x0)['x']
        thickness, T_surface = solution
        
        return target_boiloff_rate, thickness
    else:
        thickness = materials["insulation"].thickness
        
        def temp_balance(T_surface: float) -> float:
            Q = calculate_heat_transfer(T_surface, thickness)
            return (T_surface - T_ambient + Q * 0.01)**2
        
        T_surface = minimize(temp_balance, T_ambient)['x'][0]
        Q = calculate_heat_transfer(T_surface, thickness)
        boiloff_rate = Q / operating_conditions["h_vap"]
        return boiloff_rate, thickness

# Example usage:
if __name__ == "__main__":
    # Example air properties at 35,000 ft
    air_cruise = FluidProperties(
        density=0.38035,  # kg/m3
        dynamic_viscosity=1.39e-5,  # kg/s*m
        heat_capacity=1002,  # J/kg*K
        thermal_conductivity=0.0241  # W/m*K
    )
    
    # Example tank geometry
    tank = TankGeometry(
        diameter=1.939,  # m
        length=16.76  # m
    )
    
    # Example materials (from paper's Appendix A)
    materials = {
        "outer_shell": MaterialProperties(
            thermal_conductivity=0.15,  # Carbon epoxy W/m*K
            density=1.6,  # Approximate density
            thickness=0.00127  # m
        ),
        "mylar": MaterialProperties(
            thermal_conductivity=0.14,  # W/m*K
            density=1.4,  # Approximate density
            thickness=25.4e-6  # m
        ),
        "insulation": MaterialProperties(
            thermal_conductivity=0.02594,  # EPS W/m*K
            density=1.6229,  # lbs/ft3
            thickness=0.1  # Initial guess in m
        ),
        "inner_shell": MaterialProperties(
            thermal_conductivity=225.94,  # Aluminum W/m*K
            density=2700,  # kg/m3
            thickness=0.001778  # m
        ),
        "lug": MaterialProperties(
            thermal_conductivity=0.25,  # Nylon W/m*K
            density=1150,  # kg/m3
            thickness=0.0254  # m (1 inch)
        )
    }
    # Case 1: Cruise conditions, LH2 fuel use, full tank, 50% idle fuel usage
    conditions_cruise = {
        "ambient_temp": 218.9,  # K (at 35,000 ft)
        "critical_temp": 28.7,  # K
        "solar_flux": 1367,  # W/m2
        "absorptivity": 0.02,
        "cruise_velocity": 232,  # m/s
        "h_vap": 452.51 - 114.55  # kJ/kg
    }
    
    # Air properties at cruise
    air_cruise = FluidProperties(
        density=0.38035,  # kg/m3
        dynamic_viscosity=1.39e-5,  # kg*s/m
        heat_capacity=1002,  # J/kg*K
        thermal_conductivity=0.0241  # W/m*K
    )
    
    # Tank geometry
    tank = TankGeometry(
        diameter=1.939,  # m
        length=16.76  # m
    )
    
    # Target boiloff rate (50% of idle fuel usage = 173 lbs/hr)
    target_boiloff = 173  # lbs/hr
    
    # Calculate results
    boiloff_rate, thickness = analyze_tank_thermal_performance(
        tank=tank,
        ambient_fluid=air_cruise,
        materials=materials,
        operating_conditions=conditions_cruise,
        target_boiloff_rate=target_boiloff
    )
    
    # Print comparison
    print("\nCruise Conditions Validation:")
    print(f"Paper thickness: 3.97 inches")
    print(f"Calculated thickness: {thickness/0.0254:.2f} inches")
    print(f"Paper insulation weight: 624 lbs")
    #print(f"Tank  Surface Area: {tank.surface_area:.3f} m^2")
    weight = tank.surface_area * thickness * materials["insulation"].density * 3.28084**3  # m^3 to ft^3
    print(f"Calculated weight: {weight:.0f} lbs")