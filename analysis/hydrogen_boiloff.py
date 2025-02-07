import numpy as np
from dataclasses import dataclass
from typing import Dict, Tuple

@dataclass
class FluidProperties:
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
    operating_conditions: dict
) -> Tuple[float, float]:
    """
    Analyze thermal performance of tank and calculate required insulation thickness
    
    Returns:
        Tuple of (boiloff_rate, required_insulation_thickness)
    """
    # Implementation of the full thermal analysis would go here
    # This would use all the above functions to calculate the various
    # heat transfer components and determine insulation requirements
    
    # Placeholder return
    return 0.0, 0.0

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
    
    # Example materials
    materials = {
        "EPS": MaterialProperties(
            thermal_conductivity=0.02594,  # W/m*K
            density=1.6229  # lbs/ft3
        ),
        "MLI": MaterialProperties(
            thermal_conductivity=0.000135,  # W/m*K
            density=10.8  # lbs/ft3
        ),
        "Glass_Bubbles": MaterialProperties(
            thermal_conductivity=0.130,  # W/m*K
            density=14.36  # lbs/ft3
        )
    }
    
    # Operating conditions
    conditions = {
        "ambient_temp": 218.9,  # K
        "critical_temp": 28.7,  # K
        "solar_flux": 1367,  # W/m2
        "absorptivity": 0.02,
        "cruise_velocity": 232  # m/s
    }
    
    # Calculate thermal performance
    boiloff_rate, insulation_thickness = analyze_tank_thermal_performance(
        tank, air_cruise, materials, conditions
    )