import numpy as np
from typing import Dict, Any
from aircraft_design.core.base import Component, AnalysisModule
from wing_geometry import AerodynamicGeometry

class BasicLiftAnalysis(AnalysisModule):
    """Simple lift analysis for aerodynamic surfaces"""
    def __init__(self):
        super().__init__('basic_lift')
        self.parameters = {
            'alpha': 0.0,          # angle of attack in degrees
            'mach': 0.0,           # Mach number
            'reynolds': 1e6,       # Reynolds number
            'cl_alpha': 2*3.14159, # lift curve slope (default = 2pi)
        }
        self.required_parameters = ['alpha', 'mach']

    def run(self, component: Component) -> Dict[str, Any]:
        """Calculate basic lift characteristics"""
        if not isinstance(component.geometry, AerodynamicGeometry):
            raise ValueError("Component must have AerodynamicGeometry for lift analysis")

        geom = component.geometry
        
        # Basic lift calculation
        alpha_rad = np.radians(self.parameters['alpha'])
        sweep_rad = np.radians(geom.parameters['sweep'])
        
        # Prandtl-Glauert compressibility correction
        beta = np.sqrt(1 - self.parameters['mach']**2)
        
        # Finite wing correction
        AR = geom.aspect_ratio
        e = 0.9  # Oswald efficiency factor (simplified)
        cl_alpha = self.parameters['cl_alpha']
        
        # Lift curve slope correction for sweep and AR
        cl_alpha_wing = (cl_alpha * np.cos(sweep_rad)) / (
            np.sqrt(1 + (cl_alpha * np.cos(sweep_rad) / (np.pi * AR * e))**2)
        )
        
        # Calculate CL
        cl = cl_alpha_wing * alpha_rad / beta
        
        return {
            'CL': cl,
            'CL_alpha': cl_alpha_wing,
            'aspect_ratio': AR,
            'effective_sweep': np.degrees(sweep_rad),
            'mach_correction': 1/beta if beta > 0 else float('inf')
        }

class AerodynamicComponent(Component):
    """Base class for aerodynamic components like wings, tails, etc."""
    def __init__(self, name: str):
        super().__init__(name)
        self.geometry = AerodynamicGeometry()
        
        # Add default analysis modules
        self.add_analysis(BasicLiftAnalysis())

    def set_basic_geometry(self, span: float, root_chord: float, tip_chord: float, 
                         sweep: float = 0.0, dihedral: float = 0.0, twist: float = 0.0):
        """Convenience method to set basic geometric parameters"""
        self.geometry.parameters.update({
            'span': span,
            'root_chord': root_chord,
            'tip_chord': tip_chord,
            'sweep': sweep,
            'dihedral': dihedral,
            'twist': twist
        })

# Example usage
if __name__ == "__main__":
    # Create a simple wing
    wing = AerodynamicComponent("main_wing")
    wing.set_basic_geometry(
        span=30.0,          # 30m wingspan
        root_chord=5.0,     # 5m root chord
        tip_chord=2.0,      # 2m tip chord
        sweep=25.0,         # 25 degrees sweep
        dihedral=5.0,       # 5 degrees dihedral
        twist=2.0           # 2 degrees washout
    )
    
    # Set analysis conditions
    lift_analysis = wing.analyses['basic_lift']
    lift_analysis.parameters.update({
        'alpha': 5.0,       # 5 degrees angle of attack
        'mach': 0.3,        # Mach 0.3
    })
    
    # Run analysis
    results = wing.run_analysis('basic_lift')
    print("\nWing Analysis Results:")
    for key, value in results.items():
        print(f"{key}: {value}")
