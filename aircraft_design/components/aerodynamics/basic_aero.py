import numpy as np
from typing import Dict, Any
from aircraft_design.core.base import Component, AnalysisModule
from aircraft_design.components.aerodynamics.wing_geometry import AerodynamicGeometry

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
        mach = self.parameters['mach']
        if mach >= 1.0:
            mach = 0.99  # Limit to subsonic
        beta = np.sqrt(1 - mach**2)
        
        # Finite wing correction
        AR = max(geom.aspect_ratio, 0.001)  # Prevent division by zero
        e = 0.9  # Oswald efficiency factor (simplified)
        cl_alpha = self.parameters['cl_alpha']
        
        # Lift curve slope correction for sweep and AR
        denominator = np.sqrt(1 + (cl_alpha * np.cos(sweep_rad) / (np.pi * AR * e))**2)
        if denominator < 0.001:
            denominator = 0.001  # Prevent division by very small numbers
            
        cl_alpha_wing = (cl_alpha * np.cos(sweep_rad)) / denominator
        
        # Calculate CL
        cl = cl_alpha_wing * alpha_rad / beta
        
        return {
            'CL': float(cl),  # Convert to Python float to avoid numpy warnings
            'CL_alpha': float(cl_alpha_wing),
            'aspect_ratio': float(AR),
            'effective_sweep': float(np.degrees(sweep_rad)),
            'mach_correction': float(1/beta) if beta > 0.001 else 1000.0  # Limit maximum correction
        }

class ParasiticDragAnalysis(AnalysisModule):
    """Calculate parasitic drag coefficient using flat plate skin friction methods"""
    def __init__(self):
        super().__init__('parasitic_drag')
        self.parameters.update({
            'mach': 0.0,                    # Mach number
            'reynolds': 0.0,                # Reynolds number
            'k_surface': 2.08e-5,           # Surface roughness (ft)
            'reference_area': 1.0,          # Reference area for coefficients
            'wetted_area': None,            # Component wetted area
            'characteristic_length': None,   # Component characteristic length
            'form_factor': 1.0              # Component form factor
        })
        self.required_parameters = ['mach', 'wetted_area', 'characteristic_length']

    def _calculate_turbulent_cf(self, reynolds: float, mach: float) -> float:
        """Calculate turbulent skin friction coefficient using Raymer's method"""
        # Calculate cutoff Reynolds number (Raymer eq. 12.27)
        k = self.parameters['k_surface']
        L = self.parameters['characteristic_length']
        R_cutoff = 38.21 * (L/k)**1.053

        # Use minimum of actual and cutoff Reynolds numbers
        R = min(reynolds, R_cutoff)
        
        # Calculate turbulent skin friction (Raymer eq. 12.26)
        cf = 0.455 / (np.log10(R)**2.58 * (1 + 0.144*mach**2)**0.65)
        return cf

    def run(self, component: Component) -> Dict[str, Any]:
        """Calculate parasitic drag coefficient"""
        mach = self.parameters['mach']
        wetted_area = self.parameters['wetted_area']
        ref_area = self.parameters['reference_area']
        form_factor = self.parameters['form_factor']
        
        if not wetted_area:
            raise ValueError("Wetted area must be specified")
            
        # Calculate Reynolds number if not provided
        if not self.parameters['reynolds']:
            if not self.parameters['characteristic_length']:
                raise ValueError("Either Reynolds number or characteristic length must be specified")
            # TODO: Add proper Reynolds calculation based on flight conditions
            self.parameters['reynolds'] = 1e7  # Default value
            
        reynolds = self.parameters['reynolds']
        
        # Calculate skin friction coefficient
        cf = self._calculate_turbulent_cf(reynolds, mach)
        
        # Calculate component CD0 (Raymer eq. 12.23)
        cd0 = cf * form_factor * wetted_area / ref_area
        
        return {
            'CD0': float(cd0),
            'Cf': float(cf),
            'wetted_area': float(wetted_area),
            'reference_area': float(ref_area),
            'form_factor': float(form_factor),
            'reynolds': float(reynolds)
        }

class AerodynamicComponent(Component):
    """Base class for aerodynamic components like wings, tails, etc."""
    def __init__(self, name: str):
        super().__init__(name)
        self.geometry = AerodynamicGeometry()
        
        # Add default analysis modules
        self.add_analysis(BasicLiftAnalysis())
        self.add_analysis(ParasiticDragAnalysis())
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
