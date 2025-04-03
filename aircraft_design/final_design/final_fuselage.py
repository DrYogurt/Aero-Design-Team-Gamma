from typing import List, Dict, Any
import numpy as np
import matplotlib.pyplot as plt

from aircraft_design.core.base import Component, Position
from aircraft_design.analysis.mass_analysis import MassFeature, MassAnalysis
from aircraft_design.components.fuselage.fuselage_geometry import FuselageGeometry, CrossSection, CrossSectionShape
from aircraft_design.components.aerodynamics.basic_aero import AerodynamicComponent
from aircraft_design.core.plotting import Object3D, plot_orthographic_views, plot_cross_section

class Fuselage(Component):
    """Main fuselage component that defines the aircraft body"""
    
    def __init__(self):
        """Initialize the fuselage component with its geometry"""
        super().__init__(name="fuselage")
        
        # Create aerodynamic component for the fuselage
        self.aero = AerodynamicComponent("fuselage")
        
        # Fuselage dimensions in feet
        self.tall_fuselage_length = 50
        self.transition_length = 70
        self.short_fuselage_length = self.tall_fuselage_length + self.transition_length + 60
        
        self.tall_fuselage_height = 35
        self.short_fuselage_height = 27
        
        self.fuselage_width = 27
        
        self.nose_length = 30
        self.tail_length = 62
        self.tail_height = 18
        
        # Create the fuselage geometry
        self._create_fuselage_geometry()
        
        # Add mass analysis
        self._populate_mass_analysis()
    
    def _create_fuselage_geometry(self):
        """Create the fuselage geometry with waypoints"""
        fuselage_geom = FuselageGeometry()
        
        waypoints = [ #x, width, height, z_offset
            (0, 5, 5, self.tall_fuselage_height/2), # nose tip
            (self.nose_length, self.fuselage_width, self.tall_fuselage_height, self.tall_fuselage_height/2), # Nose-> tall fuselage
            (self.nose_length + self.tall_fuselage_length, self.fuselage_width, self.tall_fuselage_height, self.tall_fuselage_height/2), # tall fuselage start
            (self.nose_length + self.short_fuselage_length, self.fuselage_width, self.short_fuselage_height, self.short_fuselage_height/2), # short fuselage start
            (self.nose_length + self.tall_fuselage_length + self.transition_length, self.fuselage_width, self.short_fuselage_height, self.short_fuselage_height/2), # short fuselage start
            (self.nose_length + self.short_fuselage_length + self.tail_length, self.fuselage_width/4, self.tail_height, self.short_fuselage_height - self.tail_height/2), # short fuselage end
        ]
        
        for x, width, height, z_offset in waypoints:
            section = CrossSection(
                station=x,
                width=width,
                height=height,
                shape=CrossSectionShape.SUPER_ELLIPSE,
                z_offset=z_offset,
                parameters={'n': 2.5}  # Super-ellipse parameter for smooth blending
            )
            fuselage_geom.add_section(section)
        
        self.aero.geometry = fuselage_geom
    
    def _populate_mass_analysis(self):
        """Add mass analysis with dummy values (similar to final_wing.py)"""
        # Dummy values for fuselage mass analysis
        mass_lb = 268094.78
        cg_x_in = 1349.02
        cg_y_in = -.27
        cg_z_in = 30.05

        ixx_lb_in2 = 4768008236.83 
        ixy_lb_in2 = 57687872.85
        ixz_lb_in2 = -8615782645.12
        ixy_lb_in2 = 57687872.85
        iyy_lb_in2 = 652355028347.41
        iyz_lb_in2 = 9982748.14
        ixz_lb_in2 = -8615782645.12
        iyz_lb_in2 = 9982748.14
        izz_lb_in2 = 652031439595.14

        
        # Create the MassFeature (converting inches to feet)
        fuselage_mass_feature = MassFeature(
            mass=mass_lb,
            center_of_gravity=[cg_x_in / 12, cg_y_in / 12, cg_z_in / 12],
            ixx=ixx_lb_in2 / 144,
            iyy=iyy_lb_in2 / 144,
            izz=izz_lb_in2 / 144,
            ixy=ixy_lb_in2 / 144,
            ixz=ixz_lb_in2 / 144,
            iyz=iyz_lb_in2 / 144
        )

        self.add_feature(fuselage_mass_feature)
        self.add_analysis(MassAnalysis())
    
    def plot(self, *args, **kwargs) -> Object3D:
        """Create a 3D visualization of the fuselage"""
        # Get the fuselage's aerodynamic visualization
        fuselage_obj = self.aero.plot()
        fuselage_obj.metadata['color'] = 'blue'  # Set fuselage color
        
        return fuselage_obj

if __name__ == "__main__":
    # Create a fuselage instance
    fuselage = Fuselage()
    

    fuselage.run_analysis(analysis_names="mass_analysis")

    results = fuselage.analysis_results['mass_analysis']

    print(f"\n=== Fuselage Properties ===")
    print(f"Total Mass: {results['total_mass']:.1f} lbs")
    print(f"CG Position: ({results['cg_x']:.2f}, {results['cg_y']:.2f}, {results['cg_z']:.2f}) ft")
    print(f"Moments of Inertia:")
    print(f"  Ixx: {results['total_ixx']:.0f} lbs-ft²")
    print(f"  Iyy: {results['total_iyy']:.0f} lbs-ft²")
    print(f"  Izz: {results['total_izz']:.0f} lbs-ft²")

    # Create the fuselage object for plotting
    fuselage_obj = fuselage.plot()
    
    # Create figure and plot orthographic views
    fig = plt.figure(figsize=(15, 10))
    fig, (ax_top, ax_side, ax_front) = plot_orthographic_views(fuselage_obj, fig=fig)
    
    # Add title and adjust layout
    plt.suptitle("Aircraft Fuselage", fontsize=16)
    plt.tight_layout()
    
    # Save the plot
    plt.savefig("assets/fuselage_orthographic_views.png")
    
    # Close the plot
    plt.close()
