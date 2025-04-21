from typing import List, Dict, Any
from aircraft_design.core.base import Component, Position, Geometry
from aircraft_design.analysis.mass_analysis import MassFeature, MassAnalysis
from aircraft_design.components.aerodynamics.wing_geometry import WaypointWingGeometry
from aircraft_design.components.aerodynamics.basic_aero import AerodynamicComponent
from aircraft_design.core.plotting import plot_orthographic_views, create_box, Object3D
from aircraft_design.components.propulsion.fuel_tanks import FuelTank, FuelTankGeometry

import matplotlib.pyplot as plt
import numpy as np

class Wing(Component):
    """Main wing component with integrated fuel tanks"""
    
    def __init__(self, 
                wing_tip_position: float,
                 fuel_configuration: str = "full"):
        """
        Initialize the main wing with integrated fuel tanks
        
        Args:
            nose_length: Length of the nose section in feet
            tall_fuselage_length: Length of the tall fuselage section in feet
            fuel_configuration: Fuel configuration ("full", "empty", or "half")
        """
        super().__init__(name="main_wing")
        
        # Store fuel configuration
        self.fuel_configuration = fuel_configuration
        
        # Create aerodynamic component for the wing
        wing_geom = WaypointWingGeometry()
        
        # Set wing parameters
        wing_geom.parameters.update({
            'span': 315.0,  # Wing span in feet
            'le_sweep': 40.0,  # Leading edge sweep in degrees
            'dihedral': 5.0,  # Dihedral angle in degrees
            'root_chord': 70.0,  # Root chord in feet
            'tip_chord': 7.0,  # Tip chord in feet
        })
        
        # Wing geometry parameters
        root_chord = 70
        tip_chord = 7
        thickness_ratio = 0.14
        
        # Add waypoints from root to tip
        wing_geom.add_waypoint(0.0, root_chord, thickness_ratio * root_chord)    # Root
        wing_geom.add_waypoint(0.3, root_chord*.52, thickness_ratio * root_chord*.55)    # Root
        wing_geom.add_waypoint(1.0, tip_chord, thickness_ratio * tip_chord)    # Tip
        
        # Set wing position
        wing_position = Position(
            x=wing_tip_position,  # Position wing in the middle of the tall fuselage section
            y=0,
            z=0.14 * root_chord  # Align with the middle of the fuselage
        )
        wing_geom.position = wing_position
        self.geometry = wing_geom
        
        # Store properties for child positioning
        self.wing_tip_position = wing_tip_position
        self.root_chord = root_chord
        
        # Add fuel tanks as child components
        self._add_fuel_tanks()

        # Add control surfaces
        self._add_control_surfaces()

        # Add mass
        self._populate_mass_analysis()

    def _add_fuel_tanks(self):
        """Add a single fuel tank to the wing"""
        
        # Wing position in feet
        wing_x = self.wing_tip_position
        wing_z = 0.14 * self.root_chord
        
        # Create a single large fuel tank
        tank = FuelTank(
            length=0,  # 80% of root chord
            front_height=0,  # 12% of root chord
            back_height=0,  # 12% of root chord
            width=0,  # 80% of wing span
            fill_level=1
        )
        tank.color = "red"
        tank.geometry = FuelTankGeometry()
        tank.name = "main_fuel_tank"
        
        # Position the tank at the wing's center
        tank.geometry.position = Position(
            x=0,  # Centered along chord
            y=0,  # Centered along span
            z=0   # At wing's vertical center
        )
        
        # Set mass properties manually
        tank_mass = 6e5 -5.4e4 # lbs
        tank_cg_x = 231.05 / 12  # Convert inches to feet
        tank_cg_y = 0.0
        tank_cg_z = 36.04 / 12  # Convert inches to feet
        

        tank_ixx = 375871650626.54 / 144 #zz
        tank_iyy = 79516660699.40 / 144  #xx
        tank_izz = 450089349039.73 / 144 #yy
        tank_ixy = -10319345.66 / 144   #zx
        tank_ixz = -310505932.55 / 144   #zy
        tank_iyz = -18567.36 / 144       #yx

        tank.features[0].parameters.update({
            "mass": tank_mass,
            "center_of_gravity": [tank_cg_x, tank_cg_y, tank_cg_z],
            "ixx": tank_ixx,
            "iyy": tank_iyy,
            "izz": tank_izz,
            "ixy": tank_ixy,
            "ixz": tank_ixz,
            "iyz": tank_iyz
        })
        
        self.add_child(tank)

    def _add_control_surfaces(self):
        """Add flaps and ailerons to the wing"""
        # Flap parameters
        self.flap_start = 0.13  # Start at 10% of half span
        self.flap_end = self.flap_start + .6   # End at 60% of half span
        self.flap_chord_ratio = 0.3  # Flap is 30% of local chord
        
        # Aileron parameters
        self.aileron_start = self.flap_end +.005  # Start at 60% of half span
        self.aileron_end = 0.95    # End at 90% of half span
        self.aileron_chord_ratio = 0.25  # Aileron is 25% of local chord
        
        # Calculate control surface areas
        self._calculate_control_surface_areas()

    def _calculate_control_surface_areas(self):
        """Calculate the areas of flaps and ailerons"""
        # Get wing parameters
        span = self.geometry.parameters['span']
        root_chord = self.root_chord
        tip_chord = self.geometry.parameters['tip_chord']
        
        # Calculate half span positions
        half_span = span / 2
        flap_start_y = self.flap_start * half_span
        flap_end_y = self.flap_end * half_span
        aileron_start_y = self.aileron_start * half_span
        aileron_end_y = self.aileron_end * half_span
        #print(f"flap_start_y: {flap_start_y}, flap_end_y: {flap_end_y}, aileron_start_y: {aileron_start_y}, aileron_end_y: {aileron_end_y}")
        
        # Calculate chord lengths at control surface boundaries
        def chord_at_y(y):
            return root_chord * (1 - y/half_span) + tip_chord * (y/half_span)
        
        # Calculate flap area (trapezoidal integration)
        flap_chord_start = chord_at_y(flap_start_y)
        flap_chord_end = chord_at_y(flap_end_y)
        flap_span = flap_end_y - flap_start_y
        flap_area = (flap_chord_start + flap_chord_end) * flap_span * self.flap_chord_ratio
        
        # Calculate aileron area (trapezoidal integration)
        aileron_chord_start = chord_at_y(aileron_start_y)
        aileron_chord_end = chord_at_y(aileron_end_y)
        aileron_span = aileron_end_y - aileron_start_y
        aileron_area = (aileron_chord_start + aileron_chord_end) * aileron_span * self.aileron_chord_ratio
        
        # Store areas (multiply by 2 for both sides)
        self.flap_area = flap_area
        self.aileron_area = aileron_area
        
        # Calculate total control surface area
        self.total_control_surface_area = self.flap_area + self.aileron_area



    def _populate_mass_analysis(self):
        # Values from solidworks: 3/30/2025
        # mass_lb = 280000.86
        # cg_x_in = 822.18
        # cg_y_in = 0.0  # Setting to 0.0 to ensure symmetry about the y-axis
        # cg_z_in = 0.01  # This was 47.06 but appears to have been swapped with y

        # Fix the order of moments of inertia to match coordinate system
        # ixx_lb_in2 = 22241208042.76
        # iyy_lb_in2 = 25786865454.86  # This was incorrectly assigned to izz
        # izz_lb_in2 = 3654685797.90   # This was incorrectly assigned to iyy
        # ixy_lb_in2 = 0.0  # Setting to 0.0 for symmetric mass distribution
        # ixz_lb_in2 = 106736.46
        # iyz_lb_in2 = 0.0  # Setting to 0.0 for symmetric mass distribution

        # Create the MassFeature
        # wing_box_mass_feature = MassFeature(
        #     mass=mass_lb * 1.1,
        #     center_of_gravity=[cg_x_in / 12, cg_y_in / 12, cg_z_in / 12],
        #     ixx=ixx_lb_in2 / 144,
        #     iyy=iyy_lb_in2 / 144,
        #     izz=izz_lb_in2 / 144,
        #     ixy=ixy_lb_in2 / 144,
        #     ixz=ixz_lb_in2 / 144,
        #     iyz=iyz_lb_in2 / 144
        # )

        # self.add_feature(wing_box_mass_feature)
        self.add_analysis(MassAnalysis())

    @property
    def aspect_ratio(self) -> float:
        """Get the wing's aspect ratio"""
        return self.geometry.aspect_ratio
    
    def plot(self, *args, **kwargs) -> Object3D:
        """Create a 3D visualization of the wing"""
        # Create colors dictionary if not provided
        colors_dict = kwargs.get('colors_dict', {})
        
        # Add colors for fuel tanks if not already specified
        for child in self.children:
            if isinstance(child, FuelTank) and child.name not in colors_dict:
                colors_dict[child.name] = child.color
        
        # Update kwargs with our modified colors_dict
        kwargs['colors_dict'] = colors_dict
        
        # Get the wing's visualization using parent's plot method
        return super().plot(*args, **kwargs)

    def plot_section(self, *args, **kwargs) -> Object3D:
        """Create a 3D visualization of the wing section"""
        # Get the wing section's aerodynamic visualization
        section_obj = self.plot()
        section_obj.metadata['color'] = 'gray'  # Set section color
        
        return section_obj

    @property
    def wing_area(self) -> float:
        """Get the wing's planform area in square feet"""
        return self.geometry.area
    
    @property
    def wing_span(self) -> float:
        """Get the wing's span in feet"""
        return self.geometry.parameters['span']
    
    @property
    def mean_aerodynamic_chord(self) -> float:
        """Get the wing's mean aerodynamic chord in feet"""
        # For a linearly tapered wing
        return (2/3) * self.root_chord * (1 + self.geometry.waypoints[-1].chord_length/self.root_chord + 
                                         (self.geometry.waypoints[-1].chord_length/self.root_chord)**2) / (
                                         1 + self.geometry.waypoints[-1].chord_length/self.root_chord)
    
    @property
    def mass(self) -> float:
        """Get the wing's mass in pounds"""
        return self.features[0].parameters["mass"]

if __name__ == "__main__":
    # Create a wing instance
    wing = Wing(wing_tip_position=75)
    
    # Print fuel tank information

    total_fuel_volume = 0.0
    #total_fuel_mass = 0.0
    print("\n=== Fuel Tank Information ===")
    for tank in wing.children:
        if isinstance(tank, FuelTank):
            total_fuel_volume += tank.volume
            #total_fuel_mass += tank.fuel_mass
    print(f"Total Fuel Volume: {total_fuel_volume:.0f} ft³")
    print(f"Total Fuel Mass: {total_fuel_volume * tank.fuel_density:.0f} lbs")
        
    # Print control surface information
    print("\n=== Control Surface Information ===")
    print(f"Flap Start: {wing.flap_start * wing.wing_span/2:.3f}")
    print(f"Flap End: {wing.flap_end * wing.wing_span/2:.3f}")
    print(f"Flap Chord Ratio: {wing.flap_chord_ratio:.3f}")
    print(f"Aileron Start: {wing.aileron_start * wing.wing_span/2:.3f}")
    print(f"Aileron End: {wing.aileron_end * wing.wing_span/2:.3f}")
    print(f"Aileron Chord Ratio: {wing.aileron_chord_ratio:.3f}")
    print(f"Flap Area: {wing.flap_area:.1f} ft²")
    print(f"Aileron Area: {wing.aileron_area:.1f} ft²")
    print(f"Total Control Surface Area: {wing.total_control_surface_area:.1f} ft²")
    print(f"Control Surface Area Ratio: {wing.total_control_surface_area/wing.wing_area:.3f}")
    
    # run the mass analysis
    wing.run_analysis(analysis_names="mass_analysis")

    results = wing.analysis_results['mass_analysis']

    print(f"\n=== Wing Properties ===")
    print(f"Wing Area: {wing.wing_area:.1f} ft²")
    print(f"Wing Span: {wing.wing_span:.1f} ft")
    print(f"Mean Aerodynamic Chord: {wing.mean_aerodynamic_chord:.1f} ft")
    print(f"Aspect Ratio: {wing.aspect_ratio:.3f}")
    print(f"Total Mass: {results['total_mass']:.1f} lbs")
    print(f"CG Position: ({results['cg_x']:.2f}, {results['cg_y']:.2f}, {results['cg_z']:.2f}) ft")
    print(f"Moments of Inertia:")
    print(f"  Ixx: {results['total_ixx']:.0f} lbs-ft²")
    print(f"  Iyy: {results['total_iyy']:.0f} lbs-ft²")
    print(f"  Izz: {results['total_izz']:.0f} lbs-ft²")
    """
    # print the total fuel volume and mass
    total_fuel_volume = 0.0
    total_fuel_mass = 0.0
    for tank in wing.children:
        if isinstance(tank, FuelTank):
            total_fuel_volume += tank.volume
            total_fuel_mass += tank.fuel_mass
    print(f"Total Fuel Volume: {total_fuel_volume:.0f} ft³")
    print(f"Total Fuel Mass: {total_fuel_mass:.0f} lbs")
    """

    # Make one tank empty for visualization
    #wing.children[0].set_empty(True)
    
    # Create the wing object for plotting (includes fuel tanks)
    wing_obj = wing.plot(plot_children=True)
    
    # Create figure and plot orthographic views
    fig = plt.figure(figsize=(15, 10))
    fig, (ax_top, ax_side, ax_front) = plot_orthographic_views(wing_obj, fig=fig)
    
    # Add title and adjust layout
    plt.suptitle("Wing with Fuel Tanks and Control Surfaces", fontsize=16)
    plt.tight_layout()
    
    # Save the plot
    plt.savefig("assets/wing_orthographic_views.png")
    
    # Close the plot
    plt.close()

    # Test tank visualization
    print("\n=== Testing Single Tank Visualization ===")
    test_tank = FuelTank(
        length=30.0,
        front_height=3.0,
        back_height=2.0,
        width=40.0,
        fill_level=0.5
    )