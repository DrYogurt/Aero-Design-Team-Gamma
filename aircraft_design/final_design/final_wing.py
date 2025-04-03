from typing import List, Dict, Any
from aircraft_design.core.base import Component, Position, Geometry
from aircraft_design.analysis.mass_analysis import MassFeature, MassAnalysis
from aircraft_design.components.aerodynamics.wing_geometry import WaypointWingGeometry
from aircraft_design.components.aerodynamics.basic_aero import AerodynamicComponent
from aircraft_design.core.plotting import plot_orthographic_views, create_box, Object3D

import matplotlib.pyplot as plt
import numpy as np

class FuelTankGeometry(Geometry):
    """Geometry for a fuel tank"""
    def __init__(self):
        super().__init__()
        self.parameters.update({
            'length': 0.0,
            'width': 0.0,
            'front_height': 0.0,
            'back_height': 0.0,
            'fuel_density': 50.0,
            'empty': False
        })
    def create_object(self) -> Object3D:
        """Create a 3D object representation of this geometry"""
        obj = Object3D()
        
        # Create a box shape for the tank using the average height
        avg_height = (self.parameters['front_height'] + self.parameters['back_height']) / 2
        tank_shape = create_box(
            width=self.parameters['width'],
            length=self.parameters['length'],
            height=avg_height
        )
        
        obj.add_shape(tank_shape)
        return obj
    
    def validate(self) -> bool:
        """Validate the geometry parameters"""
        return all(v >= 0 for v in [
            self.parameters['length'],
            self.parameters['width'],
            self.parameters['front_height'],
            self.parameters['back_height']
        ])
        
    

class FuelTank(Component):
    """A fuel tank component with trapezoidal volume and mass analysis"""
    
    def __init__(self, 
                 length: float,
                 front_height: float,
                 back_height: float,
                 width: float,
                 fuel_density: float = 50.0,  # lb/ft^3, typical for aviation fuel
                 empty: bool = False):
        """
        Initialize a fuel tank with trapezoidal volume
        
        Args:
            length: Length of the tank in feet
            front_height: Height at front of tank in feet
            back_height: Height at back of tank in feet
            width: Width of the tank in feet
            fuel_density: Density of the fuel in lb/ft^3
            empty: Whether the tank is empty
        """
        super().__init__(name="fuel_tank")
        
        # Store basic parameters
        self.length = length
        self.front_height = front_height
        self.back_height = back_height
        self.width = width
        self.fuel_density = fuel_density
        self.empty = empty
        
        # Calculate volume (trapezoidal prism)
        self.volume = (front_height + back_height) / 2 * length * width
        
        # Calculate mass based on volume and density
        self.fuel_mass = 0.0 if empty else self.volume * fuel_density
        
        # Calculate center of gravity (assuming uniform density)
        # For a trapezoidal prism, CG is at the centroid
        cg_x = length / 2
        cg_y = width / 2
        cg_z = (front_height + back_height) / 3  # Height of centroid for trapezoid
        
        # Add mass feature
        self.add_feature(MassFeature(
            mass=self.fuel_mass,
            center_of_gravity=[cg_x, cg_y, cg_z],
            ixx=self._calculate_ixx(),
            iyy=self._calculate_iyy(),
            izz=self._calculate_izz()
        ))
        
        # Add mass analysis
        self.add_analysis(MassAnalysis())
        
        # Create and configure geometry
        self.geometry = FuelTankGeometry()
        self.geometry.parameters.update({
            'length': length,
            'width': width,
            'front_height': front_height,
            'back_height': back_height,
            'fuel_density': fuel_density,
            'empty': empty
        })
    
    def _calculate_ixx(self) -> float:
        """Calculate moment of inertia about x-axis"""
        # Simplified calculation for rectangular prism approximation
        return self.fuel_mass * (self.width**2 + self.front_height**2) / 12
    
    def _calculate_iyy(self) -> float:
        """Calculate moment of inertia about y-axis"""
        # Simplified calculation for rectangular prism approximation
        return self.fuel_mass * (self.length**2 + self.front_height**2) / 12
    
    def _calculate_izz(self) -> float:
        """Calculate moment of inertia about z-axis"""
        # Simplified calculation for rectangular prism approximation
        return self.fuel_mass * (self.length**2 + self.width**2) / 12
    
    def set_empty(self, empty: bool) -> None:
        """Set whether the tank is empty and update mass accordingly"""
        self.empty = empty
        self.fuel_mass = 0.0 if empty else self.volume * self.fuel_density
        
        # Update mass feature
        for feature in self.features:
            if isinstance(feature, MassFeature):
                feature.parameters["mass"] = self.fuel_mass
                break
    
    def plot(self) -> Object3D:
        """Create a 3D visualization of the fuel tank"""
        tank_obj = self.geometry.create_object()
        
        # Set color based on empty state
        tank_obj.metadata['color'] = 'red' if self.empty else 'green'
        
        # Get the global position of the component
        global_pos = self.get_global_position()
        
        # Apply the global position to all shapes in the tank object
        for shape in tank_obj.shapes:
            shape.vertices = shape.vertices.astype(np.float64)
            shape.vertices += global_pos
        
        return tank_obj

class Wing(AerodynamicComponent):
    """Main wing component with integrated fuel tanks"""
    
    def __init__(self, 
                 nose_length: float = 20.0,
                 tall_fuselage_length: float = 50.0,
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
        })
        
        # Wing geometry parameters
        root_chord = 49.53
        tip_chord = 9.91
        thickness_ratio = 0.14
        
        # Add waypoints from root to tip
        wing_geom.add_waypoint(0.0, root_chord, thickness_ratio * root_chord)    # Root
        wing_geom.add_waypoint(1.0, tip_chord, thickness_ratio * tip_chord)    # Tip
        
        # Set wing position
        wing_position = Position(
            x=112,  # Position wing in the middle of the tall fuselage section
            y=0,
            z=0.14 * root_chord  # Align with the middle of the fuselage
        )
        wing_geom.position = wing_position
        self.geometry = wing_geom
        
        # Store properties for child positioning
        self.nose_length = nose_length
        self.tall_fuselage_length = tall_fuselage_length
        self.root_chord = root_chord
        
        # Add fuel tanks as child components
        self._add_fuel_tanks()

        # Add mass
        self._populate_mass_analysis()

    
    def _add_fuel_tanks(self):
        """Add four fuel tanks to the wing"""
        # Tank dimensions in feet
        tank_length = 15.0  # Reduced from 20.0
        tank_width = 30.0   # Reduced from 50.0
        tank_front_height = 3.0  # Reduced from 5.0
        tank_back_height = 2.0   # Reduced from 3.0
        
        # Wing position in feet
        wing_x = self.nose_length + self.tall_fuselage_length
        wing_z = 0.14 * self.root_chord
        
        # Determine tank states based on configuration
        is_empty = {
            "full": False,
            "empty": True,
            "half": False  # We'll handle half differently
        }[self.fuel_configuration]

        tank_count = 0
        for side in ['left', 'right']:
            for position in ['front', 'back']:
                tank = FuelTank(
                    length=tank_length,
                    front_height=tank_front_height,
                    back_height=tank_back_height,
                    width=tank_width,
                    empty=(is_empty or (self.fuel_configuration == "half" and tank_count >= 2))
                )
                tank_count += 1
                tank.geometry = FuelTankGeometry()
                # Set tank name for identification
                tank.name = f"fuel_tank_{side}_{position}"
                
                # Position along wing span (y-axis)
                y_offset = 30.0 if side == 'left' else -30.0  # Reduced from 50.0
                
                # Position along wing chord (x-axis)
                # Position tanks at 25% and 60% of local chord
                local_chord = self.root_chord  # Simplified: use root chord for all tanks
                x_offset = 0.25 * local_chord if position == 'front' else 0.60 * local_chord
                
                # Adjust x position for sweep
                sweep_rad = np.radians(40.0)  # wing sweep in radians
                sweep_offset = abs(y_offset) * np.tan(sweep_rad)
                x_with_sweep = x_offset + sweep_offset
                
                # Z-offset (height) - position tanks inside the wing
                z_offset = wing_z - 0.5  # Reduced from 1.0 to bring tanks closer to wing center
                
                # Set position directly in feet
                tank.geometry.position = Position(
                    x=wing_x + x_with_sweep,  # Add wing x position plus chord position
                    y=y_offset,               # Y position along span
                    z=z_offset                # Z position (height)
                )
                
                self.add_child(tank)
    
    def set_fuel_configuration(self, config: str):
        """
        Set the fuel configuration for all tanks
        
        Args:
            config: One of "full", "empty", or "half"
        """
        self.fuel_configuration = config
        tank_count = 0
        for child in self.children:
            if isinstance(child, FuelTank):
                is_empty = (config == "empty" or (config == "half" and tank_count >= 2))
                child.set_empty(is_empty)
                tank_count += 1

    def _populate_mass_analysis(self):
        # Values from solidworks: 3/30/2025
        mass_lb = 28087.86
        cg_x_in = 822.18
        cg_y_in = 0.0  # Setting to 0.0 to ensure symmetry about the y-axis
        cg_z_in = 0.01  # This was 47.06 but appears to have been swapped with y

        # Fix the order of moments of inertia to match coordinate system
        ixx_lb_in2 = 22241208042.76
        iyy_lb_in2 = 25786865454.86  # This was incorrectly assigned to izz
        izz_lb_in2 = 3654685797.90   # This was incorrectly assigned to iyy
        ixy_lb_in2 = 0.0  # Setting to 0.0 for symmetric mass distribution
        ixz_lb_in2 = 106736.46
        iyz_lb_in2 = 0.0  # Setting to 0.0 for symmetric mass distribution

        # Create the MassFeature
        wing_box_mass_feature = MassFeature(
            mass=mass_lb,
            center_of_gravity=[cg_x_in / 12, cg_y_in / 12, cg_z_in / 12],
            ixx=ixx_lb_in2 / 144,
            iyy=iyy_lb_in2 / 144,
            izz=izz_lb_in2 / 144,
            ixy=ixy_lb_in2 / 144,
            ixz=ixz_lb_in2 / 144,
            iyz=iyz_lb_in2 / 144
        )

        self.add_feature(wing_box_mass_feature)
        self.add_analysis(MassAnalysis())

    @property
    def aspect_ratio(self) -> float:
        """Get the wing's aspect ratio"""
        return self.geometry.aspect_ratio
    
    def plot(self, *args, **kwargs) -> Object3D:
        """Create a 3D visualization of the wing"""
        # Get the wing's aerodynamic visualization
        wing_obj = self.plot()
        wing_obj.metadata['color'] = 'gray'  # Set wing color
        
        # Plot all child components (fuel tanks)
        for child in self.children:
            if hasattr(child, 'plot'):
                child_obj = child.plot()
                # Add all shapes from the child object
                for shape in child_obj.shapes:
                    wing_obj.add_shape(shape)
        
        return wing_obj

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
    wing = Wing()
    
    # Print fuel tank information
    print("\n=== Fuel Tank Information ===")
    for tank in wing.children:
        if isinstance(tank, FuelTank):
            print(f"\nTank: {tank.name}")
            print(f"Dimensions: {tank.length:.1f}ft x {tank.width:.1f}ft x {tank.front_height:.1f}ft (front) x {tank.back_height:.1f}ft (back)")
            print(f"Volume: {tank.volume:.1f} ft³")
            print(f"Position: ({tank.geometry.position.x:.1f}, {tank.geometry.position.y:.1f}, {tank.geometry.position.z:.1f})")
            print(f"Empty: {tank.empty}")
    
    # run the mass analysis
    wing.run_analysis(analysis_names="mass_analysis")

    results = wing.analysis_results['mass_analysis']

    print(f"\n=== Wing Properties ===")
    print(f"Total Mass: {results['total_mass']:.1f} lbs")
    print(f"CG Position: ({results['cg_x']:.2f}, {results['cg_y']:.2f}, {results['cg_z']:.2f}) ft")
    print(f"Moments of Inertia:")
    print(f"  Ixx: {results['total_ixx']:.0f} lbs-ft²")
    print(f"  Iyy: {results['total_iyy']:.0f} lbs-ft²")
    print(f"  Izz: {results['total_izz']:.0f} lbs-ft²")

    # print the total fuel volume and mass
    total_fuel_volume = 0.0
    total_fuel_mass = 0.0
    for tank in wing.children:
        if isinstance(tank, FuelTank):
            total_fuel_volume += tank.volume
            total_fuel_mass += tank.fuel_mass
    print(f"Total Fuel Volume: {total_fuel_volume:.0f} ft³")
    print(f"Total Fuel Mass: {total_fuel_mass:.0f} lbs")

    # Make one tank empty for visualization
    #wing.children[0].set_empty(True)
    
    # Create the wing object for plotting (includes fuel tanks)
    wing_obj = wing.plot(plot_children=True)
    
    # Create figure and plot orthographic views
    fig = plt.figure(figsize=(15, 10))
    fig, (ax_top, ax_side, ax_front) = plot_orthographic_views(wing_obj, fig=fig)
    
    # Add title and adjust layout
    plt.suptitle("Wing with Fuel Tanks", fontsize=16)
    plt.tight_layout()
    
    # Save the plot
    plt.savefig("assets/wing_orthographic_views.png")
    
    # Close the plot
    plt.close()
