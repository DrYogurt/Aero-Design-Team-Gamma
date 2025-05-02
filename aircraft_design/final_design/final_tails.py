from typing import List, Dict, Any
import numpy as np
import matplotlib.pyplot as plt

from aircraft_design.core.base import Component, Position, Orientation
from aircraft_design.analysis.mass_analysis import MassFeature, MassAnalysis
from aircraft_design.core.plotting import Object3D
from aircraft_design.components.aerodynamics.wing_geometry import SimpleSweptWing, TailGeometry
from aircraft_design.components.propulsion.fuel_tanks import FuelTank


class HorizontalTail(Component):
    """Horizontal tail component with hard-coded dimensions from lw-jp.py"""
    
    def __init__(self, position, wing_ref):
        """Initialize horizontal tail with hardcoded sizing"""
        super().__init__(name="horizontal_tail")
        
        # Store position
        self.position = position
        
        # Hardcoded values from lw-jp.py
        self.span = 70  # feet
        self.root_chord = 25  # feet
        self.tip_chord = 10  # feet
        self.sweep_angle = 37.0  # degrees
        self.dihedral = 0.0  # degrees
        
        # Calculate derived values
        self.tail_area = (self.span/2) * (self.root_chord + self.tip_chord)
        self.aspect_ratio = (self.span ** 2) / self.tail_area
        self.taper_ratio = self.tip_chord / self.root_chord
        self.mean_chord = self.tail_area / self.span
        
        # Reference values from wing - still needed for some calculations
        self.wing_area = wing_ref.wing_area
        self.wing_mac = wing_ref.mean_aerodynamic_chord
        self.tail_arm = position.x - wing_ref.geometry.position.x - 0.25 * wing_ref.mean_aerodynamic_chord
        
        # Calculate actual volume ratio based on hardcoded values
        self.volume_ratio = (self.tail_area * self.tail_arm) / (self.wing_area * self.wing_mac)
        
        # Create geometry using SimpleSweptWing
        self.geometry = SimpleSweptWing(
            root_chord=self.root_chord,
            tip_chord=self.tip_chord,
            span=self.span,
            sweep=self.sweep_angle,
            dihedral=self.dihedral
        )
        self.geometry.position = self.position
        
        # Set initial incidence angle (stored in orientation)
        self.geometry.orientation = Orientation(roll=0.0, pitch=7.2, yaw=0.0)  # 2 degrees initial pitch
        
        # Mass properties (simplified model - approximately 10% of wing mass)
        # self.mass = 0 # ~40% of wing mass
        # self.features.append(MassFeature(self.mass, [position.x, position.y, position.z]))
        
        # Add analysis modules
        self.add_analysis(MassAnalysis())

        # Add fuel tank capable of storing 540 ft 3 of fuel
        tank = FuelTank(
            length=9,
            front_height=3,  # 10% of root chord
            back_height=3,  # 10% of root chord
            width=20,
            fill_level=1.0  # Full tank
        )
        tank.name = "horizontal_tail_fuel_tank"
        
        # Position tank in the center of the tail
        tank.geometry.position = Position(
            x=position.x,
            y=position.y,
            z=position.z
        )
        self.add_child(tank)

        # Add control surfaces
        self._add_control_surfaces()
        
    def _add_control_surfaces(self):
        """Add stabilators to the horizontal tail"""
        # Stabilator parameters
        self.stabilator_start = 0.1  # Start at 10% of half span
        self.stabilator_end = 0.9    # End at 90% of half span
        self.stabilator_chord_ratio = 0.30  # Stabilator is 30% of local chord
        
        # Calculate control surface areas
        self._calculate_control_surface_areas()

    def _calculate_control_surface_areas(self):
        """Calculate the areas of stabilators"""
        # Get tail parameters
        span = self.span
        root_chord = self.root_chord
        tip_chord = self.tip_chord
        
        # Calculate half span positions
        half_span = span / 2
        stabilator_start_y = self.stabilator_start * half_span
        stabilator_end_y = self.stabilator_end * half_span
        
        # Calculate chord lengths at control surface boundaries
        def chord_at_y(y):
            return root_chord * (1 - y/half_span) + tip_chord * (y/half_span)
        
        # Calculate stabilator area (trapezoidal integration)
        stabilator_chord_start = chord_at_y(stabilator_start_y)
        stabilator_chord_end = chord_at_y(stabilator_end_y)
        stabilator_span = stabilator_end_y - stabilator_start_y
        stabilator_area = (stabilator_chord_start + stabilator_chord_end) * stabilator_span * self.stabilator_chord_ratio
        
        # Store areas (multiply by 2 for both sides)
        self.stabilator_area = stabilator_area
        
        # Calculate total control surface area
        self.total_control_surface_area = self.stabilator_area

    def set_incidence(self, angle_deg: float) -> None:
        """Set the tail incidence angle in degrees"""
        if self.geometry:
            self.geometry.orientation.pitch = angle_deg


class VerticalTail(Component):
    """Vertical tail component with hard-coded dimensions from lw-jp.py"""
    
    def __init__(self, position, wing_ref):
        """Initialize vertical tail with hardcoded sizing"""
        super().__init__(name="vertical_tail")
        
        # Store position
        self.position = position
        
        # Hardcoded values from lw-jp.py
        self.height = 50  # feet
        self.root_chord = 45  # feet
        self.tip_chord = 15.75  # feet
        self.sweep_angle = 40.0  # degrees
        self.thickness_ratio = 0.12
        self.cant_angle = 0.0  # degrees (vertical)
        
        # Calculate derived values
        self.tail_area = 0.5 * (self.root_chord + self.tip_chord) * self.height
        self.aspect_ratio = (self.height ** 2) / self.tail_area
        self.taper_ratio = self.tip_chord / self.root_chord
        
        # Reference values from wing - still needed for some calculations
        self.wing_area = wing_ref.wing_area
        self.wing_span = wing_ref.wing_span
        self.tail_arm = position.x - wing_ref.geometry.position.x - 0.25 * wing_ref.mean_aerodynamic_chord
        
        # Calculate actual volume ratio based on hardcoded values
        self.volume_ratio = (self.tail_area * self.tail_arm) / (self.wing_area * self.wing_span)
        
        # Create geometry
        self.geometry = TailGeometry()
        self.geometry.parameters.update({
            'height': self.height,
            'root_chord': self.root_chord,
            'tip_chord': self.tip_chord,
            'sweep': self.sweep_angle,
            'cant_angle': self.cant_angle,
            'thickness_ratio': self.thickness_ratio
        })
        
        self.geometry.position = self.position
        
        # Mass properties (simplified model - approximately 10% of wing mass)
        # self.mass = 0 # ~25% of wing mass
        # self.features.append(MassFeature(self.mass, [position.x, position.y, position.z]))
        
        # Add analysis modules
        self.add_analysis(MassAnalysis())

        # Add control surfaces
        self._add_control_surfaces()

    def _add_control_surfaces(self):
        """Add rudder to the vertical tail"""
        # Rudder parameters
        self.rudder_start = 0.1  # Start at 10% of height
        self.rudder_end = 0.9    # End at 90% of height
        self.rudder_chord_ratio = 0.25  # Rudder is 25% of local chord
        
        # Calculate control surface areas
        self._calculate_control_surface_areas()

    def _calculate_control_surface_areas(self):
        """Calculate the areas of rudder"""
        # Get tail parameters
        height = self.height
        root_chord = self.root_chord
        tip_chord = self.tip_chord
        
        # Calculate height positions
        rudder_start_z = self.rudder_start * height
        rudder_end_z = self.rudder_end * height
        
        # Calculate chord lengths at control surface boundaries
        def chord_at_z(z):
            return root_chord * (1 - z/height) + tip_chord * (z/height)
        
        # Calculate rudder area (trapezoidal integration)
        rudder_chord_start = chord_at_z(rudder_start_z)
        rudder_chord_end = chord_at_z(rudder_end_z)
        rudder_height = rudder_end_z - rudder_start_z
        rudder_area = (rudder_chord_start + rudder_chord_end) / 2 * rudder_height * self.rudder_chord_ratio
        
        # Store areas
        self.rudder_area = rudder_area
        
        # Calculate total control surface area
        self.total_control_surface_area = self.rudder_area


if __name__ == "__main__":
    from aircraft_design.final_design.final_wing import Wing

    # Create wing reference
    wing_ref = Wing(wing_tip_position = 85)
    fuselage_length = 50
    transition_length = 70
    short_fuselage_length = fuselage_length + transition_length + 60
        
    tall_fuselage_height = 35
    short_fuselage_height = 27
        
    fuselage_width = 27
        
    nose_length = 30
    tail_length = 62
    tail_height = 18

    # Create horizontal tail
    htail_position = Position(
        x=nose_length + short_fuselage_length + tail_length - 30,
        y=0,
        z=short_fuselage_height - 5
    )

    # Create vertical tail
    vtail_position = Position(
        x=nose_length + short_fuselage_length + tail_length - 35,
        y=0,
        z=short_fuselage_height
    )

    # Create tails with fuel tanks
    horizontal_tail = HorizontalTail(
        position=htail_position,
        wing_ref=wing_ref
    )
    
    vertical_tail = VerticalTail(
        position=vtail_position,
        wing_ref=wing_ref
    )

    # Run mass analysis for both tails
    horizontal_tail.run_analysis(analysis_names="mass_analysis")
    vertical_tail.run_analysis(analysis_names="mass_analysis")

    # Get analysis results
    h_tail_results = horizontal_tail.analysis_results['mass_analysis']
    v_tail_results = vertical_tail.analysis_results['mass_analysis']

    print("\n=== Horizontal Tail Analysis ===")
    print(f"Tail Area: {horizontal_tail.tail_area:.2f} ft²")
    print(f"Volume Ratio: {horizontal_tail.volume_ratio:.2f}")
    print(f"Total Mass: {h_tail_results['total_mass']:.1f} lbs")
    print(f"CG Position: ({h_tail_results['cg_x']:.2f}, {h_tail_results['cg_y']:.2f}, {h_tail_results['cg_z']:.2f}) ft")
    print(f"Fuel Tank Mass: {horizontal_tail.children[0].full_mass * horizontal_tail.children[0].features[1].fill_level:.1f} lbs")
    print(f"Fuel Tank Volume: {horizontal_tail.children[0].volume:.1f} ft³")
    print(f"Fuel Tank Fill Level: {horizontal_tail.children[0].features[1].fill_level * 100:.1f}%")


    print("\n=== Vertical Tail Analysis ===")
    print(f"Tail Area: {vertical_tail.tail_area:.2f} ft²")
    print(f"Volume Ratio: {vertical_tail.volume_ratio:.2f}")
    print(f"Total Mass: {v_tail_results['total_mass']:.1f} lbs")
    print(f"CG Position: ({v_tail_results['cg_x']:.2f}, {v_tail_results['cg_y']:.2f}, {v_tail_results['cg_z']:.2f}) ft")
    #print(f"Fuel Tank Mass: {vertical_tail.children[0].full_mass * vertical_tail.children[0].features[1].fill_level:.1f} lbs")
    #print(f"Fuel Tank Volume: {vertical_tail.children[0].volume:.1f} ft³")
    #print(f"Fuel Tank Fill Level: {vertical_tail.children[0].features[1].fill_level * 100:.1f}%")
    
    # Calculate total fuel mass and volume
    h_tank = horizontal_tail.children[0]
    #v_tank = vertical_tail.children[0]
    total_fuel_mass = (h_tank.full_mass * h_tank.features[1].fill_level)
    total_fuel_volume = h_tank.volume
    print("\n=== Total Fuel Analysis ===")
    print(f"Total Fuel Mass: {total_fuel_mass:.1f} lbs")
    print(f"Total Fuel Volume: {total_fuel_volume:.1f} ft³")
    
    # Print control surface information
    print("\n=== Control Surface Information ===")
    print("\nHorizontal Tail:")
    print(f"Stabilator Area: {horizontal_tail.stabilator_area:.1f} ft²")
    print(f"Stabilator Area Ratio: {horizontal_tail.stabilator_area/horizontal_tail.tail_area:.3f}")
    
    print("\nVertical Tail:")
    print(f"Rudder Area: {vertical_tail.rudder_area:.1f} ft²")
    print(f"Rudder Area Ratio: {vertical_tail.rudder_area/vertical_tail.tail_area:.3f}")
    
    print("\nTotal Control Surface Areas:")
    print(f"Horizontal Tail: {horizontal_tail.total_control_surface_area:.1f} ft²")
    print(f"Vertical Tail: {vertical_tail.total_control_surface_area:.1f} ft²")
    