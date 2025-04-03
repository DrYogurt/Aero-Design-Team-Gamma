from typing import List, Dict, Any
import numpy as np
import matplotlib.pyplot as plt

from aircraft_design.core.base import Component, Position, Orientation
from aircraft_design.analysis.mass_analysis import MassFeature, MassAnalysis
from aircraft_design.core.plotting import Object3D
from aircraft_design.components.aerodynamics.wing_geometry import SimpleSweptWing, TailGeometry


class HorizontalTail(Component):
    """Horizontal tail component with hard-coded dimensions from lw-jp.py"""
    
    def __init__(self, position, wing_ref, volume_ratio=0.7):
        """Initialize horizontal tail with hardcoded sizing"""
        super().__init__(name="horizontal_tail")
        
        # Store position
        self.position = position
        
        # Hardcoded values from lw-jp.py
        self.span = 75  # feet
        self.root_chord = 25  # feet
        self.tip_chord = 15  # feet
        self.sweep_angle = 42.0  # degrees
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
        self.geometry.orientation = Orientation(roll=0.0, pitch=2.0, yaw=0.0)  # 2 degrees initial pitch
        
        # Mass properties (simplified model - approximately 10% of wing mass)
        self.mass = 0.1 * wing_ref.mass
        self.features.append(MassFeature(self.mass, [position.x, position.y, position.z]))
        
        # Add analysis modules
        self.add_analysis(MassAnalysis())
        
    def set_incidence(self, angle_deg: float) -> None:
        """Set the tail incidence angle in degrees"""
        if self.geometry:
            self.geometry.orientation.pitch = angle_deg


class VerticalTail(Component):
    """Vertical tail component with hard-coded dimensions from lw-jp.py"""
    
    def __init__(self, position, wing_ref, volume_ratio=0.08):
        """Initialize vertical tail with hardcoded sizing"""
        super().__init__(name="vertical_tail")
        
        # Store position
        self.position = position
        
        # Hardcoded values from lw-jp.py
        self.height = 50  # feet
        self.root_chord = 40  # feet
        self.tip_chord = 20  # feet
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
        
        # Mass properties (simplified model - approximately 5% of wing mass)
        self.mass = 0.05 * wing_ref.mass
        self.features.append(MassFeature(self.mass, [position.x, position.y, position.z]))
        
        # Add analysis modules
        self.add_analysis(MassAnalysis())
    