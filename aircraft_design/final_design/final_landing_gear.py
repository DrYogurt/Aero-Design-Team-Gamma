from typing import List, Dict, Any
import numpy as np
import matplotlib.pyplot as plt

from aircraft_design.core.base import Component, Position
from aircraft_design.analysis.mass_analysis import MassFeature, MassAnalysis
from aircraft_design.core.plotting import Object3D

class LandingGear(Component):
    """Landing gear component with mass analysis"""
    
    def __init__(self, name: str = "landing_gear", position: Position = None, num_wheels: int = 2, wheel_mass: float = 100):
        """
        Initialize the landing gear component
        
        Args:
            name: Component name
            position: Position of the landing gear relative to aircraft
        """
        super().__init__(name=name)
        
        # Store position
        self.position = position if position else Position()
        
        # Landing gear parameters
        self.mass = num_wheels * wheel_mass  # lbs, typical for main landing gear
        self.height = 8.0  # feet
        self.width = 3.0   # feet
        self.depth = 3.0   # feet
        
        # Add mass analysis
        self._add_mass_analysis()
        
    def _add_mass_analysis(self):
        """Add mass analysis with moments of inertia"""
        # Calculate moments of inertia for a rectangular prism
        ixx = (1/12) * self.mass * (self.height**2 + self.depth**2)
        iyy = (1/12) * self.mass * (self.width**2 + self.height**2)
        izz = (1/12) * self.mass * (self.width**2 + self.depth**2)
        
        # Add mass feature
        mass_feature = MassFeature(
            mass=self.mass,
            center_of_gravity=[
                self.position.x + self.depth/2,
                self.position.y + self.width/2,
                self.position.z + self.height/2
            ],
            ixx=ixx,
            iyy=iyy,
            izz=izz,
            ixy=0,  # Assuming symmetric mass distribution
            ixz=0,
            iyz=0
        )
        
        self.add_feature(mass_feature)
        self.add_analysis(MassAnalysis())
        
    def plot(self, *args, **kwargs) -> Object3D:
        """Create a 3D visualization of the landing gear"""
        # Create a simple box representation
        obj = Object3D()
        obj.add_box(
            self.position.x,
            self.position.y,
            self.position.z,
            self.depth,
            self.width,
            self.height,
            color='gray'
        )
        return obj
