from typing import List, Dict, Any
import numpy as np

from aircraft_design.core.base import Component, Position, Geometry, Feature
from aircraft_design.analysis.mass_analysis import MassFeature, MassAnalysis
from aircraft_design.core.plotting import Object3D, create_box

class FillLevel(Feature):
    """Feature that modifies mass properties based on fill level percentage"""
    def __init__(self, fill_level: float = 1.0):
        """
        Initialize fill level feature
        
        Args:
            fill_level: Fill level as a percentage (0.0 to 1.0)
        """
        super().__init__(name="fill_level")
        self.fill_level = max(0.0, min(1.0, fill_level))  # Clamp between 0 and 1
        
    def modify_analysis(self, analysis: MassAnalysis, component: Component) -> None:
        """Modify mass analysis by scaling mass properties by fill level"""
        #print(f"Fill level: {self.fill_level}")
        if isinstance(analysis, MassAnalysis):
            # Find the mass feature in the component
            for feature in component.features:
                if isinstance(feature, MassFeature):
                    # Scale all mass properties by fill level
                    feature.parameters["mass"] *= self.fill_level
                    feature.parameters["ixx"] *= self.fill_level
                    feature.parameters["iyy"] *= self.fill_level
                    feature.parameters["izz"] *= self.fill_level
                    feature.parameters["ixy"] *= self.fill_level
                    feature.parameters["ixz"] *= self.fill_level
                    feature.parameters["iyz"] *= self.fill_level
                    break
    
    def validate(self) -> bool:
        """Validate that fill level is between 0 and 1"""
        return 0.0 <= self.fill_level <= 1.0

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
            'fill_level': 1.0
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
        
        # Apply the position transformation to the shape vertices
        if self.position:
            tank_shape.vertices = tank_shape.vertices.astype(np.float64)
            tank_shape.vertices += np.array([
                self.position.x,
                self.position.y,
                self.position.z
            ])
        
        obj.add_shape(tank_shape)
        return obj
    
    def validate(self) -> bool:
        """Validate the geometry parameters"""
        return all(v >= 0 for v in [
            self.parameters['length'],
            self.parameters['width'],
            self.parameters['front_height'],
            self.parameters['back_height'],
            self.parameters['fill_level']
        ])

class FuelTank(Component):
    """A fuel tank component with trapezoidal volume and mass analysis"""
    
    def __init__(self, 
                 length: float,
                 front_height: float,
                 back_height: float,
                 width: float,
                 fuel_density: float = 50.0,  # lb/ft^3, typical for aviation fuel
                 fill_level: float = 1.0):    # Fill level as percentage (0.0 to 1.0)
        """
        Initialize a fuel tank with trapezoidal volume
        
        Args:
            length: Length of the tank in feet
            front_height: Height at front of tank in feet
            back_height: Height at back of tank in feet
            width: Width of the tank in feet
            fuel_density: Density of the fuel in lb/ft^3
            fill_level: Fill level as a percentage (0.0 to 1.0)
        """
        super().__init__(name="fuel_tank")
        
        # Store basic parameters
        self.length = length
        self.front_height = front_height
        self.back_height = back_height
        self.width = width
        self.fuel_density = fuel_density
        
        # Calculate volume (trapezoidal prism)
        self.volume = (front_height + back_height) / 2 * length * width
        
        # Calculate full mass based on volume and density
        self.full_mass = self.volume * fuel_density
        
        # Calculate center of gravity (assuming uniform density)
        # For a trapezoidal prism, CG is at the centroid
        cg_x = length / 2
        cg_y = width / 2
        cg_z = (front_height + back_height) / 3  # Height of centroid for trapezoid
        
        # Add mass feature with full mass
        self.add_feature(MassFeature(
            mass=self.full_mass,
            center_of_gravity=[cg_x, cg_y, cg_z],
            ixx=self._calculate_ixx(self.full_mass),
            iyy=self._calculate_iyy(self.full_mass),
            izz=self._calculate_izz(self.full_mass)
        ))
        
        # Add fill level feature
        self.add_feature(FillLevel(fill_level))
        
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
            'fill_level': fill_level
        })
        
        # Set the color based on fill level
        self.color = 'red' if fill_level == 0.0 else 'green'
    
    def _calculate_ixx(self, mass: float) -> float:
        """Calculate moment of inertia about x-axis"""
        # Simplified calculation for rectangular prism approximation
        return mass * (self.width**2 + self.front_height**2) / 12
    
    def _calculate_iyy(self, mass: float) -> float:
        """Calculate moment of inertia about y-axis"""
        # Simplified calculation for rectangular prism approximation
        return mass * (self.length**2 + self.front_height**2) / 12
    
    def _calculate_izz(self, mass: float) -> float:
        """Calculate moment of inertia about z-axis"""
        # Simplified calculation for rectangular prism approximation
        return mass * (self.length**2 + self.width**2) / 12
    
    def set_fill_level(self, fill_level: float) -> None:
        """Set the tank fill level and update mass accordingly"""
        # Update fill level feature
        for feature in self.features:
            if isinstance(feature, FillLevel):
                feature.fill_level = max(0.0, min(1.0, fill_level))
                break
        
        # Update geometry parameters
        self.geometry.parameters['fill_level'] = fill_level
        
        # Update color based on fill level
        self.color = 'red' if fill_level == 0.0 else 'green'
    
    def plot(self, *args, **kwargs) -> Object3D:
        """Create a 3D visualization of the fuel tank"""
        tank_obj = self.geometry.create_object()
        
        # Set color in metadata for each shape
        for shape in tank_obj.shapes:
            shape.metadata['color'] = self.color
            
        return tank_obj
