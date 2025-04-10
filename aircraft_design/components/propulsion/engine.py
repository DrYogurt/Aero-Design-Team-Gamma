from dataclasses import dataclass
from aircraft_design.core.base import Component, Geometry
from aircraft_design.core.plotting import Object3D, create_cylinder
import numpy as np

@dataclass
class EngineGeometry(Geometry):
    """Basic engine geometry"""
    def __init__(self):
        super().__init__()
        self.parameters.update({
            'radius': 0.0,
            'length': 0.0,
        })
        # Set default orientation for horizontal alignment
        self.orientation.pitch = -90  # Rotate -90 degrees around Y axis to point forward
    
    def validate(self) -> bool:
        return all(v > 0 for v in self.parameters.values())
    
    @property
    def wetted_area(self) -> float:
        """Calculate the wetted area of the engine as a cylinder
        
        Returns:
            float: Total surface area including lateral surface and end caps
        """
        radius = self.parameters['radius']
        length = self.parameters['length']
        
        # Lateral surface area of cylinder (2πrh)
        lateral_area = 2 * np.pi * radius * length
        
        # Area of two circular end caps (2πr²)
        end_caps_area = 2 * np.pi * radius * radius
        
        return lateral_area + end_caps_area
    
    def create_object(self) -> Object3D:
        """Create a 3D object representation of this geometry"""
        obj = Object3D()
        
        # Create a cylinder for the engine
        cylinder = create_cylinder(
            self.parameters['radius'],
            self.parameters['length']
        )
        
        # Rotate cylinder to point forward (around Y axis)
        pitch_rad = np.radians(self.orientation.pitch)
        Ry = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                      [0, 1, 0],
                      [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
        
        # Apply rotation to vertices
        cylinder.vertices = cylinder.vertices @ Ry.T
        
        obj.add_shape(cylinder)

        obj.position = np.array([self.position.x, self.position.y, self.position.z])
        
        return obj

class Engine(Component):
    """Represents an aircraft engine"""
    def __init__(self, name: str, thrust: float):
        super().__init__(name)
        self.geometry = EngineGeometry() 
        self.thrust = thrust
    
    def plot(self, *args, colors_dict=None, plot_children=True, **kwargs):
        """Custom plot method for engines to ensure correct positioning"""
        obj = super().plot(*args, colors_dict=colors_dict, plot_children=plot_children, **kwargs)
        
        # print the position of the engine
        #print(f"Engine position: {obj.position}")
        return obj
    
