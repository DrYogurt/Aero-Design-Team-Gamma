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
            'bypass_ratio': 0.0,
            'thrust': 0.0
        })
    
    def validate(self) -> bool:
        return all(v > 0 for v in self.parameters.values())
    
    def create_object(self) -> Object3D:
        """Create a 3D object representation of this geometry"""
        obj = Object3D()
        
        # Create a cylinder for the engine, rotated 90 degrees around Y axis to face forward
        cylinder = create_cylinder(
            self.parameters['radius'],
            self.parameters['length'],
            rotation=np.array([0, np.pi/2, 0])  # Rotate 90 degrees around Y axis
        )
        obj.add_shape(cylinder)
        
        # Apply position after creating the engine
        obj.position = np.array([self.position.x, self.position.y, self.position.z])
        
        return obj
class Engine(Component):
    """Represents an aircraft engine"""
    def __init__(self, name: str):
        super().__init__(name)
        self.geometry = EngineGeometry() 