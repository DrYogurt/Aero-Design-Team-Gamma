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
        
        # Create a cylinder for the engine
        cylinder = create_cylinder(
            self.parameters['radius'],
            self.parameters['length']
        )
        
        # Add cylinder to object
        obj.add_shape(cylinder)
        
        # Rotate cylinder 90 degrees around Y axis to face forward
        # This is done by rotating the vertices directly
        for shape in obj.shapes:
            # Swap X and Z coordinates to rotate 90 degrees around Y
            shape.vertices[:, [0, 2]] = shape.vertices[:, [2, 0]]  # Swap X and Z
            shape.vertices[:, 0] *= -1  # Negate X to complete rotation
        
        # Apply position
        if self.position:
            obj.position = np.array([self.position.x, self.position.y, self.position.z])
        
        return obj
class Engine(Component):
    """Represents an aircraft engine"""
    def __init__(self, name: str):
        super().__init__(name)
        self.geometry = EngineGeometry() 