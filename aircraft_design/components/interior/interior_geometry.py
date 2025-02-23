from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import numpy as np
from aircraft_design.core.base import Geometry
from aircraft_design.core.plotting import Object3D, Shape3D, create_cylinder, create_ellipsoid

class InteriorGeometry(Geometry):
    """Geometry for interior components like cabin, cargo holds, etc."""
    def __init__(self):
        super().__init__()
        self.parameters = {
            'length': 0.0,
            'width': 0.0,
            'height': 0.0,
            'floor_height': 0.0,  # Height of floor above fuselage bottom
        }
        self.sections = []

    def validate(self) -> bool:
        """Validate interior geometry parameters"""
        # Check for positive dimensions
        for param in ['length', 'width', 'height', 'floor_height']:
            if self.parameters[param] <= 0:
                return False
        return True

    def create_object(self) -> Object3D:
        """Create a 3D object representation of the interior"""
        obj = Object3D()
        
        # Create main cabin volume as a rectangular prism
        length = self.parameters['length']
        width = self.parameters['width']
        height = self.parameters['height']
        
        # Create floor
        floor = create_cylinder(width/2, length, num_points=4)  # Square cross-section
        floor.vertices += np.array([self.position.x,
                                  self.position.y,
                                  self.position.z + self.parameters['floor_height']])
        floor.metadata['type'] = 'floor'
        obj.add_shape(floor)
        
        # Create ceiling
        ceiling = create_cylinder(width/2, length, num_points=4)
        ceiling.vertices += np.array([self.position.x,
                                    self.position.y,
                                    self.position.z + height])
        ceiling.metadata['type'] = 'ceiling'
        obj.add_shape(ceiling)
        
        # Create walls (simplified as cylinders)
        left_wall = create_cylinder(height/2, length, num_points=4)
        left_wall.vertices = np.roll(left_wall.vertices, 1, axis=1)  # Rotate 90 degrees
        left_wall.vertices += np.array([self.position.x,
                                      self.position.y - width/2,
                                      self.position.z + height/2])
        left_wall.metadata['type'] = 'wall'
        obj.add_shape(left_wall)
        
        right_wall = create_cylinder(height/2, length, num_points=4)
        right_wall.vertices = np.roll(right_wall.vertices, 1, axis=1)  # Rotate 90 degrees
        right_wall.vertices += np.array([self.position.x,
                                       self.position.y + width/2,
                                       self.position.z + height/2])
        right_wall.metadata['type'] = 'wall'
        obj.add_shape(right_wall)
        
        return obj 