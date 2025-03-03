from dataclasses import dataclass
from typing import List, Optional
import numpy as np
from .base import InteriorComponent, InteriorGeometry
from aircraft_design.core.plotting import Object3D, Shape3D, create_box

@dataclass
class ServiceGeometry(InteriorGeometry):
    """Base geometry for service areas with basic dimensions"""
    def __init__(self):
        super().__init__()
        self.parameters.update({
            'width': 0.0,
            'depth': 0.0,
            'height': 0.0,
        })

class ServiceArea(InteriorComponent):
    """Base class for service areas"""
    def __init__(self, name: str):
        super().__init__(name)
        self.geometry = ServiceGeometry()

    def plot(self, color: str) -> Object3D:
        """Create a 3D visualization of the service area"""
        obj = Object3D()
        
        # Get global position
        global_pos = self.get_global_position()
        
        service_box = create_box(
            width=self.geometry.parameters['width'],
            length=self.geometry.parameters['depth'],
            height=self.geometry.parameters['height']
        )
        service_box.vertices += global_pos
        service_box.metadata.update({
            'type': self.__class__.__name__.lower(),
            'color': color
        })
        obj.add_shape(service_box)
        return obj

class Galley(ServiceArea):
    """Aircraft galley with standard dimensions"""
    def __init__(self, name: str, width: float = 8.0, depth: float = 3.0, height: float = 7.0):
        super().__init__(name)
        self.geometry.parameters.update({
            'width': width,
            'depth': depth,
            'height': height
        })

    def plot(self, color: str = 'green') -> Object3D:
        return super().plot(color)

class Bathroom(ServiceArea):
    """Aircraft bathroom with standard dimensions"""
    def __init__(self, name: str, width: float = 3.0, depth: float = 3.0, height: float = 7.0):
        super().__init__(name)
        self.geometry.parameters.update({
            'width': width,
            'depth': depth,
            'height': height
        })
    
    def plot(self, color: str = 'purple') -> Object3D:
        return super().plot(color) 