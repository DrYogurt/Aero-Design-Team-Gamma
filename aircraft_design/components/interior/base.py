from dataclasses import dataclass
from typing import List, Optional
from aircraft_design.core.base import Component, Geometry
from aircraft_design.core.plotting import Object3D
@dataclass
class InteriorGeometry(Geometry):
    """Base geometry for interior components"""
    def __init__(self):
        super().__init__()
        self.parameters.update({
            'length': 0.0,
            'width': 0.0,
            'height': 0.0,
        })
    
    def validate(self) -> bool:
        """
        Validate the interior geometry parameters
        Returns True if all dimensions are non-negative
        """
        return all(
            isinstance(v, (int, float)) and v >= 0 
            for k, v in self.parameters.items()
        )
    
    def create_object(self) -> Object3D:
        """Create a 3D object representation of the interior geometry"""
        pass

class InteriorComponent(Component):
    """Base class for all interior components"""
    def __init__(self, name: str):
        super().__init__(name)
        self.geometry = InteriorGeometry() 