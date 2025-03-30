from dataclasses import dataclass
from typing import List, Optional
from aircraft_design.core.base import Component, Geometry
from aircraft_design.core.plotting import Object3D
from aircraft_design.analysis.mass_analysis import MassAnalysis, MassFeature

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
        self.add_feature(MassFeature())
        self.add_analysis(MassAnalysis())
    @property
    def total_length(self) -> float:
        """Get the total length of the component"""
        return self.geometry.parameters['length']
    
    @property
    def total_width(self) -> float:
        """Get the total width of the component"""
        return self.geometry.parameters['width']
    
    @property
    def total_height(self) -> float:
        """Get the total height of the component"""
        return self.geometry.parameters['height']
    
    @property
    def total_volume(self) -> float:
        """Get the total volume of the component"""
        return self.geometry.parameters['length'] * self.geometry.parameters['width'] * self.geometry.parameters['height']