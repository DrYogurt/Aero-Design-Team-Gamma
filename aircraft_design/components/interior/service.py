from dataclasses import dataclass
from typing import List, Optional
import numpy as np
from .base import InteriorComponent, InteriorGeometry
from aircraft_design.core.plotting import Object3D, Shape3D, create_box
from aircraft_design.analysis.mass_analysis import MassAnalysis, MassFeature

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
        
        # Add mass analysis
        self.add_analysis(MassAnalysis())
        
        # Add mass (~1000 lbs for galley with equipment)
        mass = 1000
        
        # Calculate moments of inertia
        ixx = (1/12) * mass * (height**2 + depth**2)
        iyy = (1/12) * mass * (width**2 + height**2)
        izz = (1/12) * mass * (width**2 + depth**2)
        
        # Add mass feature
        mass_feature = MassFeature(
            mass=mass,
            center_of_gravity=[depth/2, width/2, height/2],
            ixx=ixx, iyy=iyy, izz=izz,
            ixy=0, ixz=0, iyz=0
        )
        self.add_feature(mass_feature)

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
        
        # Add mass analysis
        self.add_analysis(MassAnalysis())
        
        # Add mass (~500 lbs for bathroom with fixtures)
        mass = 500
        
        # Calculate moments of inertia
        ixx = (1/12) * mass * (height**2 + depth**2)
        iyy = (1/12) * mass * (width**2 + height**2)
        izz = (1/12) * mass * (width**2 + depth**2)
        
        # Add mass feature
        mass_feature = MassFeature(
            mass=mass,
            center_of_gravity=[depth/2, width/2, height/2],
            ixx=ixx, iyy=iyy, izz=izz,
            ixy=0, ixz=0, iyz=0
        )
        self.add_feature(mass_feature)
    
    def plot(self, color: str = 'purple') -> Object3D:
        return super().plot(color) 
    

class Stairs(ServiceArea):
    """Aircraft stairs with standard dimensions"""
    def __init__(self, name: str, width: float = 3.0, depth: float = 3.0, height: float = 7.0):
        super().__init__(name)
        self.geometry.parameters.update({
            'width': width,
            'depth': depth,
            'height': height
        })
        
        # Add mass analysis
        self.add_analysis(MassAnalysis())
        
        # Add mass (~800 lbs for stairs structure)
        mass = 800
        
        # Calculate moments of inertia
        ixx = (1/12) * mass * (height**2 + depth**2)
        iyy = (1/12) * mass * (width**2 + height**2)
        izz = (1/12) * mass * (width**2 + depth**2)
        
        # Add mass feature
        mass_feature = MassFeature(
            mass=mass,
            center_of_gravity=[depth/2, width/2, height/2],
            ixx=ixx, iyy=iyy, izz=izz,
            ixy=0, ixz=0, iyz=0
        )
        self.add_feature(mass_feature)
    
    def plot(self, color: str = 'orange') -> Object3D:
        return super().plot(color)
