from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Any, Optional
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from aircraft_design.core.plotting import Object3D

@dataclass
class Position:
    """3D position in aircraft coordinates"""
    x: float = 0.0  # Positive forward
    y: float = 0.0  # Positive right
    z: float = 0.0  # Positive up

@dataclass
class Orientation:
    """Orientation relative to parent component in degrees"""
    roll: float = 0.0    # Rotation about x-axis
    pitch: float = 0.0   # Rotation about y-axis
    yaw: float = 0.0     # Rotation about z-axis

class Geometry(ABC):
    """Base class for all geometric definitions"""
    def __init__(self):
        self.position = Position()
        self.orientation = Orientation()
        self.parameters = {}

    @abstractmethod
    def validate(self) -> bool:
        """Validate the geometry parameters"""
        pass

    @abstractmethod
    def create_object(self) -> Object3D:
        """Create a 3D object representation of this geometry"""
        pass

    def plot(self, ax: Axes3D, color: str = 'blue', alpha: float = 0.5) -> None:
        """Plot the geometry using its 3D object representation"""
        from aircraft_design.core.plotting import plot_3d_object
        obj = self.create_object()
        plot_3d_object(ax, obj, color, alpha)

class AnalysisModule(ABC):
    """Base class for all analysis modules"""
    def __init__(self, name: str):
        self.name = name
        self.parameters = {}
        self.results = {}
        self.required_parameters = []

    @abstractmethod
    def run(self, component: 'Component') -> Dict[str, Any]:
        """Run the analysis on the component"""
        pass

    def validate_parameters(self) -> bool:
        """Check if all required parameters are set"""
        return all(param in self.parameters for param in self.required_parameters)

class Feature(ABC):
    """Base class for features that can modify component behavior"""
    def __init__(self, name: str):
        self.name = name
        self.parameters = {}
        self.geometry: Optional[Geometry] = None

    @abstractmethod
    def modify_analysis(self, analysis: AnalysisModule, component: 'Component') -> None:
        """Define how this feature modifies an analysis"""
        pass

    @abstractmethod
    def validate(self) -> bool:
        """Validate the feature configuration"""
        pass

class Component:
    """Base class for all aircraft components"""
    def __init__(self, name: str):
        self.name = name
        self.geometry: Optional[Geometry] = None
        self.children: List[Component] = []
        self.features: List[Feature] = []
        self.analyses: Dict[str, AnalysisModule] = {}
        self.parent: Optional[Component] = None

    def add_child(self, component: 'Component') -> None:
        """Add a child component"""
        component.parent = self
        self.children.append(component)

    def remove_child(self, component: 'Component') -> None:
        """Remove a child component"""
        if component in self.children:
            component.parent = None
            self.children.remove(component)

    def add_feature(self, feature: Feature) -> None:
        """Add a feature to the component"""
        self.features.append(feature)

    def remove_feature(self, feature: Feature) -> None:
        """Remove a feature from the component"""
        if feature in self.features:
            self.features.remove(feature)

    def add_analysis(self, analysis: AnalysisModule) -> None:
        """Add an analysis module"""
        self.analyses[analysis.name] = analysis

    def remove_analysis(self, analysis_name: str) -> None:
        """Remove an analysis module"""
        if analysis_name in self.analyses:
            del self.analyses[analysis_name]

    def get_global_position(self) -> np.ndarray:
        """Calculate global position by traversing up through parents"""
        if self.geometry is None:
            return np.zeros(3)
        
        local_pos = np.array([self.geometry.position.x,
                             self.geometry.position.y,
                             self.geometry.position.z])
        
        if self.parent is None:
            return local_pos
        
        # Get parent's global position and orientation
        parent_global_pos = self.parent.get_global_position()
        parent_orientation = self.parent.get_global_orientation()
        
        # Convert angles to radians
        roll = np.radians(parent_orientation.roll)
        pitch = np.radians(parent_orientation.pitch)
        yaw = np.radians(parent_orientation.yaw)
        
        # Create rotation matrices
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])
        
        # Combined rotation matrix (order: yaw, pitch, roll)
        R = Rz @ Ry @ Rx
        
        # Transform local position by parent's rotation and add to parent's position
        rotated_pos = R @ local_pos
        return parent_global_pos + rotated_pos

    def get_global_orientation(self) -> Orientation:
        """Calculate global orientation by combining parent orientations"""
        if self.parent is None or self.geometry is None:
            return self.geometry.orientation if self.geometry else Orientation()
        
        # Get parent's global orientation
        parent_global = self.parent.get_global_orientation()
        local_orient = self.geometry.orientation
        
        # TODO: Implement proper attitude combination (e.g., quaternions)
        # For now, simple addition of angles (this is not correct for general 3D rotations)
        return Orientation(
            parent_global.roll + local_orient.roll,
            parent_global.pitch + local_orient.pitch,
            parent_global.yaw + local_orient.yaw
        )

    def plot(self, color: Optional[str] = None, colors_dict: Optional[Dict[str, str]] = None) -> Object3D:
        """Create a 3D object representation of this component and all its children"""
        obj = Object3D()
        
        # Add this component's geometry if it exists
        if hasattr(self, 'geometry') and self.geometry is not None:
            try:
                component_obj = self.geometry.create_object()
                if component_obj is not None and component_obj.shapes:
                    # Apply global position
                    total_position = self.get_global_position()
                    if np.any(total_position != 0):
                        component_obj.apply_position(total_position)
                    
                    # Set color in metadata if provided
                    component_color = colors_dict.get(self.name, color) if colors_dict else color
                    if component_color:
                        for shape in component_obj.shapes:
                            shape.metadata['color'] = component_color
                    
                    # Add shapes to combined object
                    for shape in component_obj.shapes:
                        obj.add_shape(shape)
            except Exception as e:
                print(f"Warning: Failed to create object for {self.name}: {e}")
        
        # Add all children's objects
        for child in self.children:
            #print(f"Plotting child {child.name} of {self.name} at absolute position {self.get_global_position()}")
            try:
                child_obj = child.plot(color=color, colors_dict=colors_dict)
                if child_obj is not None and child_obj.shapes:
                    for shape in child_obj.shapes:
                        obj.add_shape(shape)
            except Exception as e:
                print(f"Warning: Failed to plot child {child.name}: {e}")
        
        return obj

    def run_analysis(self, analysis_name: str) -> Dict[str, Any]:
        """Run a specific analysis"""
        if analysis_name not in self.analyses:
            raise ValueError(f"No analysis module named {analysis_name}")
        
        analysis = self.analyses[analysis_name]
        
        # Validate analysis parameters
        if not analysis.validate_parameters():
            raise ValueError(f"Analysis {analysis_name} is missing required parameters")
        
        # Apply feature modifications
        for feature in self.features:
            if feature.validate():
                feature.modify_analysis(analysis, self)
        
        # Run the analysis
        results = analysis.run(self)
        return results

    def validate(self) -> bool:
        """Validate the component configuration"""
        if self.geometry and not self.geometry.validate():
            return False
        
        for feature in self.features:
            if not feature.validate():
                return False
        
        for child in self.children:
            if not child.validate():
                return False
        
        return True

    def __repr__(self) -> str:
        return f"Component(name='{self.name}', {len(self.children)} children, {len(self.features)} features)"

