from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, List, Any, Optional

@dataclass
class Position:
    """Position relative to parent component in meters"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

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

    def get_global_position(self) -> Position:
        """Calculate global position by traversing up through parents"""
        if self.parent is None or self.geometry is None:
            return self.geometry.position if self.geometry else Position()
        
        # Get parent's global position
        parent_global = self.parent.get_global_position()
        local_pos = self.geometry.position
        
        # TODO: Apply proper orientation transformations
        # For now, simple addition of coordinates
        return Position(
            parent_global.x + local_pos.x,
            parent_global.y + local_pos.y,
            parent_global.z + local_pos.z
        )

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

