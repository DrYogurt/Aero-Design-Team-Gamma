from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import numpy as np

from aircraft_design.core.component import Component
from aircraft_design.core.geometry import Geometry, CompositeGeometry
from aircraft_design.core.base import Position
from aircraft_design.core.analysis import AnalysisModule, AnalysisResult
from aircraft_design.components.interior.floor import Floor, CargoFloor, FloorConfiguration
from aircraft_design.components.interior.seating import SeatSection, SeatConfiguration

class FuselageGeometry(Geometry):
    """Base geometry for fuselage"""
    def __init__(self, name: str):
        super().__init__(name)
        self.cross_sections: List[Tuple[float, float, float]] = []  # [(x, width, height)]
        
    def add_cross_section(self, x: float, width: float, height: float):
        """Add a cross section at position x"""
        self.cross_sections.append((x, width, height))
        self.cross_sections.sort(key=lambda x: x[0])  # Sort by x position
        
    def get_cross_section(self, x: float) -> Tuple[float, float]:
        """Get width and height at position x"""
        if not self.cross_sections:
            return (0, 0)
        
        # Find surrounding cross sections
        for i in range(len(self.cross_sections) - 1):
            x1, w1, h1 = self.cross_sections[i]
            x2, w2, h2 = self.cross_sections[i + 1]
            
            if x1 <= x <= x2:
                # Linear interpolation
                t = (x - x1) / (x2 - x1)
                width = w1 + t * (w2 - w1)
                height = h1 + t * (h2 - h1)
                return (width, height)
        
        # If x is outside range, return nearest cross section
        if x < self.cross_sections[0][0]:
            return (self.cross_sections[0][1], self.cross_sections[0][2])
        return (self.cross_sections[-1][1], self.cross_sections[-1][2])

    def calculate_volume(self) -> float:
        """Calculate volume using trapezoidal integration"""
        volume = 0
        for i in range(len(self.cross_sections) - 1):
            x1, w1, h1 = self.cross_sections[i]
            x2, w2, h2 = self.cross_sections[i + 1]
            
            area1 = w1 * h1
            area2 = w2 * h2
            dx = x2 - x1
            
            volume += (area1 + area2) * dx / 2
        return volume
    
    def calculate_surface_area(self) -> float:
        """Approximate surface area using trapezoidal integration"""
        surface_area = 0
        for i in range(len(self.cross_sections) - 1):
            x1, w1, h1 = self.cross_sections[i]
            x2, w2, h2 = self.cross_sections[i + 1]
            
            # Calculate perimeter at each cross section
            p1 = 2 * (w1 + h1)
            p2 = 2 * (w2 + h2)
            
            dx = x2 - x1
            ds = np.sqrt(dx**2 + (w2-w1)**2 + (h2-h1)**2)
            
            surface_area += (p1 + p2) * ds / 2
        return surface_area

class BasicFuselage(Component):
    """Simple fuselage with constant cross-section"""
    def __init__(self, name: str, length: float, width: float, height: float):
        super().__init__(name)
        self.length = length
        self.width = width
        self.height = height
        
        # Create geometry
        self.geometry = FuselageGeometry(f"{name}_geometry")
        self.geometry.add_cross_section(0, width, height)  # Front
        self.geometry.add_cross_section(length, width, height)  # Back
        
        # Set basic properties
        self.cross_section_area = width * height
        self.surface_area = self.geometry.calculate_surface_area()
        self.volume = self.geometry.calculate_volume()

class WettedAreaAnalysis(AnalysisModule):
    """Calculate wetted area of fuselage"""
    def __init__(self):
        super().__init__("wetted_area")
    
    def run(self, component: Component, **kwargs) -> AnalysisResult:
        if not isinstance(component.geometry, FuselageGeometry):
            return AnalysisResult(
                success=False,
                message="Component must have FuselageGeometry"
            )
        
        wetted_area = component.geometry.calculate_surface_area()
        
        return AnalysisResult(
            success=True,
            data={
                'wetted_area': wetted_area
            }
        )

@dataclass
class FuselageConfiguration:
    """Configuration for flexible fuselage"""
    total_length: float
    max_width: float
    nose_length: float
    tail_length: float
    floor_configs: List[FloorConfiguration]
    nose_taper_ratio: float = 0.3  # How much nose tapers from max width
    tail_taper_ratio: float = 0.2  # How much tail tapers from max width

class FlexFuselage(Component):
    """Flexible fuselage with variable cross-section and internal configuration"""
    def __init__(self, name: str, config: FuselageConfiguration):
        super().__init__(name)
        self.config = config
        
        # Create geometry
        self._create_geometry()
        
        # Add analyses
        self.add_analysis(WettedAreaAnalysis())
        
        # Initialize floors
        self._floors: List[Floor] = []
        self._create_floors()
    
    def _create_geometry(self):
        """Create fuselage geometry with nose and tail sections"""
        self.geometry = FuselageGeometry(f"{self.name}_geometry")
        
        # Calculate heights for each floor
        total_height = sum(fc.ceiling_height for fc in self.config.floor_configs)
        
        # Add cross sections
        # Nose
        self.geometry.add_cross_section(
            0,  # Front
            self.config.max_width * self.config.nose_taper_ratio,
            total_height * self.config.nose_taper_ratio
        )
        self.geometry.add_cross_section(
            self.config.nose_length,  # End of nose
            self.config.max_width,
            total_height
        )
        
        # Main section
        self.geometry.add_cross_section(
            self.config.total_length - self.config.tail_length,  # Start of tail
            self.config.max_width,
            total_height
        )
        
        # Tail
        self.geometry.add_cross_section(
            self.config.total_length,  # Back
            self.config.max_width * self.config.tail_taper_ratio,
            total_height * self.config.tail_taper_ratio
        )
    
    def _create_floors(self):
        """Create floors based on configuration"""
        current_height = 0.0
        main_cabin_length = (self.config.total_length - 
                           self.config.nose_length - 
                           self.config.tail_length)
        
        for i, floor_config in enumerate(self.config.floor_configs):
            # Create appropriate floor type
            if floor_config.is_cargo:
                floor = CargoFloor(
                    f"{self.name}_cargo_{i}",
                    floor_config,
                    main_cabin_length,
                    self.config.max_width
                )
            else:
                floor = Floor(
                    f"{self.name}_floor_{i}",
                    floor_config,
                    main_cabin_length,
                    self.config.max_width
                )
            
            # Position floor after nose section
            floor.reference_frame.position = Position(
                self.config.nose_length,
                0,
                current_height
            )
            
            self.add_child(floor)
            self._floors.append(floor)
            
            # Update height for next floor
            current_height += floor_config.ceiling_height
    
    def get_floor(self, index: int) -> Optional[Floor]:
        """Get floor by index"""
        if 0 <= index < len(self._floors):
            return self._floors[index]
        return None
    
    def get_cargo_volume(self) -> float:
        """Calculate total available cargo volume"""
        return sum(
            floor.usable_volume
            for floor in self._floors
            if isinstance(floor, CargoFloor)
        )
    
    def get_passenger_volume(self) -> float:
        """Calculate total passenger cabin volume"""
        return sum(
            floor.usable_volume
            for floor in self._floors
            if isinstance(floor, Floor) and not floor.config.is_cargo
        )
    
    @property
    def wetted_area(self) -> float:
        """Get wetted area from analysis"""
        result = self.run_analysis("wetted_area")
        return result.data['wetted_area']

