from dataclasses import dataclass
from typing import List, Optional
from .base import InteriorComponent, InteriorGeometry

@dataclass
class FloorGeometry(InteriorGeometry):
    """Floor geometry including structural properties"""
    def __init__(self):
        super().__init__()
        self.parameters.update({
            'thickness': 0.0,
            'load_capacity': 0.0,  # kg/m^2
            'floor_beam_spacing': 0.0,
        })

class Floor(InteriorComponent):
    """Represents an aircraft floor (passenger or cargo)"""
    def __init__(self, name: str, floor_type: str = 'passenger'):
        super().__init__(name)
        self.geometry = FloorGeometry()
        self.floor_type = floor_type
        self.sections: List['SeatinSection'] = []
        
    def add_section(self, section: 'SeatingSection'):
        """Add a seating or cargo section to the floor"""
        self.sections.append(section) 