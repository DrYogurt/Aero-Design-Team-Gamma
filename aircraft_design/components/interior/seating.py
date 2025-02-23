from dataclasses import dataclass
from typing import List, Dict, Optional
from .base import InteriorComponent, InteriorGeometry
from .service import Galley, Bathroom
from .aisle import Aisle

@dataclass
class SeatGeometry(InteriorGeometry):
    """Individual seat geometry"""
    def __init__(self):
        super().__init__()
        self.parameters.update({
            'pitch': 0.0,  # Distance between rows
            'recline_angle': 0.0,
            'cushion_thickness': 0.0,
            'armrest_width': 0.0,
        })

class Seat(InteriorComponent):
    """Individual aircraft seat"""
    def __init__(self, name: str, seat_class: str = 'economy'):
        super().__init__(name)
        self.geometry = SeatGeometry()
        self.seat_class = seat_class

class SeatingSection(InteriorComponent):
    """A section of the aircraft with a specific seating configuration"""
    def __init__(self, name: str, section_type: str = 'passenger'):
        super().__init__(name)
        self.section_type = section_type
        self.seat_groups: List[List[Seat]] = []  # Groups of seats separated by aisles
        self.aisles: List[Aisle] = []
        self.service_areas: List[InteriorComponent] = []  # Galleys, bathrooms, etc.
        
    def add_seat_group(self, num_seats: int, seat_class: str = 'economy'):
        """Add a group of seats (e.g., 3 seats together)"""
        seats = [
            Seat(f"{self.name}_seat_{len(self.seat_groups)}_{i}", seat_class)
            for i in range(num_seats)
        ]
        self.seat_groups.append(seats)
        
    def add_aisle(self, width: float, position: int):
        """Add an aisle after the specified seat group position"""
        aisle = Aisle(f"{self.name}_aisle_{len(self.aisles)}", width)
        self.aisles.append(aisle)
        
    def add_service_area(self, service_component: InteriorComponent):
        """Add a service area (galley, bathroom, etc.) to the section"""
        if not isinstance(service_component, (Galley, Bathroom)):
            raise ValueError("Service component must be a Galley or Bathroom")
        self.service_areas.append(service_component)
        self.add_child(service_component)  # Add as child component for proper hierarchy
        
    def get_total_width(self) -> float:
        """Calculate total width including seats and aisles"""
        total = 0
        for group in self.seat_groups:
            total += sum(seat.geometry.parameters['width'] for seat in group)
        total += sum(aisle.geometry.parameters['width'] for aisle in self.aisles)
        return total
        
    def get_total_length(self) -> float:
        """Calculate total length including seats and service areas"""
        seat_length = max(
            (group[0].geometry.parameters['length'] for group in self.seat_groups),
            default=0
        )
        service_length = sum(
            area.geometry.parameters['length'] for area in self.service_areas
        )
        return seat_length + service_length 