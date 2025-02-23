from dataclasses import dataclass
from typing import List, Optional
from .base import InteriorComponent, InteriorGeometry

@dataclass
class GalleyGeometry(InteriorGeometry):
    """Galley geometry including equipment space"""
    def __init__(self):
        super().__init__()
        self.parameters.update({
            'cart_capacity': 0,
            'counter_space': 0.0,
            'storage_volume': 0.0,
        })

@dataclass
class BathroomGeometry(InteriorGeometry):
    """Bathroom geometry"""
    def __init__(self):
        super().__init__()
        self.parameters.update({
            'sink_space': 0.0,
            'waste_capacity': 0.0,
        })

class Galley(InteriorComponent):
    """Aircraft galley"""
    def __init__(self, name: str):
        super().__init__(name)
        self.geometry = GalleyGeometry()
        self.equipment: List[str] = []
        
    def add_equipment(self, equipment_type: str):
        """Add galley equipment (ovens, coffee makers, etc.)"""
        self.equipment.append(equipment_type)

class Bathroom(InteriorComponent):
    """Aircraft bathroom"""
    def __init__(self, name: str):
        super().__init__(name)
        self.geometry = BathroomGeometry() 