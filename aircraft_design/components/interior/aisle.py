from dataclasses import dataclass
from .base import InteriorComponent, InteriorGeometry

@dataclass
class AisleGeometry(InteriorGeometry):
    """Aisle geometry"""
    def __init__(self):
        super().__init__()
        self.parameters.update({
            'minimum_clearance': 0.0,
        })

class Aisle(InteriorComponent):
    """Aircraft aisle"""
    def __init__(self, name: str, width: float):
        super().__init__(name)
        self.geometry = AisleGeometry()
        self.geometry.parameters['width'] = width 