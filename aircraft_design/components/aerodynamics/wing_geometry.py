from dataclasses import dataclass
from typing import List, Dict, Any, Tuple
import numpy as np

from aircraft_design.core.base import Geometry
@dataclass
class AerodynamicGeometry(Geometry):
    """Basic geometry for aerodynamic surfaces"""
    def __init__(self):
        super().__init__()
        # Basic geometric parameters
        self.parameters = {
            'span': 0.0,           # wingspan in ft
            'root_chord': 0.0,     # root chord length in ft
            'tip_chord': 0.0,      # tip chord length in ft
            'sweep': 0.0,          # sweep angle in degrees (at quarter chord)
            'dihedral': 0.0,       # dihedral angle in degrees
            'twist': 0.0,          # twist angle in degrees (positive = washout)
        }

    @property
    def area(self) -> float:
        """Calculate planform area"""
        return (self.parameters['root_chord'] + self.parameters['tip_chord']) * self.parameters['span'] / 2

    @property
    def aspect_ratio(self) -> float:
        """Calculate aspect ratio"""
        return self.parameters['span']**2 / self.area if self.area > 0 else 0.0

    @property
    def taper_ratio(self) -> float:
        """Calculate taper ratio"""
        return (self.parameters['tip_chord'] / self.parameters['root_chord'] 
                if self.parameters['root_chord'] > 0 else 0.0)

    def validate(self) -> bool:
        """Validate geometric parameters"""
        # Check for positive values where required
        for param in ['span', 'root_chord', 'tip_chord']:
            if self.parameters[param] <= 0:
                return False
        
        # Check for reasonable angle ranges
        for param in ['sweep', 'dihedral', 'twist']:
            if abs(self.parameters[param]) > 90:  # degrees
                return False
        
        return True



@dataclass
class WaypointChord:
    """Represents a chord at a specific spanwise location"""
    span_location: float    # Fraction of span (0 to 1)
    chord_length: float     # Length in ft
    thickness: float        # Thickness in ft
    
    def __post_init__(self):
        if not 0 <= self.span_location <= 1:
            raise ValueError("Span location must be between 0 and 1")
        if self.chord_length <= 0:
            raise ValueError("Chord length must be positive")
        if self.thickness <= 0:
            raise ValueError("Thickness must be positive")

class WaypointWingGeometry(AerodynamicGeometry):
    """Wing geometry with constant leading edge defined by waypoints along the span"""
    def __init__(self):
        super().__init__()
        self.waypoints: List[WaypointChord] = []
        # Leading edge sweep is fixed, trailing edge is determined by waypoints
        self.parameters.update({
            'le_sweep': 0.0,  # Leading edge sweep angle in degrees
        })
        
    def add_waypoint(self, span_fraction: float, chord: float, thickness: float) -> None:
        """Add a new waypoint at a specified span fraction"""
        new_waypoint = WaypointChord(span_fraction, chord, thickness)
        
        # Insert maintaining span_location order
        insert_idx = 0
        for i, wp in enumerate(self.waypoints):
            if wp.span_location > span_fraction:
                insert_idx = i
                break
            insert_idx = i + 1
        
        self.waypoints.insert(insert_idx, new_waypoint)

    def get_chord_at_span(self, span_fraction: float) -> Tuple[float, float]:
        """Get interpolated chord and thickness at any span fraction"""
        if not self.waypoints:
            return 0.0, 0.0
        
        # Handle endpoints
        if span_fraction <= self.waypoints[0].span_location:
            return self.waypoints[0].chord_length, self.waypoints[0].thickness
        if span_fraction >= self.waypoints[-1].span_location:
            return self.waypoints[-1].chord_length, self.waypoints[-1].thickness
        
        # Find surrounding waypoints
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            if wp1.span_location <= span_fraction <= wp2.span_location:
                # Linear interpolation
                t = ((span_fraction - wp1.span_location) / 
                     (wp2.span_location - wp1.span_location))
                chord = wp1.chord_length + t * (wp2.chord_length - wp1.chord_length)
                thickness = wp1.thickness + t * (wp2.thickness - wp1.thickness)
                return chord, thickness
        
        return 0.0, 0.0

    @property
    def area(self) -> float:
        """Calculate planform area using trapezoidal integration"""
        if len(self.waypoints) < 2:
            return 0.0
        
        area = 0.0
        span = self.parameters['span']
        
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            dx = (wp2.span_location - wp1.span_location) * span
            avg_chord = (wp1.chord_length + wp2.chord_length) / 2
            area += dx * avg_chord
            
        return area

    def validate(self) -> bool:
        """Validate the waypoint geometry"""
        if not super().validate():
            return False
            
        if not self.waypoints:
            return False
            
        # Check waypoints are properly ordered
        for i in range(len(self.waypoints) - 1):
            if self.waypoints[i].span_location >= self.waypoints[i + 1].span_location:
                return False
                
        # Check first and last waypoints
        if self.waypoints[0].span_location != 0.0:
            return False
        if self.waypoints[-1].span_location != 1.0:
            return False
            
        return True

class TailGeometry(AerodynamicGeometry):
    """Geometry for vertical or angled tail surfaces"""
    def __init__(self):
        super().__init__()
        self.parameters.update({
            'cant_angle': 0.0,     # Angle from vertical in degrees (0 = vertical)
            'height': 0.0,         # Height of tail in meters (replaces span)
            'root_chord': 0.0,     # Root chord length in meters
            'tip_chord': 0.0,      # Tip chord length in meters
            'sweep': 0.0,          # Quarter-chord sweep angle in degrees
            'thickness_ratio': 0.0  # Maximum thickness as fraction of chord
        })

    @property
    def area(self) -> float:
        """Calculate tail area"""
        return (self.parameters['root_chord'] + self.parameters['tip_chord']) * \
               self.parameters['height'] / 2

    @property
    def aspect_ratio(self) -> float:
        """Calculate aspect ratio using height instead of span"""
        return self.parameters['height']**2 / self.area if self.area > 0 else 0.0

    def get_effective_height(self) -> float:
        """Calculate effective vertical height considering cant angle"""
        cant_rad = np.radians(self.parameters['cant_angle'])
        return self.parameters['height'] * np.cos(cant_rad)

    def validate(self) -> bool:
        """Validate tail geometry parameters"""
        if not super().validate():
            return False
            
        # Check for positive values
        for param in ['height', 'root_chord', 'tip_chord', 'thickness_ratio']:
            if self.parameters[param] <= 0:
                return False
                
        # Check angle ranges
        if abs(self.parameters['cant_angle']) > 90:
            return False
            
        # Check thickness ratio is reasonable
        if not 0.02 <= self.parameters['thickness_ratio'] <= 0.20:
            return False
            
        return True

# Example usage
if __name__ == "__main__":
    
    from basic_aero import AerodynamicComponent
        
    # Create a multi-section wing
    wing_geom = WaypointWingGeometry()
    wing_geom.parameters['span'] = 30.0  # 30m wingspan
    wing_geom.parameters['le_sweep'] = 25.0  # 25 degrees sweep
    
    # Add waypoints from root to tip
    wing_geom.add_waypoint(0.0, 5.0, 0.6)    # Root
    wing_geom.add_waypoint(0.3, 4.0, 0.5)    # Inboard
    wing_geom.add_waypoint(0.7, 3.0, 0.4)    # Outboard
    wing_geom.add_waypoint(1.0, 2.0, 0.3)    # Tip
    
    
    main_wing = AerodynamicComponent("main_wing")

    lift_analysis = main_wing.analyses['basic_lift']
    lift_analysis.parameters.update({
        'alpha': 5.0,       # 5 degrees angle of attack
        'mach': 0.3,        # Mach 0.3
    })
    
    # Run analysis
    results = main_wing.run_analysis('basic_lift')
    print("\nMain Wing Analysis Results:")
    for key, value in results.items():
        print(f"{key}: {value}")
    # Create a vertical tail
    tail_geom = TailGeometry()
    tail_geom.parameters.update({
        'height': 6.0,
        'root_chord': 4.0,
        'tip_chord': 2.0,
        'sweep': 35.0,
        'cant_angle': 15.0,
        'thickness_ratio': 0.12
    })
    tail_wing = AerodynamicComponent("tail_wing")

    lift_analysis = tail_wing.analyses['basic_lift']
    lift_analysis.parameters.update({
        'alpha': 5.0,       # 5 degrees angle of attack
        'mach': 0.3,        # Mach 0.3
    })
    
    # Run analysis
    results = tail_wing.run_analysis('basic_lift')
    print("\nTail Wing Analysis Results:")
    for key, value in results.items():
        print(f"{key}: {value}")
    # Create a vertical tail

