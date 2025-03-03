from dataclasses import dataclass
from typing import List, Dict, Any, Tuple, Optional
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from aircraft_design.core.base import Geometry
from aircraft_design.core.plotting import Object3D, Shape3D, create_wing_section, plot_3d_shape

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
            'twist': 0.0,          # twist angle in degrees (positive = washout),
            'thickness_ratio': 0.12  # thickness to chord ratio
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

    @property
    def mean_chord(self) -> float:
        """Calculate mean aerodynamic chord"""
        return self.area / self.parameters['span'] if self.parameters['span'] > 0 else 0.0

    @property
    def exposed_area(self) -> float:
        """Calculate exposed area (to be overridden by subclasses for more sophisticated calculations)"""
        return self.area

    def get_section_mean_chord(self, root_chord: float, tip_chord: float) -> float:
        """Calculate mean chord for a wing section"""
        return (root_chord + tip_chord) / 2

    def get_section_wetted_area(self, section_area: float, tc_ratio: float) -> float:
        """Calculate wetted area for a wing section based on t/c ratio"""
        if tc_ratio < 0.05:
            return 2.003 * section_area
        else:
            return (1.977 + 0.52 * tc_ratio) * section_area

    @property
    def wetted_area(self) -> float:
        """Calculate wetted area using basic geometry"""
        # Calculate mean chord
        mean_chord = (self.parameters['root_chord'] + self.parameters['tip_chord']) / 2
        
        # Calculate exposed area
        exposed_area = mean_chord * self.parameters['span']
        
        # Get t/c ratio
        tc_ratio = self.parameters['thickness_ratio']
        
        # Apply wetted area formula based on t/c ratio
        if tc_ratio < 0.05:
            return 2.003 * exposed_area
        else:
            return (1.977 + 0.52 * tc_ratio) * exposed_area

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

    def create_object(self) -> Object3D:
        """Create a 3D object representation of this geometry"""
        obj = Object3D()
        
        # Create single wing section from root to tip
        half_span = self.parameters['span'] / 2
        sweep_rad = np.radians(self.parameters['sweep'])
        dihedral_rad = np.radians(self.parameters['dihedral'])
        
        # Calculate tip position considering sweep and dihedral
        tip_x = half_span * np.tan(sweep_rad)
        tip_y = half_span
        tip_z = half_span * np.tan(dihedral_rad)
        
        root_pos = np.array([0, 0, 0])
        tip_pos = np.array([tip_x, tip_y, tip_z])
        
        # Create right wing
        right_wing = create_wing_section(
            root_pos, tip_pos,
            self.parameters['root_chord'],
            self.parameters['tip_chord']
        )
        obj.add_shape(right_wing)
        
        # Mirror for left wing
        tip_pos[1] = -tip_pos[1]  # Negate y coordinate
        left_wing = create_wing_section(
            root_pos, tip_pos,
            self.parameters['root_chord'],
            self.parameters['tip_chord']
        )
        obj.add_shape(left_wing)
        
        # Apply position after creating the wing
        obj.position = np.array([self.position.x, self.position.y, self.position.z])
        
        return obj

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
            raise ValueError(f"Chord length must be positive: {self.chord_length}")
        if self.thickness <= 0:
            raise ValueError(f"Thickness must be positive: {self.thickness}")

class WaypointWingGeometry(AerodynamicGeometry):
    """Wing geometry with constant leading edge defined by waypoints along the span"""
    def __init__(self, waypoints: Optional[List[Dict[str, float]]] = None):
        super().__init__()
        self.waypoints: List[WaypointChord] = []
        # Leading edge sweep is fixed, trailing edge is determined by waypoints
        self.parameters.update({
            'le_sweep': 0.0,  # Leading edge sweep angle in degrees
        })
        
        # Initialize with provided waypoints
        if waypoints:
            for wp in waypoints:
                self.add_waypoint(wp['span_fraction'], wp['chord'], wp['thickness'])

    def update_parameters(self, params: Dict[str, Any]) -> None:
        """Update parameters and regenerate waypoints if needed"""
        # Update parameters
        self.parameters.update(params)

        # If waypoints are provided, clear existing and add new waypoints
        if 'waypoints' in params:
            self.waypoints = []  # Clear existing waypoints
            for wp in params['waypoints']:
                self.add_waypoint(wp['span_fraction'], wp['chord'], wp['thickness'])

    def add_waypoint(self, span_fraction: float, chord: float, thickness: float) -> None:
        """Add a new waypoint at a specified span fraction"""
        if chord <= 0:
            raise ValueError(f"Chord length must be positive: {chord}")
        if thickness <= 0:
            raise ValueError(f"Thickness must be positive: {thickness}")
            
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
        """Calculate planform area using trapezoidal integration of waypoints"""
        if len(self.waypoints) < 2:
            # If no waypoints defined, use basic geometry
            return super().area
        
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

    def create_object(self) -> Object3D:
        """Create a 3D object representation using waypoints"""
        obj = Object3D()
        
        if len(self.waypoints) < 2:
            # Fall back to basic wing if no waypoints
            return super().create_object()
            
        half_span = self.parameters['span'] / 2
        sweep_rad = np.radians(self.parameters['sweep'])
        dihedral_rad = np.radians(self.parameters['dihedral'])
        
        # Create wing sections between each pair of waypoints
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            
            # Calculate positions for this section
            y1 = wp1.span_location * half_span
            y2 = wp2.span_location * half_span
            
            # Calculate x positions based on leading edge sweep
            x1 = y1 * np.tan(sweep_rad)
            x2 = y2 * np.tan(sweep_rad)
            
            # Calculate z positions based on dihedral
            z1 = y1 * np.tan(dihedral_rad)
            z2 = y2 * np.tan(dihedral_rad)
            
            # Create right wing section
            root_pos = np.array([x1, y1, z1])
            tip_pos = np.array([x2, y2, z2])
            right_section = create_wing_section(
                root_pos, tip_pos,
                wp1.chord_length,
                wp2.chord_length,
                thickness_ratio_root=wp1.thickness/wp1.chord_length,
                thickness_ratio_tip=wp2.thickness/wp2.chord_length
            )
            obj.add_shape(right_section)
            
            # Mirror for left wing
            root_pos[1] = -y1
            tip_pos[1] = -y2
            left_section = create_wing_section(
                root_pos, tip_pos,
                wp1.chord_length,
                wp2.chord_length,
                thickness_ratio_root=wp1.thickness/wp1.chord_length,
                thickness_ratio_tip=wp2.thickness/wp2.chord_length
            )
            obj.add_shape(left_section)
        
        # Apply position after creating the wing
        obj.position = np.array([self.position.x, self.position.y, self.position.z])
        
        return obj

    @property
    def volume(self) -> float:
        """Calculate the total volume of the wing by integrating over sections"""
        return self._calculate_volume()
    
    def _calculate_volume(self) -> float:
        """Calculate the total volume of the wing by integrating over sections
        
        Returns:
            float: Total wing volume in cubic feet
        """
        if len(self.waypoints) < 2:
            return 0.0
            
        total_volume = 0.0
        half_span = self.parameters['span'] / 2
        
        # Integrate volume between each pair of waypoints
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            
            # Calculate section span
            dy = (wp2.span_location - wp1.span_location) * half_span
            
            # Calculate average cross-sectional area using trapezoidal rule
            area1 = wp1.chord_length * wp1.thickness
            area2 = wp2.chord_length * wp2.thickness
            avg_area = (area1 + area2) / 2
            
            # Calculate volume of this section
            section_volume = avg_area * dy
            
            # Multiply by 2 to account for both wings
            total_volume += 2 * section_volume
            
        return total_volume

    @property
    def wetted_area(self) -> float:
        """Calculate total wetted area for the wing using waypoints"""
        if len(self.waypoints) < 2:
            return super().wetted_area
            
        total_wetted_area = 0.0
        half_span = self.parameters['span'] / 2
        
        # Calculate wetted area for each section
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            
            # Calculate section span
            dy = (wp2.span_location - wp1.span_location) * half_span
            
            # Calculate mean chord for this section
            mean_chord = (wp1.chord_length + wp2.chord_length) / 2
            
            # Calculate section area
            section_area = mean_chord * dy
            
            # Calculate mean t/c ratio for this section
            mean_tc = ((wp1.thickness / wp1.chord_length) + 
                      (wp2.thickness / wp2.chord_length)) / 2
            
            # Calculate wetted area for this section
            section_wetted_area = self.get_section_wetted_area(section_area, mean_tc)
            total_wetted_area += section_wetted_area
            
        # Multiply by 2 to account for both wings
        return 2.0 * total_wetted_area

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
            'thickness_ratio': 0.12  # Maximum thickness as fraction of chord
        })

    @property
    def mean_chord(self) -> float:
        """Calculate mean aerodynamic chord for vertical tail"""
        root_chord = self.parameters['root_chord']
        tip_chord = self.parameters['tip_chord']
        return (root_chord + tip_chord) / 2

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

    def create_object(self) -> Object3D:
        """Create a 3D object representation of the tail"""
        obj = Object3D()
        
        # For vertical tail, use height as span
        if 'height' in self.parameters:
            height = self.parameters['height']
            sweep_rad = np.radians(self.parameters['sweep'])
            cant_rad = np.radians(self.parameters.get('cant_angle', 0.0))
            
            # Calculate tip position considering sweep and cant angle
            tip_x = height * np.tan(sweep_rad)
            # Add y-offset based on cant angle
            tip_y = height * np.sin(cant_rad)  # Changed: Add y-component from cant
            tip_z = height * np.cos(cant_rad)
            
            root_pos = np.array([0, 0, 0])  # Changed: Start at origin
            tip_pos = np.array([tip_x, tip_y, tip_z])
            
            # Create tail section
            tail_section = create_wing_section(
                root_pos, tip_pos,
                self.parameters['root_chord'],
                self.parameters['tip_chord'],
                thickness_ratio_root=self.parameters['thickness_ratio'],
                thickness_ratio_tip=self.parameters['thickness_ratio']
            )
            obj.add_shape(tail_section)
        else:
            # For horizontal tail, use span
            return super().create_object()
        
        # Apply position after creating the tail
        obj.position = np.array([self.position.x, self.position.y, self.position.z])
        
        return obj

    def plot(self, ax: Axes3D, color: str = 'blue', alpha: float = 0.5) -> None:
        """Plot tail geometry"""
        # For vertical tail, use height as span
        if 'height' in self.parameters:
            height = self.parameters['height']
            sweep_rad = np.radians(self.parameters['sweep'])
            cant_rad = np.radians(self.parameters.get('cant_angle', 0.0))
            
            # Calculate tip position considering sweep and cant angle
            tip_x = self.position.x + height * np.tan(sweep_rad)
            tip_y = self.position.y
            tip_z = self.position.z + height * np.cos(cant_rad)
            
            root_pos = np.array([self.position.x, self.position.y, self.position.z])
            tip_pos = np.array([tip_x, tip_y, tip_z])
            
            # Create and plot tail section
            vertices, faces = create_wing_section(
                root_pos, tip_pos,
                self.parameters['root_chord'],
                self.parameters['tip_chord'],
                thickness_ratio_root=self.parameters['thickness_ratio'],
                thickness_ratio_tip=.12 # TODO: Make this dynamic
            )
            plot_3d_shape(ax, vertices, faces, color, alpha)
        else:
            # For horizontal tail, use span
            super().plot(ax, color, alpha)

class SimpleSweptWing(WaypointWingGeometry):
    """Simple swept wing with only root and tip chord"""
    def __init__(self, root_chord: float, tip_chord: float, span: float, sweep: float = 0.0, dihedral: float = 0.0):
        waypoints = [
            {'span_fraction': 0.0, 'chord': root_chord, 'thickness': root_chord * 0.12},
            {'span_fraction': 1.0, 'chord': tip_chord, 'thickness': tip_chord * 0.10}
        ]
        super().__init__(waypoints)
        self.parameters.update({
            'span': span,
            'sweep': sweep,
            'dihedral': dihedral
        })

class TrailingEdgeWingGeometry(AerodynamicGeometry):
    """Wing geometry with constant trailing edge sweep angle defined by waypoints along the span"""
    def __init__(self, waypoints: Optional[List[Dict[str, float]]] = None):
        super().__init__()
        self.waypoints: List[WaypointChord] = []
        # Trailing edge sweep is fixed, leading edge is determined by waypoints
        self.parameters.update({
            'te_sweep': 0.0,  # Trailing edge sweep angle in degrees
        })
        
        # Initialize with provided waypoints
        if waypoints:
            for wp in waypoints:
                self.add_waypoint(wp['span_fraction'], wp['chord'], wp['thickness'])

    def update_parameters(self, params: Dict[str, Any]) -> None:
        """Update parameters and regenerate waypoints if needed"""
        # Update parameters
        self.parameters.update(params)
        
        # If waypoints are provided, clear existing and add new waypoints
        if 'waypoints' in params:
            self.waypoints = []  # Clear existing waypoints
            for wp in params['waypoints']:
                self.add_waypoint(wp['span_fraction'], wp['chord'], wp['thickness'])

    def add_waypoint(self, span_fraction: float, chord: float, thickness: float) -> None:
        """Add a new waypoint at a specified span fraction"""
        if chord <= 0:
            raise ValueError(f"Chord length must be positive: {chord}")
        if thickness <= 0:
            raise ValueError(f"Thickness must be positive: {thickness}")
            
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

    def create_object(self) -> Object3D:
        """Create a 3D object representation using waypoints with fixed trailing edge sweep"""
        obj = Object3D()
        
        if len(self.waypoints) < 2:
            # Fall back to basic wing if no waypoints
            return super().create_object()
            
        half_span = self.parameters['span'] / 2
        te_sweep_rad = np.radians(self.parameters['te_sweep'])
        dihedral_rad = np.radians(self.parameters['dihedral'])
        
        # Find the root chord for reference
        root_chord = self.waypoints[0].chord_length
        
        # Create wing sections between each pair of waypoints
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            
            # Calculate spanwise positions
            y1 = wp1.span_location * half_span
            y2 = wp2.span_location * half_span
            
            # Calculate trailing edge positions relative to root trailing edge
            x1_te = y1 * np.tan(te_sweep_rad)
            x2_te = y2 * np.tan(te_sweep_rad)
            
            # Calculate leading edge positions
            # Root leading edge starts at x=0, trailing edge at x=root_chord
            x1_le = x1_te - wp1.chord_length + root_chord
            x2_le = x2_te - wp2.chord_length + root_chord
            
            # Calculate z positions based on dihedral
            z1 = y1 * np.tan(dihedral_rad)
            z2 = y2 * np.tan(dihedral_rad)
            
            # Create right wing section
            root_pos = np.array([x1_le, y1, z1])
            tip_pos = np.array([x2_le, y2, z2])
            right_section = create_wing_section(
                root_pos, tip_pos,
                wp1.chord_length,
                wp2.chord_length,
                thickness_ratio_root=wp1.thickness/wp1.chord_length,
                thickness_ratio_tip=wp2.thickness/wp2.chord_length
            )
            obj.add_shape(right_section)
            
            # Mirror for left wing
            root_pos[1] = -y1
            tip_pos[1] = -y2
            left_section = create_wing_section(
                root_pos, tip_pos,
                wp1.chord_length,
                wp2.chord_length,
                thickness_ratio_root=wp1.thickness/wp1.chord_length,
                thickness_ratio_tip=wp2.thickness/wp2.chord_length
            )
            obj.add_shape(left_section)
        
        # Apply position after creating the wing
        obj.position = np.array([self.position.x, self.position.y, self.position.z])
        
        return obj

    @property
    def area(self) -> float:
        """Calculate planform area using trapezoidal integration of waypoints"""
        if len(self.waypoints) < 2:
            return super().area
        
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

    @property
    def volume(self) -> float:
        """Calculate the total volume of the wing by integrating over sections"""
        return self._calculate_volume()
    
    def _calculate_volume(self) -> float:
        """Calculate the total volume of the wing by integrating over sections
        
        Returns:
            float: Total wing volume in cubic feet
        """
        if len(self.waypoints) < 2:
            return 0.0
            
        total_volume = 0.0
        half_span = self.parameters['span'] / 2
        
        # Integrate volume between each pair of waypoints
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            
            # Calculate section span
            dy = (wp2.span_location - wp1.span_location) * half_span
            
            # Calculate average cross-sectional area using trapezoidal rule
            area1 = wp1.chord_length * wp1.thickness
            area2 = wp2.chord_length * wp2.thickness
            avg_area = (area1 + area2) / 2
            
            # Calculate volume of this section
            section_volume = avg_area * dy
            
            # Multiply by 2 to account for both wings
            total_volume += 2 * section_volume
            
        return total_volume

    @property
    def mean_chord(self) -> float:
        """Calculate mean aerodynamic chord for the entire wing"""
        if len(self.waypoints) < 2:
            return super().mean_chord
            
        total_area = 0.0
        total_chord_integral = 0.0
        span = self.parameters['span']
        
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            
            # Calculate section span
            dy = (wp2.span_location - wp1.span_location) * span
            
            # Calculate average chord for this section
            avg_chord = self.get_section_mean_chord(wp1.chord_length, wp2.chord_length)
            
            # Add to integrals
            section_area = avg_chord * dy
            total_area += section_area
            total_chord_integral += section_area * avg_chord
            
        return total_chord_integral / total_area if total_area > 0 else 0.0

    @property
    def exposed_area(self) -> float:
        """Calculate exposed wing area (currently same as planform area, to be updated)"""
        return self.area

    def get_section_wetted_area(self, wp1: WaypointChord, wp2: WaypointChord) -> float:
        """Calculate wetted area for a specific wing section based on t/c ratio"""
        # Calculate mean values for this section
        mean_chord = self.get_section_mean_chord(wp1.chord_length, wp2.chord_length)
        mean_thickness = (wp1.thickness + wp2.thickness) / 2
        
        # Calculate t/c ratio
        tc_ratio = mean_thickness / mean_chord if mean_chord > 0 else 0
        
        # Calculate section span
        dy = (wp2.span_location - wp1.span_location) * self.parameters['span']
        
        # Calculate section exposed area
        section_exposed_area = mean_chord * dy
        
        # Apply appropriate wetted area formula based on t/c ratio
        if tc_ratio < 0.05:
            return 2.003 * section_exposed_area
        else:
            return (1.977 + 0.52 * tc_ratio) * section_exposed_area

# Example usage
if __name__ == "__main__":
    

        
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

