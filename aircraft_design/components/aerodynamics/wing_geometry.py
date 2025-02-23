from dataclasses import dataclass
from typing import List, Dict, Any, Tuple
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from aircraft_design.core.base import Geometry
from aircraft_design.core.plotting import Object3D, Shape3D, create_wing_section, plot_3d_shape

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
    def __init__(self):
        super().__init__()
        self.waypoints: List[WaypointChord] = []
        # Leading edge sweep is fixed, trailing edge is determined by waypoints
        self.parameters.update({
            'le_sweep': 0.0,  # Leading edge sweep angle in degrees
        })

    def update_parameters(self, params: Dict[str, float]) -> None:
        """Update parameters and regenerate waypoints if needed"""
        # Store old values for comparison
        old_root = self.parameters.get('root_chord', 0.0)
        old_tip = self.parameters.get('tip_chord', 0.0)
        
        # Update parameters
        self.parameters.update(params)
        
        # If root or tip chord changed and we don't have custom waypoints, regenerate default waypoints
        if ((old_root != self.parameters['root_chord'] or old_tip != self.parameters['tip_chord']) and 
            len(self.waypoints) <= 2):
            self.waypoints = []  # Clear existing waypoints
            if self.parameters['root_chord'] > 0 and self.parameters['tip_chord'] > 0:
                self.add_waypoint(0.0, self.parameters['root_chord'], self.parameters['root_chord'] * 0.12)
                self.add_waypoint(1.0, self.parameters['tip_chord'], self.parameters['tip_chord'] * 0.10)

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
            
            x1 = y1 * np.tan(sweep_rad)
            x2 = y2 * np.tan(sweep_rad)
            
            z1 = y1 * np.tan(dihedral_rad)
            z2 = y2 * np.tan(dihedral_rad)
            
            # Create right wing section
            root_pos = np.array([x1, y1, z1])
            tip_pos = np.array([x2, y2, z2])
            right_section = create_wing_section(
                root_pos, tip_pos,
                wp1.chord_length,
                wp2.chord_length,
                thickness_ratio=wp1.thickness/wp1.chord_length
            )
            obj.add_shape(right_section)
            
            # Mirror for left wing
            root_pos[1] = -y1
            tip_pos[1] = -y2
            left_section = create_wing_section(
                root_pos, tip_pos,
                wp1.chord_length,
                wp2.chord_length,
                thickness_ratio=wp1.thickness/wp1.chord_length
            )
            obj.add_shape(left_section)
        
        # Apply position after creating the wing
        obj.position = np.array([self.position.x, self.position.y, self.position.z])
        
        return obj

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
            tip_y = 0
            tip_z = height * np.cos(cant_rad)
            
            root_pos = np.array([self.position.x, self.position.y, self.position.z])
            tip_pos = np.array([tip_x, tip_y, tip_z])
            
            # Create tail section
            tail_section = create_wing_section(
                root_pos, tip_pos,
                self.parameters['root_chord'],
                self.parameters['tip_chord'],
                thickness_ratio=self.parameters['thickness_ratio']
            )
            obj.add_shape(tail_section)
        else:
            # For horizontal tail, use span
            return super().create_object()
        
        # Apply position after creating the wing
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
                thickness_ratio=self.parameters['thickness_ratio']
            )
            plot_3d_shape(ax, vertices, faces, color, alpha)
        else:
            # For horizontal tail, use span
            super().plot(ax, color, alpha)

    def plot_top_view(self, ax: plt.Axes, color: str = 'blue') -> None:
        """Plot top view (X-Y plane)"""
        if 'height' in self.parameters:  # Vertical tail
            height = self.parameters['height']
            sweep_rad = np.radians(self.parameters['sweep'])
            
            # Plot root chord
            root_x = [self.position.x, self.position.x + self.parameters['root_chord']]
            root_y = [self.position.y, self.position.y]
            ax.plot(root_x, root_y, color=color)
            
            # Plot tip chord
            tip_x = self.position.x + height * np.tan(sweep_rad)
            tip_chord_x = [tip_x, tip_x + self.parameters['tip_chord']]
            tip_y = [self.position.y, self.position.y]
            ax.plot(tip_chord_x, tip_y, color=color)
            
            # Plot leading and trailing edges
            ax.plot([root_x[0], tip_x], [root_y[0], tip_y[0]], color=color)
            ax.plot([root_x[1], tip_chord_x[1]], [root_y[1], tip_y[1]], color=color)
        else:  # Horizontal tail
            half_span = self.parameters['span'] / 2
            sweep_rad = np.radians(self.parameters['sweep'])
            
            # Calculate tip positions
            tip_x = self.position.x + half_span * np.tan(sweep_rad)
            
            # Plot root chord
            root_x = [self.position.x, self.position.x + self.parameters['root_chord']]
            root_y = [0, 0]
            ax.plot(root_x, root_y, color=color)
            
            # Plot tip chords
            for side in [-1, 1]:
                tip_y = side * half_span
                tip_chord_x = [tip_x, tip_x + self.parameters['tip_chord']]
                ax.plot(tip_chord_x, [tip_y, tip_y], color=color)
                
                # Plot leading and trailing edges
                ax.plot([root_x[0], tip_x], [0, tip_y], color=color)
                ax.plot([root_x[1], tip_chord_x[1]], [0, tip_y], color=color)

    def plot_side_view(self, ax: plt.Axes, color: str = 'blue') -> None:
        """Plot side view (X-Z plane)"""
        if 'height' in self.parameters:  # Vertical tail
            height = self.parameters['height']
            sweep_rad = np.radians(self.parameters['sweep'])
            cant_rad = np.radians(self.parameters.get('cant_angle', 0.0))
            
            # Calculate tip position
            tip_x = self.position.x + height * np.tan(sweep_rad)
            tip_z = self.position.z + height * np.cos(cant_rad)
            
            # Plot root chord
            root_x = [self.position.x, self.position.x + self.parameters['root_chord']]
            root_z = [self.position.z, self.position.z]
            ax.plot(root_x, root_z, color=color)
            
            # Plot tip chord
            tip_chord_x = [tip_x, tip_x + self.parameters['tip_chord']]
            ax.plot(tip_chord_x, [tip_z, tip_z], color=color)
            
            # Plot leading and trailing edges
            ax.plot([root_x[0], tip_x], [root_z[0], tip_z], color=color)
            ax.plot([root_x[1], tip_chord_x[1]], [root_z[1], tip_z], color=color)
        else:  # Horizontal tail
            sweep_rad = np.radians(self.parameters['sweep'])
            
            # Plot root chord
            root_x = [self.position.x, self.position.x + self.parameters['root_chord']]
            ax.plot(root_x, [self.position.z, self.position.z], color=color)

    def plot_front_view(self, ax: plt.Axes, color: str = 'blue') -> None:
        """Plot front view (Y-Z plane)"""
        if 'height' in self.parameters:  # Vertical tail
            height = self.parameters['height']
            cant_rad = np.radians(self.parameters.get('cant_angle', 0.0))
            
            # Plot vertical line
            ax.plot([0, 0], [self.position.z, self.position.z + height], color=color)
        else:  # Horizontal tail
            half_span = self.parameters['span'] / 2
            
            # Plot horizontal line
            ax.plot([-half_span, half_span], [self.position.z, self.position.z], color=color)

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

