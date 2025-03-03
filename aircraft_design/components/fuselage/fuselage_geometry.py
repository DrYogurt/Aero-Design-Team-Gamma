from dataclasses import dataclass, field
from enum import Enum
from typing import List, Dict, Any, Optional
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from aircraft_design.core.base import Component, Geometry
from aircraft_design.core.plotting import Object3D, Shape3D, create_cylinder, create_ellipsoid, plot_3d_shape
from aircraft_design.core.plotting import plot_orthographic_views

class CrossSectionShape(Enum):
    CIRCULAR = "circular"
    RECTANGULAR = "rectangular"
    SUPER_ELLIPSE = "super_ellipse"
    CUSTOM = "custom"

@dataclass
class CrossSection:
    """Represents a fuselage cross-section at a specific station"""
    station: float  # Distance from nose in meters
    width: float   # Maximum width in meters
    height: float  # Maximum height in meters
    shape: CrossSectionShape = CrossSectionShape.CIRCULAR
    z_offset: float = 0.0  # Vertical offset from centerline in meters
    parameters: Dict[str, Any] = field(default_factory=dict)

    def calculate_area(self) -> float:
        """Calculate cross-sectional area"""
        if self.shape == CrossSectionShape.CIRCULAR:
            radius = max(self.width, self.height) / 2
            return np.pi * radius**2
        elif self.shape == CrossSectionShape.RECTANGULAR:
            return self.width * self.height
        elif self.shape == CrossSectionShape.SUPER_ELLIPSE:
            # Super-ellipse with parameter n
            n = self.parameters.get('n', 2.0)
            return self.width * self.height * self._super_ellipse_area_factor(n)
        else:
            raise NotImplementedError(f"Area calculation not implemented for {self.shape}")

    def calculate_perimeter(self) -> float:
        """Calculate cross-section perimeter"""
        if self.shape == CrossSectionShape.CIRCULAR:
            radius = max(self.width, self.height) / 2
            return 2 * np.pi * radius
        elif self.shape == CrossSectionShape.RECTANGULAR:
            return 2 * (self.width + self.height)
        elif self.shape == CrossSectionShape.SUPER_ELLIPSE:
            # Approximate perimeter for super-ellipse
            n = self.parameters.get('n', 2.0)
            return self._super_ellipse_perimeter_approx(n)
        else:
            raise NotImplementedError(f"Perimeter calculation not implemented for {self.shape}")

    def _super_ellipse_area_factor(self, n: float) -> float:
        """Calculate area factor for super-ellipse of order n"""
        # This is an approximation
        return 4 * self._gamma_func(1 + 1/n)**2 / (n * self._gamma_func(2/n))

    def _super_ellipse_perimeter_approx(self, n: float) -> float:
        """Approximate perimeter for super-ellipse"""
        # This is a rough approximation
        a = self.width / 2
        b = self.height / 2
        return 2 * np.pi * np.sqrt((a**2 + b**2) / 2)

    def _gamma_func(self, x: float) -> float:
        """Simple gamma function approximation"""
        return np.exp(np.math.lgamma(x))

class FuselageGeometry(Geometry):
    """Geometry definition for fuselage using cross-sections"""
    def __init__(self):
        super().__init__()
        self.sections: List[CrossSection] = []
        self.parameters.update({
            'length': 0.0,  # Total length in meters
            'max_width': 0.0,  # Maximum width in meters
            'max_height': 0.0,  # Maximum height in meters
            'nose_fineness': 2.0,  # Length/diameter ratio for nose
            'tail_fineness': 3.0,  # Length/diameter ratio for tail
        })

    @property
    def wetted_area(self) -> float:
        """Calculate the wetted surface area of the fuselage"""
        return self.calculate_wetted_area()

    @property
    def volume(self) -> float:
        """Calculate the total volume of the fuselage"""
        return self.calculate_volume()

    def add_section(self, section: CrossSection) -> None:
        """Add a cross-section, maintaining station order"""
        insert_idx = 0
        for i, existing in enumerate(self.sections):
            if existing.station > section.station:
                insert_idx = i
                break
            insert_idx = i + 1
        self.sections.insert(insert_idx, section)
        
        # Update maximum dimensions
        self.parameters['max_width'] = max(self.parameters['max_width'], section.width)
        self.parameters['max_height'] = max(self.parameters['max_height'], section.height)
        self.parameters['length'] = max(self.parameters['length'], section.station)

    def get_section(self, station: float) -> Optional[CrossSection]:
        """Get cross-section at exact station if it exists"""
        for section in self.sections:
            if abs(section.station - station) < 1e-6:
                return section
        return None

    def interpolate_section(self, station: float) -> CrossSection:
        """Interpolate cross-section at any station"""
        if not self.sections:
            raise ValueError("No sections defined")
            
        # Handle stations outside range
        if station <= self.sections[0].station:
            return self.sections[0]
        if station >= self.sections[-1].station:
            return self.sections[-1]
            
        # Find surrounding sections
        for i in range(len(self.sections) - 1):
            s1 = self.sections[i]
            s2 = self.sections[i + 1]
            if s1.station <= station <= s2.station:
                # Linear interpolation
                t = (station - s1.station) / (s2.station - s1.station)
                width = s1.width + t * (s2.width - s1.width)
                height = s1.height + t * (s2.height - s1.height)
                return CrossSection(station, width, height, s1.shape)
                
        raise ValueError(f"Could not interpolate section at station {station}")

    def calculate_volume(self) -> float:
        """Calculate total volume using trapezoidal integration"""
        if len(self.sections) < 2:
            return 0.0
            
        volume = 0.0
        for i in range(len(self.sections) - 1):
            s1 = self.sections[i]
            s2 = self.sections[i + 1]
            dx = s2.station - s1.station
            avg_area = (s1.calculate_area() + s2.calculate_area()) / 2
            volume += dx * avg_area
            
        return volume

    def calculate_wetted_area(self) -> float:
        """Calculate approximate wetted surface area"""
        if len(self.sections) < 2:
            return 0.0
            
        area = 0.0
        for i in range(len(self.sections) - 1):
            s1 = self.sections[i]
            s2 = self.sections[i + 1]
            dx = s2.station - s1.station
            avg_perimeter = (s1.calculate_perimeter() + s2.calculate_perimeter()) / 2
            area += dx * avg_perimeter
            
        return area

    def validate(self) -> bool:
        """Validate fuselage geometry"""
        if not self.sections:
            return False
            
        # Check sections are properly ordered
        for i in range(len(self.sections) - 1):
            if self.sections[i].station >= self.sections[i + 1].station:
                return False
                
        # Check dimensions are positive
        if any(param <= 0 for param in [
            self.parameters['length'],
            self.parameters['max_width'],
            self.parameters['max_height']
        ]):
            return False
            
        return True

    def create_object(self) -> Object3D:
        """Create a 3D object representation of the fuselage"""
        obj = Object3D()
        
        if len(self.sections) < 2:
            return obj
            
        num_points = 32  # Number of points around each cross-section
        theta = np.linspace(0, 2*np.pi, num_points)
        
        # Create points for each cross-section
        all_vertices = []
        for section in self.sections:
            # Calculate points for this cross-section
            if section.shape == CrossSectionShape.CIRCULAR:
                radius = max(section.width, section.height) / 2
                y = radius * np.cos(theta)  # y is right
                z = radius * np.sin(theta) + section.z_offset  # z is up, add offset
            else:  # Super-ellipse or rectangular
                # Use super-ellipse equation for smooth transition
                n = section.parameters.get('n', 2.5)  # Higher n makes it more rectangular
                y = (section.width/2) * np.sign(np.cos(theta)) * np.abs(np.cos(theta))**(2/n)
                z = (section.height/2) * np.sign(np.sin(theta)) * np.abs(np.sin(theta))**(2/n) + section.z_offset
            
            # Create points for this cross-section
            x = np.zeros_like(y) + section.station  # x is forward
            section_points = np.column_stack([x, y, z])  # Aircraft coordinates: x forward, y right, z up
            all_vertices.append(section_points)
            
        # Convert to numpy array for easier manipulation
        all_vertices = np.array(all_vertices)
        
        # Create faces between adjacent cross-sections
        faces = []
        for i in range(len(self.sections) - 1):
            for j in range(num_points):
                next_j = (j + 1) % num_points
                # Define vertices for this quad face
                v1 = i * num_points + j
                v2 = i * num_points + next_j
                v3 = (i + 1) * num_points + next_j
                v4 = (i + 1) * num_points + j
                faces.append([v1, v2, v3, v4])
        
        # Flatten vertices
        vertices = all_vertices.reshape(-1, 3)
        
        # Create nose cone if needed
        if self.sections[0].station > 0:
            # Add a point at the nose
            nose_point = np.array([[0, 0, 0]])
            vertices = np.vstack([nose_point, vertices])
            # Create triangular faces for nose cone
            nose_vertex = 0
            for i in range(num_points):
                next_i = (i + 1) % num_points
                faces.append([nose_vertex, i + 1, next_i + 1])
        
        # Create tail cone if needed
        last_section = self.sections[-1]
        if last_section.station < self.parameters['length']:
            # Add a point at the tail
            tail_point = np.array([[self.parameters['length'], 0, 0]])
            vertices = np.vstack([vertices, tail_point])
            # Create triangular faces for tail cone
            tail_vertex = len(vertices) - 1
            last_section_start = (len(self.sections) - 1) * num_points
            for i in range(num_points):
                next_i = (i + 1) % num_points
                faces.append([tail_vertex, last_section_start + i, last_section_start + next_i])
        
        # Create the shape
        shape = Shape3D(vertices=vertices, faces=faces)
        obj.add_shape(shape)
        
        # Apply position
        obj.position = np.array([self.position.x, self.position.y, self.position.z])
        
        return obj

# Example usage
if __name__ == "__main__":
    geom = FuselageGeometry()
    
    # Add cross-sections with varying vertical positions and smooth transitions
    geom.add_section(CrossSection(0.0, 0.2, 0.2))   # Nose (lower)
    geom.add_section(CrossSection(10.0, 2.0, 2.0, z_offset=0.2))   # Start of cockpit
    geom.add_section(CrossSection(35.0, 20.0, 20.0, z_offset=0))   # Start of main cabin
    geom.add_section(CrossSection(150.0, 20.0, 20.0, z_offset=0))   # Start of tail cabin
    geom.add_section(CrossSection(170.0, 10.0, 10.0, z_offset=10))   # Tail (higher)
    
    print(f"Fuselage volume: {geom.calculate_volume():.1f} m^3")
    print(f"Wetted area: {geom.calculate_wetted_area():.1f} m^2")
    
    # Create and plot the 3D object
    obj = geom.create_object()
    # plot the orthographic views
    ax, fig = plot_orthographic_views(obj)
    plt.show()
    # plot the 3d views
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    plot_3d_shape(ax, obj)
    plt.show()
