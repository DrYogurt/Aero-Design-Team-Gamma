import numpy as np
from typing import List, Tuple, Optional, Dict, Any
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class Shape3D:
    """Represents a 3D shape with vertices and faces"""
    def __init__(self, vertices: np.ndarray, faces: List[List[int]], metadata: Dict[str, Any] = None):
        self.vertices = vertices
        self.faces = faces
        self.metadata = metadata if metadata is not None else {}

class Object3D:
    """Represents a complete 3D object composed of multiple shapes"""
    def __init__(self, shapes: List[Shape3D] = None):
        self.shapes = shapes if shapes is not None else []
        self.position = np.zeros(3)  # Global position offset
        self.metadata: Dict[str, Any] = {}
    
    def add_shape(self, shape: Shape3D) -> None:
        """Add a shape to the object"""
        self.shapes.append(shape)
    
    def apply_position(self, position: np.ndarray) -> None:
        """Apply a position offset to all shapes"""
        # print(f"Applying position offset: {position}")
        for shape in self.shapes:
            shape.vertices = shape.vertices + position
            # print(f"  Shape vertices range: {np.min(shape.vertices, axis=0)} to {np.max(shape.vertices, axis=0)}")
            
    def get_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get the min and max bounds of the object"""
        if not self.shapes:
            return np.zeros(3), np.zeros(3)
        
        all_vertices = np.vstack([shape.vertices for shape in self.shapes])
        return np.min(all_vertices, axis=0), np.max(all_vertices, axis=0)

def create_cylinder(radius: float, height: float, num_points: int = 32) -> Shape3D:
    """Create vertices and faces for a cylinder"""
    # Create circular points for top and bottom
    theta = np.linspace(0, 2*np.pi, num_points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    
    # Create vertices
    vertices = []
    # Bottom circle
    for i in range(num_points):
        vertices.append([x[i], y[i], 0])
    # Top circle
    for i in range(num_points):
        vertices.append([x[i], y[i], height])
    # Add center points for top and bottom caps
    vertices.append([0, 0, 0])  # Bottom center
    vertices.append([0, 0, height])  # Top center
    
    vertices = np.array(vertices)
    
    # Create faces
    faces = []
    # Side faces
    for i in range(num_points):
        next_i = (i + 1) % num_points
        faces.append([i, next_i, next_i + num_points, i + num_points])
    
    # Bottom cap
    bottom_center = 2 * num_points
    for i in range(num_points):
        next_i = (i + 1) % num_points
        faces.append([i, next_i, bottom_center])
    
    # Top cap
    top_center = 2 * num_points + 1
    for i in range(num_points):
        next_i = (i + 1) % num_points
        faces.append([i + num_points, next_i + num_points, top_center])
    
    return Shape3D(vertices=vertices, faces=faces)

def create_ellipsoid(a: float, b: float, c: float, num_points: int = 32) -> Shape3D:
    """Create vertices and faces for an ellipsoid"""
    # Create points on a sphere and scale them
    u = np.linspace(0, 2*np.pi, num_points)
    v = np.linspace(0, np.pi, num_points)
    x = a * np.outer(np.cos(u), np.sin(v))
    y = b * np.outer(np.sin(u), np.sin(v))
    z = c * np.outer(np.ones_like(u), np.cos(v))
    
    # Create vertices and faces
    vertices = []
    faces = []
    
    # Convert the parametric surface points to vertices and faces
    for i in range(num_points-1):
        for j in range(num_points-1):
            vertices.extend([
                [x[i,j], y[i,j], z[i,j]],
                [x[i+1,j], y[i+1,j], z[i+1,j]],
                [x[i+1,j+1], y[i+1,j+1], z[i+1,j+1]],
                [x[i,j+1], y[i,j+1], z[i,j+1]]
            ])
            base_idx = len(vertices) - 4
            faces.append([base_idx, base_idx+1, base_idx+2, base_idx+3])
    
    return Shape3D(vertices=np.array(vertices), faces=faces)

def create_wing_section(root_pos: np.ndarray, tip_pos: np.ndarray, 
                       root_chord: float, tip_chord: float,
                       thickness_ratio_root: float = 0.12,
                       thickness_ratio_tip: float = 0.08,
                       num_points: int = 20) -> Shape3D:
    """Create vertices and faces for a wing section with triangular thickness"""
    # Function to create points for a triangular wing section
    def wing_section_points(chord: float, thickness: float) -> np.ndarray:
        # Create points for a triangular wing section
        points = np.array([
            [0, 0, 0],              # 0: Leading edge top
            [chord, 0, 0],          # 1: Trailing edge top
            [0, 0, -thickness],     # 2: Leading edge bottom
            [chord, 0, -thickness*.9]  # 3: Trailing edge bottom (slightly below top)
        ])
        return points

    # Create root and tip section points
    root_points = wing_section_points(root_chord, root_chord * thickness_ratio_root)
    tip_points = wing_section_points(tip_chord, tip_chord * thickness_ratio_tip)
    
    # Transform tip points to tip position
    direction = tip_pos - root_pos
    tip_points = tip_points + tip_pos
    root_points = root_points + root_pos
    
    # Create vertices by combining root and tip points
    vertices = np.vstack([root_points, tip_points])
    #print(f"Vertices: {vertices}")
    # Create faces
    faces = [
        [0, 1, 5, 4],      # Top face
        [2, 3, 7, 6],      # Bottom face
        [0, 2, 6, 4],      # Leading edge face
        [1, 3, 7, 5],      # Trailing edge face
        [0, 1, 3, 2],      # Root face
        [4, 5, 7, 6]       # Tip face
    ]
    
    return Shape3D(vertices=vertices, faces=faces)

def plot_3d_shape(ax: Axes3D, shape: Shape3D, color: str = 'blue', alpha: float = 0.5) -> None:
    """Plot a 3D shape on the given axes"""
    polygons = [[shape.vertices[idx] for idx in face] for face in shape.faces]
    poly = Poly3DCollection(polygons, alpha=alpha)
    poly.set_facecolor(color)
    ax.add_collection3d(poly)

def plot_3d_object(ax: Axes3D, obj: Object3D, color: str = 'blue', alpha: float = 0.5) -> None:
    """Plot a complete 3D object"""
    # Apply any global position offset
    if np.any(obj.position != 0):
        # print(f"Plotting object at position: {obj.position}")
        obj.apply_position(obj.position)
        
    for shape in obj.shapes:
        plot_3d_shape(ax, shape, color, alpha)

def plot_orthographic_views(obj: Object3D, figsize: Tuple[int, int] = (15, 10)) -> Tuple[plt.Figure, Tuple[plt.Axes, plt.Axes, plt.Axes]]:
    """Create three-view orthographic projection of a 3D object"""
    fig = plt.figure(figsize=figsize)
    
    # Create three subplots for top, side, and front views
    ax_top = fig.add_subplot(221)
    ax_side = fig.add_subplot(223)
    ax_front = fig.add_subplot(222)
    
    # Apply any global position offset
    if np.any(obj.position != 0):
        obj.apply_position(obj.position)
    
    # Sort shapes by their minimum z-coordinate (height) for bottom-to-top plotting
    sorted_shapes = sorted(obj.shapes, key=lambda shape: np.min(shape.vertices[:, 2]))
    
    # Project vertices onto each plane
    for shape in sorted_shapes:
        vertices = shape.vertices
        faces = shape.faces
        color = shape.metadata.get('color', 'blue')  # Get color from metadata, default to blue
        
        # For each face, plot its projection on each view
        for face in faces:
            face_vertices = vertices[face]
            
            # Top view (X-Y plane)
            x_top = face_vertices[:, 0]
            y_top = face_vertices[:, 1]
            ax_top.fill(x_top, y_top, color=color, alpha=0.3)  # Fill face
            ax_top.plot(x_top, y_top, color=color)  # Draw edges
            
            # Side view (X-Z plane)
            x_side = face_vertices[:, 0]
            z_side = face_vertices[:, 2]
            ax_side.fill(x_side, z_side, color=color, alpha=0.3)  # Fill face
            ax_side.plot(x_side, z_side, color=color)  # Draw edges
            
            # Front view (Y-Z plane)
            y_front = face_vertices[:, 1]
            z_front = face_vertices[:, 2]
            ax_front.fill(y_front, z_front, color=color, alpha=0.3)  # Fill face
            ax_front.plot(y_front, z_front, color=color)  # Draw edges
    
    # Set labels and titles
    ax_top.set_xlabel('X (ft)')
    ax_top.set_ylabel('Y (ft)')
    ax_top.set_title('Top View')
    
    ax_side.set_xlabel('X (ft)')
    ax_side.set_ylabel('Z (ft)')
    ax_side.set_title('Side View')
    
    ax_front.set_xlabel('Y (ft)')
    ax_front.set_ylabel('Z (ft)')
    ax_front.set_title('Front View')
    
    # Set equal aspect ratio for all views
    for ax in (ax_top, ax_side, ax_front):
        ax.set_aspect('equal')
        ax.grid(True)
    
    plt.tight_layout()
    return fig, (ax_top, ax_side, ax_front) 