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
            [chord, 0, -thickness*.1]  # 3: Trailing edge bottom (slightly below top)
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

def create_box(width: float, length: float, height: float) -> Shape3D:
    """Create vertices and faces for a box (cuboid)
    
    Args:
        width: Width of the box (y-dimension)
        length: Length of the box (x-dimension)
        height: Height of the box (z-dimension)
    """
    # Create vertices
    vertices = np.array([
        [0, 0, 0],          # 0: bottom front left
        [length, 0, 0],     # 1: bottom front right
        [length, width, 0], # 2: bottom back right
        [0, width, 0],      # 3: bottom back left
        [0, 0, height],     # 4: top front left
        [length, 0, height], # 5: top front right
        [length, width, height], # 6: top back right
        [0, width, height]   # 7: top back left
    ])
    
    # Define faces using vertex indices
    faces = [
        [0, 1, 2, 3],  # bottom
        [4, 5, 6, 7],  # top
        [0, 1, 5, 4],  # front
        [2, 3, 7, 6],  # back
        [0, 3, 7, 4],  # left
        [1, 2, 6, 5]   # right
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
        obj.apply_position(obj.position)
        
    for shape in obj.shapes:
        plot_3d_shape(ax, shape, color, alpha)

def plot_orthographic_views(obj: Object3D, fig: Optional[plt.Figure] = None, axes: Optional[List[plt.Axes]] = None) -> Tuple[plt.Figure, Tuple[plt.Axes, plt.Axes, plt.Axes]]:
    """Create three-view orthographic projection of a 3D object
    
    Args:
        obj: The 3D object to plot
        fig: Optional figure to plot on. If None, creates a new figure
        axes: Optional list of 3 axes to plot on. If None, creates new axes
        
    Returns:
        Tuple of (Figure, (top_ax, side_ax, front_ax))
    """
    if fig is None:
        fig = plt.figure(figsize=(15, 10))
    
    if axes is None:
        ax_top = fig.add_subplot(221)
        ax_side = fig.add_subplot(223)
        ax_front = fig.add_subplot(222)
    else:
        ax_top, ax_side, ax_front = axes
    
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
    
    return fig, (ax_top, ax_side, ax_front)

def plot_cross_section(obj: Object3D, plane: str = 'x', value: float = 0.0, height: float = 1.0,
                      use_opacity: bool = False,
                      ax: Optional[plt.Axes] = None) -> plt.Axes:
    """Create a cross-sectional view of a 3D object by slicing along a specified plane.
    
    Args:
        obj: The 3D object to slice
        plane: The plane to slice along ('x', 'y', or 'z')
        value: The coordinate value where to make the slice
        height: The thickness of the slice (objects within ±height/2 will be shown)
        use_opacity: Whether to vary opacity based on distance from slice plane
        ax: Optional axes to plot on. If None, uses current axes
        
    Returns:
        The axes object with the plot
    """
    if ax is None:
        ax = plt.gca()
    
    # Define axis mappings for different planes
    axis_map = {
        'x': {'slice': 0, 'horiz': 1, 'vert': 2, 'xlabel': 'Y (ft)', 'ylabel': 'Z (ft)'},
        'y': {'slice': 1, 'horiz': 0, 'vert': 2, 'xlabel': 'X (ft)', 'ylabel': 'Z (ft)'},
        'z': {'slice': 2, 'horiz': 0, 'vert': 1, 'xlabel': 'X (ft)', 'ylabel': 'Y (ft)'}
    }
    
    if plane not in axis_map:
        raise ValueError(f"Invalid plane '{plane}'. Must be one of: {list(axis_map.keys())}")
    
    axes = axis_map[plane]
    half_height = height / 2
    slice_min = value - half_height
    slice_max = value + half_height
    
    # Apply any global position offset
    if np.any(obj.position != 0):
        obj.apply_position(obj.position)
    
    # Process all shapes at once
    all_faces = []
    all_colors = []
    all_alphas = [] if use_opacity else None
    
    for shape in obj.shapes:
        vertices = shape.vertices
        color = shape.metadata.get('color', 'blue')
        
        # Get slice coordinates for all vertices
        slice_coords = vertices[:, axes['slice']]
        
        # For each face, check intersection and project if needed
        for face in shape.faces:
            face_vertices = vertices[face]
            face_slice_coords = slice_coords[face]
            
            # Vectorized intersection test
            min_slice = np.min(face_slice_coords)
            max_slice = np.max(face_slice_coords)
            
            if not (max_slice < slice_min or min_slice > slice_max):
                # Project vertices to view plane
                x = face_vertices[:, axes['horiz']]
                y = face_vertices[:, axes['vert']]
                projected_vertices = np.column_stack((x, y))
                
                all_faces.append(projected_vertices)
                all_colors.append(color)
                
                if use_opacity:
                    # Calculate alpha based on minimum distance from slice plane
                    min_dist = min(abs(min_slice - value), abs(max_slice - value))
                    alpha = max(0.1, 1 - (min_dist / half_height))
                    all_alphas.append(alpha)
    
    # Sort and plot faces
    if all_faces:
        if use_opacity:
            # Sort by alpha (which corresponds to distance from slice)
            alphas = np.array(all_alphas)
            sort_idx = np.argsort(alphas)
            all_faces = [all_faces[i] for i in sort_idx]
            all_colors = [all_colors[i] for i in sort_idx]
            all_alphas = alphas[sort_idx]
            
            # Plot all faces in sorted order with varying opacity
            for face, color, alpha in zip(all_faces, all_colors, all_alphas):
                ax.fill(face[:, 0], face[:, 1], color=color, alpha=alpha * 0.3)
                ax.plot(face[:, 0], face[:, 1], color=color, alpha=alpha)
        else:
            # Plot all faces with constant opacity
            for face, color in zip(all_faces, all_colors):
                ax.fill(face[:, 0], face[:, 1], color=color, alpha=0.3)
                ax.plot(face[:, 0], face[:, 1], color=color)
    
    # Set labels and title
    ax.set_xlabel(axes['xlabel'])
    ax.set_ylabel(axes['ylabel'])
    ax.set_title(f'Cross Section at {plane.upper()}={value:.1f} ft')
    ax.grid(True)
    ax.set_aspect('equal')
    
    return ax 