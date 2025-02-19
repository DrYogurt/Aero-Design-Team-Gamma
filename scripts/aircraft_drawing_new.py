import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import make_interp_spline
import math
from scipy.special import comb


def process_control_points(points):
    """Process control points to ensure smooth curve generation"""
    # Convert to numpy array for easier manipulation
    points = np.array(points)
    
    # Split into left and right sides
    left_points = points[points[:, 0] <= 0]
    right_points = points[points[:, 0] >= 0]
    
    # Sort by y coordinate first
    left_points = left_points[left_points[:, 1].argsort()]
    right_points = right_points[right_points[:, 1].argsort()]
    
    return left_points, right_points


class AircraftPlotter:
    def __init__(self, total_passengers, seat_configs, galleys, wingspan=30, sweep=25,
                 wing_chord_data=[(0, 4), (50, 3), (100, 2)], wing_thickness=0.12, dihedral=5,
                 v_span=6, v_sweep=35, v_chord_data=[(0, 4), (100, 2)],
                 h_span=12, h_sweep=30, h_chord_data=[(0, 3), (100, 1.5)], tail_configuration='high',
                 cockpit_length=5, cargo_per_passenger = 10, wing_center_x=None, wing_center_y=None,
                 engines=None):
        """
        Initialize aircraft parameters based on passenger count and seating configuration
        
        Args:
            total_passengers: int, total number of passengers
            seat_configs: list of dicts with seating config for each floor
            galleys: list of dicts with format {'length': float, 'floor': int}
            wingspan: float, wingspan in meters
            sweep: float, wing sweep angle in degrees
            wing_chord_data: list of tuples (pos%, chord_length)
            wing_thickness: float, wing thickness ratio
            dihedral: float, wing dihedral angle in degrees
            v_span: float, vertical tail span in meters
            v_sweep: float, vertical tail sweep angle in degrees
            v_chord_data: list of tuples (pos%, chord_length)
            h_span: float, horizontal tail span in meters 
            h_sweep: float, horizontal tail sweep angle in degrees
            h_chord_data: list of tuples (pos%, chord_length)
            tail_configuration: string, high or low
            cockpit_length: float, cockpit length in meters
            cargo_per_passenger: float, cargo volume per passenger 
        """
        # Calculate required rows and fuselage dimensions
        total_seats_per_row = sum(
        sum(config['seat_distribution']) + (len(config['seat_distribution']) - 1)  # Add aisles
        for config in seat_configs
        if not config.get('cargo', False)
        )
        print(f"seats per row: {total_seats_per_row}")
        self.required_rows = -(-total_passengers // total_seats_per_row)  # Ceiling division
        
        # Calculate maximum floor width
        max_floor_width = max(
            sum(dist) * config['seat_width'] + 
            (len(dist) - 1) * config['aisle_width']  # Width for aisles
            for config, dist in ((config, config['seat_distribution']) 
            for config in seat_configs)
        )
        
        # Add 20% margin to width for fuselage structure
        self.fuselage_width = max_floor_width * 1.2
        
        # Calculate total height
        self.total_height = sum(config['ceiling_height']
                              for config in seat_configs)

        
        # Calculate total length
        galley_length = sum(galley['length']*galley['number'] for galley in galleys)
        total_row_depth = self.required_rows * max(config['seat_depth'] 
                                                 for config in seat_configs)
        self.fuselage_length = (total_row_depth + galley_length) * 1.1  # 10% margin

        cargo_volume = sum(config['seat_width']*config['seat_height'] * self.fuselage_length for config in seat_configs if config.get('cargo', False))
        print(f"Total Cargo Volume: {cargo_volume:.0f} ft^3")
        assert cargo_volume >= total_passengers * cargo_per_passenger
        
        # Set all parameters
        self.wing = {
            'span': wingspan,
            'sweep': np.radians(sweep),
            'chord_data': wing_chord_data,
            'thickness': wing_thickness,
            'dihedral': np.radians(dihedral)
        }
        
        self.vtail = {
            'span': v_span,
            'sweep': np.radians(v_sweep),
            'chord_data': v_chord_data
        }
        
        self.htail = {
            'span': h_span,
            'sweep': np.radians(h_sweep),
            'chord_data': h_chord_data,
            'configuration': tail_configuration,
        }
        
        self.fuselage = {
            'width': self.fuselage_width,
            'length': self.fuselage_length,
            'height': self.total_height,
            'num_floors': len(seat_configs),
            'floor_heights': [config['ceiling_height'] 
                            for config in seat_configs]
        }
        
        self.cockpit = {
            'length': cockpit_length
        }
        
        self.seating = {
            'configs': seat_configs,
            'spacing': 0.05
        }
        
        self.galleys = galleys

        self.wing.update({
        'center_x': wing_center_x if wing_center_x is not None else -self.fuselage_length/4,
        'center_y': wing_center_y if wing_center_y is not None else self.total_height/2
        })

        self.wing.update({
            'area': self._calculate_wing_area(),
            'AR': self._calculate_aspect_ratio(),
            'volume': self._calculate_wing_volume(),
        })
        
        self.engines = engines or []

    def _calculate_wing_area(self):
        """
        Calculate total wing area by integrating chord distribution along span.
        Returns area of both wings combined.
        """
        half_span = self.wing['span'] / 2
        chord_data = self.wing['chord_data']
        
        # Convert percentage positions to actual distances
        positions = [pos * half_span / 100 for pos, _ in chord_data]
        chords = [chord for _, chord in chord_data]
        
        # Calculate area by trapezoidal integration
        area = 0
        for i in range(len(positions) - 1):
            dx = positions[i+1] - positions[i]
            avg_chord = (chords[i] + chords[i+1]) / 2
            area += dx * avg_chord
            
        # Multiply by 2 for both wings
        return 2 * area

    def _calculate_wing_volume(self):
        """
        Calculate total wing volume by integrating chord and thickness distribution along span.
        Returns volume of both wings combined.
        Uses thickness ratio and assumes a typical airfoil shape factor of 0.6.
        """
        half_span = self.wing['span'] / 2
        chord_data = self.wing['chord_data']
        thickness_ratio = self.wing['thickness']
        airfoil_shape_factor = 0.6  # Typical value for transport aircraft airfoils
        
        # Convert percentage positions to actual distances
        positions = [pos * half_span / 100 for pos, _ in chord_data]
        chords = [chord for _, chord in chord_data]
        
        # Calculate volume by trapezoidal integration
        volume = 0
        for i in range(len(positions) - 1):
            dx = positions[i+1] - positions[i]
            # Average chord in this section
            avg_chord = (chords[i] + chords[i+1]) / 2
            # Volume of wing section = length * chord * thickness * shape factor
            section_volume = dx * avg_chord * avg_chord * thickness_ratio * airfoil_shape_factor
            volume += section_volume
            
        # Multiply by 2 for both wings, but use triangular cross section
        return volume 
    
    def _calculate_aspect_ratio(self):
        """
        Calculate wing aspect ratio: AR = (wingspan)^2 / area
        """
        wing_area = self._calculate_wing_area()
        return (self.wing['span']**2) / wing_area

    
    def _generate_wing_points(self):
        """Generate wing coordinates based on parameters"""
        points = []
        half_span = self.wing['span'] / 2
        
        for pos, chord in self.wing['chord_data']:
            # Convert percentage to actual position
            y = pos * half_span / 100
            # Calculate sweep offset
            x = y * np.tan(self.wing['sweep'])
            points.append((x, y, chord))
            
        return np.array(points)


    def to_dict(self):
        """Return all aircraft parameters as a dictionary"""
        return {
            'wing': self.wing,
            'vtail': self.vtail,
            'htail': self.htail,
            'fuselage': self.fuselage,
            'cockpit': self.cockpit,
            'seating': self.seating,
            'galleys': self.galleys,
            'required_rows': self.required_rows,
            'fuselage_width': self.fuselage_width,
            'total_height': self.total_height,
            'fuselage_length': self.fuselage_length
        }
    
    
    def _plot_engines(self, ax, view='top'):
        """Plot engines based on view type"""
        if not self.engines:
            return
            
        for engine in self.engines:
            radius = engine['radius']
            length = engine['length']
            
            for x, y, height in engine['positions']:
                # Mirror positions
                for sign in [1, -1]:
                    if view == 'front':
                        circle = plt.Circle((y * sign, height), radius, 
                                         fill=False, color='purple', linewidth=2)
                        ax.add_patch(circle)
                        
                    elif view == 'side':
                        rect = plt.Rectangle((x, height - radius), length, 2*radius,
                                          fill=False, color='purple', linewidth=2)
                        ax.add_patch(rect)
                        
                    elif view == 'top':
                        # Dotted if above wing, solid if below
                        style = '--' if height > self.wing['center_y'] else '-'
                        rect = plt.Rectangle((x, (y) * sign - radius), length, 2*radius,
                                          fill=False, color='purple', 
                                          linestyle=style, linewidth=2)
                        ax.add_patch(rect)

    def _plot_top_content(self, ax):
        """Plot top view content on given axes"""
        # Fuselage rectangle
        fus_width = self.fuselage['width']
        fus_length = self.fuselage['length']
        ax.plot([-fus_length/2, fus_length/2, fus_length/2, -fus_length/2, -fus_length/2], 
                [-fus_width/2, -fus_width/2, fus_width/2, fus_width/2, -fus_width/2], 
                'k-', linewidth=2)
        
        # Cockpit oval
        t = np.linspace(np.pi/2, 3*np.pi/2, 100)
        cockpit_x = -fus_length/2 + (self.cockpit['length']) * np.cos(t)
        cockpit_y = (fus_width/2) * np.sin(t)
        ax.plot(cockpit_x, cockpit_y, 'k-', linewidth=2)
        
        # Wings
        wing_start_x =  self.wing['center_x']
        wing_points = self._generate_wing_points()
        leading_edge_points = [(x + wing_start_x, y) for x, y, _ in wing_points]
        trailing_edge_points = [(x + wing_start_x + chord, y) for x, y, chord in wing_points]
        
        x_le = [x for x, y in leading_edge_points]
        y_le = [y for x, y in leading_edge_points]
        x_te = [x for x, y in trailing_edge_points]
        y_te = [y for x, y in trailing_edge_points]
        
        # Plot wings
        for sign in [1, -1]:  # Plot both wings
            ax.plot(x_le, [y * sign for y in y_le], 'b-', linewidth=2)
            ax.plot(x_te, [y * sign for y in y_te], 'b-', linewidth=2)
            ax.plot([x_le[-1], x_te[-1]], 
                   [y_le[-1] * sign, y_te[-1] * sign], 'b-', linewidth=2)
        
        # Tail section
        tail_length = self.vtail['chord_data'][0][1]
        ax.plot([fus_length/2, fus_length/2 + tail_length], 
                [-fus_width/2, 0], 'k-', linewidth=2)
        ax.plot([fus_length/2, fus_length/2 + tail_length], 
                [fus_width/2, 0], 'k-', linewidth=2)
        
        # Horizontal tail
        htail_le_points = []
        htail_te_points = []
        htail_root_x = fus_length/2 - self.htail['chord_data'][0][1]
        
        for pos, chord in self.htail['chord_data']:
            y = pos * self.htail['span'] / 2 / 100
            x = htail_root_x + tail_length + y * np.tan(self.htail['sweep'])
            htail_le_points.append((x, y))
            htail_te_points.append((x + chord, y))
        
        htail_x_le = [x for x, y in htail_le_points]
        htail_y_le = [y for x, y in htail_le_points]
        htail_x_te = [x for x, y in htail_te_points]
        htail_y_te = [y for x, y in htail_te_points]
        
        # Plot horizontal tail
        for sign in [1, -1]:  # Plot both sides
            ax.plot(htail_x_le, [y * sign for y in htail_y_le], 'r-', linewidth=2)
            ax.plot(htail_x_te, [y * sign for y in htail_y_te], 'r-', linewidth=2)
            ax.plot([htail_x_le[-1], htail_x_te[-1]], 
                   [htail_y_le[-1] * sign, htail_y_te[-1] * sign], 'r-', linewidth=2)

        self._plot_engines(ax, 'top')
        
        ax.grid(True)
        ax.set_title('Top View')

    def _plot_side_content(self, ax):
        """Plot side view content on given axes"""
        height = self.fuselage['height']
        length = self.fuselage['length']
        
        # Fuselage
        ax.plot([-length/2, length/2], [0, 0], 'k-', linewidth=2)
        ax.plot([-length/2, length/2], [height, height], 'k-', linewidth=2)
        ax.plot([-length/2, -length/2], [0, height], 'k-', linewidth=2)
        ax.plot([length/2, length/2], [0, height], 'k-', linewidth=2)
        
        # Cockpit
        t = np.linspace(np.pi/2, 3*np.pi/2, 100)
        x = -length/2 + self.cockpit['length'] * np.cos(t)
        y = height/2 + (height/2) * np.sin(t)
        ax.plot(x, y, 'k-')
        
        wing_root_chord = self.wing['chord_data'][0][1]
        wing_start_x = self.wing['center_x']
        wing_start_height = self.wing['center_y']
        half_span = self.wing['span'] / 2
        wing_height = half_span * np.tan(self.wing['dihedral'])
        wing_tip_chord = self.wing['chord_data'][-1][1]
        tip_x = half_span * np.tan(self.wing['sweep'])
        
        ax.plot([wing_start_x, wing_start_x + wing_root_chord], 
                [wing_start_height, wing_start_height], 'b-', linewidth=2)
        ax.plot([wing_start_x, wing_start_x+tip_x], 
                [wing_start_height,wing_start_height + wing_height], 'b-', linewidth=2)
        ax.plot([wing_start_x+tip_x, wing_start_x + tip_x + wing_tip_chord], 
                [wing_start_height + wing_height,wing_start_height + wing_height], 'b-', linewidth=2)
        ax.plot([wing_start_x + wing_root_chord, wing_start_x + tip_x + wing_tip_chord], 
                [wing_start_height,wing_start_height+ wing_height], 'b-', linewidth=2)
        
        # Tail section
        vtail_height = self.vtail['span']
        vtail_sweep = self.vtail['sweep']
        vtail_root = self.vtail['chord_data'][0][1]
        vtail_tip = self.vtail['chord_data'][-1][1]

        # Plot tail fuselage (side view - right triangle)
        tail_length = vtail_root = vtail_root  # Use vertical tail root chord as length
        tail_height = height
        ax.plot([length/2, length/2+tail_length], [height - tail_height, height], 'k-', linewidth=2)
        ax.plot([length/2, length/2+tail_length], [height, height], 'k-', linewidth=2)

        
        # Vertical tail
        ax.plot([length/2, length/2 + vtail_height*np.tan(vtail_sweep)], 
                [height, height + vtail_height], 'g-', linewidth=2)
        ax.plot([length/2+vtail_root, length/2 + vtail_height*np.tan(vtail_sweep)+vtail_tip], 
                [height, height + vtail_height], 'g-', linewidth=2)
        ax.plot([length/2 + vtail_height*np.tan(vtail_sweep), 
                length/2 + vtail_height*np.tan(vtail_sweep)+vtail_tip], 
                [height + vtail_height, height + vtail_height], 'g-', linewidth=2)
        
        # Horizontal tail
        htail_root_chord = self.htail['chord_data'][0][1]
        if self.htail['configuration'] == 'high':
            htail_start_x = length/2 + vtail_height*np.tan(vtail_sweep)
            htail_y = height + vtail_height
        elif self.htail['configuration'] == 'low':
            htail_start_x = length/2 + vtail_root - htail_root_chord
            htail_y = height / 2
        else:
            raise ValueError("Unknown configuration. Please use either low or high")
            
        ax.plot([htail_start_x, htail_start_x + htail_root_chord], 
                [htail_y, htail_y], 'r-', linewidth=2)

        self._plot_engines(ax, 'side')
        
        ax.grid(True)
        ax.set_title('Side View')

    def _plot_front_content(self, ax):
        """Plot front view content on given axes"""
        width = self.fuselage['width']
        height = self.fuselage['height']
        
        # Fuselage ellipse
        t = np.linspace(0, 2*np.pi, 100)
        ax.plot(width/2 * np.cos(t), height/2 + height/2 * np.sin(t), 'k-', linewidth=2)
        
        # Wings with dihedral
        half_span = self.wing['span'] / 2
        wing_start_height = self.wing['center_y']
        wing_height = half_span * np.tan(self.wing['dihedral'])
        ax.plot([0, half_span], [wing_start_height, wing_start_height + wing_height], 'b-', linewidth=2)
        ax.plot([0, -half_span], [wing_start_height,wing_start_height + wing_height], 'b-', linewidth=2)
        
        # Horizontal tail
        if self.htail['configuration'] == 'high':
            htail_height = height + self.vtail['span']
        elif self.htail['configuration'] == 'low':
            htail_height = height/2
            
        htail_span = self.htail['span'] / 2
        ax.plot([-htail_span, htail_span], [htail_height, htail_height], 'r-', linewidth=2)
        
        # Vertical tail
        ax.plot([0, 0], [height, height + self.vtail['span']], 'g-', linewidth=2)
        
        # Floor lines
        for h in np.cumsum(self.fuselage['floor_heights'])[:-1]:
            ax.plot([-width/2, width/2], [h, h], 'k--', linewidth=1)

        self._plot_engines(ax, 'front')
        
        ax.grid(True)
        ax.set_title('Front View')

    def _plot_cross_section_content(self, ax):
        """Plot cross section content on given axis"""
        width = self.fuselage['width']
        floor_heights = self.fuselage['floor_heights']
        y_positions = np.cumsum([0] + floor_heights)
        control_points = []
        
        for floor_idx in range(len(floor_heights)):
            floor_y = y_positions[floor_idx]
            config = self.seating['configs'][floor_idx]
            seat_distribution = config['seat_distribution']
            
            # Calculate seating layout dimensions
            seat_width = config['seat_width']
            effective_seat_width = seat_width + self.seating['spacing']
            aisle_width = config['aisle_width']
            effective_seat_height = config['seat_height'] + config['headroom_height']
            
            # Calculate total width including all seats and aisles
            total_width = (sum(seat_distribution) * effective_seat_width + 
                          (len(seat_distribution) - 1) * aisle_width)
            
            # Add control points for fuselage shape
            if floor_y < y_positions[-1] / 2:
                control_points.extend([
                    (-total_width/2, floor_y),
                    (total_width/2, floor_y)
                ])
    
            if (floor_y + effective_seat_height) > (y_positions[-1] / 2):
                control_points.extend([
                    (-total_width/2, floor_y + effective_seat_height),
                    (total_width/2, floor_y + effective_seat_height)
                ])
                
            # Draw floor line
            floor_line = plt.Line2D([-total_width/2, total_width/2], 
                                  [floor_y, floor_y], 
                                  linestyle='--', 
                                  color='k', 
                                  linewidth=1)
            ax.add_line(floor_line)
    
            # Draw seats and aisles
            current_x = -total_width/2
            
            for section_idx, num_seats in enumerate(seat_distribution):
                # Draw seats in this section
                for seat in range(num_seats):
                    seat_patch = plt.Rectangle(
                        (current_x, floor_y),
                        seat_width,
                        config['seat_height'],
                        fill=True,
                        color='blue',
                        alpha=0.5
                    )
                    ax.add_patch(seat_patch)
                    current_x += effective_seat_width
                
                # Draw aisle after this section (except for last section)
                if section_idx < len(seat_distribution) - 1:
                    aisle_line = plt.Line2D(
                        [current_x, current_x + aisle_width],
                        [floor_y, floor_y],
                        color='r',
                        linewidth=2
                    )
                    ax.add_line(aisle_line)
                    current_x += aisle_width
        
        # Add points for top and bottom curves
        control_points.extend([
            (0, y_positions[0]-floor_heights[0]/4-1e-3),
            (0, y_positions[-1]+1e-3),
        ])
        control_points = np.array(control_points)
        
        # Generate smooth curves
        left_points, right_points = process_control_points(control_points)
        
        t_left = np.linspace(0, 1, len(left_points))
        t_right = np.linspace(0, 1, len(right_points))
        
        left_spline = make_interp_spline(t_left, left_points, k=3)
        right_spline = make_interp_spline(t_right, right_points, k=3)
        
        t_smooth = np.linspace(0, 1, 100)
        left_smooth = left_spline(t_smooth)
        right_smooth = right_spline(t_smooth)
        
        # Plot fuselage outline using Line2D
        left_line = plt.Line2D(left_smooth[:, 0], left_smooth[:, 1], 
                              color='k', linewidth=2)
        right_line = plt.Line2D(right_smooth[:, 0], right_smooth[:, 1], 
                               color='k', linewidth=2)
        ax.add_line(left_line)
        ax.add_line(right_line)
        
        # Set view limits
        x_max = max(control_points[:,0])
        y_max = max(control_points[:,1])
        y_min = min(control_points[:,1])
        margin = max(y_positions[-1],total_width) * .1
        ax.set_xlim(-x_max - margin,x_max + margin)
        ax.set_ylim(y_min - margin, y_max + margin)
        
        ax.grid(True)
        ax.set_title('Cross Section')
        ax.set_aspect('equal')
        
    def plot_three_view(self):
        """Generate a three-view drawing of the aircraft with shared axes"""
        fig = plt.figure(figsize=(20, 20))
        gs = fig.add_gridspec(2, 2, width_ratios=[1, 1], height_ratios=[1,1], 
                            hspace=-0.4, wspace=0.2)
        
        ax_top = fig.add_subplot(gs[0, 0])
        ax_side = fig.add_subplot(gs[1, 0])
        ax_front = fig.add_subplot(gs[0, 1])
        
        # Calculate maximum dimensions with larger margins
        max_length = self.fuselage['length'] * 1.4
        max_width = self.wing['span'] * 1.4
        max_height = (self.fuselage['height']) * 1.4
        tail_length =  self.vtail['span'] * self.vtail['sweep'] + self.vtail['chord_data'][0][1]
        tail_height =  self.vtail['span']
        
        # Plot content
        self._plot_top_content(ax_top)
        self._plot_side_content(ax_side)
        self._plot_front_content(ax_front)
        
        # Set limits
        ax_top.set_xlim([-max_length/2, max_length/2 + tail_length])
        ax_top.set_ylim([-max_width/2, max_width/2])
        
        ax_side.set_xlim([-max_length/2, max_length/2 + tail_length])
        
        ax_front.set_xlim([-max_width/2, max_width/2])
        ax_front.set_ylim([-max_height, max_height + tail_height])
        
        # Set labels
        ax_top.set_xlabel('Length (ft)')
        ax_top.set_ylabel('Span (ft)')
        ax_side.set_xlabel('Length (ft)')
        ax_side.set_ylabel('Height (ft)')
        ax_front.set_xlabel('Span (ft)')
        ax_front.set_ylabel('Height (ft)')
        
        # Set equal aspect ratio
        for ax in [ax_top, ax_side, ax_front]:
            ax.set_aspect('equal', adjustable='box')
        
        plt.tight_layout()
        return fig

    def plot_top_view(self):
        """Generate standalone top view"""
        fig, ax = plt.subplots(figsize=(12, 8))
        self._plot_top_content(ax)
        ax.set_aspect('equal', adjustable='box')
        return fig, ax

    def plot_side_view(self):
        """Generate standalone side view"""
        fig, ax = plt.subplots(figsize=(12, 6))
        self._plot_side_content(ax)
        ax.set_aspect('equal', adjustable='box')
        return fig, ax

    def plot_front_view(self):
        """Generate standalone front view"""
        fig, ax = plt.subplots(figsize=(10, 8))
        self._plot_front_content(ax)
        ax.set_aspect('equal', adjustable='box')
        return fig, ax

    def plot_cross_section(self):
        """Generate standalone cross section view"""
        fig, ax = plt.subplots(figsize=(10, 8))
        self._plot_cross_section_content(ax)
        return fig, ax

# Example usage:
def main():
    # Add seating configuration
    seat_configs = [
        {
            'seat_width': 14,
            'seat_depth': 1,    
            'seat_height': 8,
            'headroom_height':0,
            'ceiling_height':8,
            'aisle_width': 0.5,
            'seat_distribution':[1],
            'cargo':True
        },
        {
            'seat_width': 1.5,
            'seat_depth': 2.6,
            'seat_height': 4,
            'headroom_height':1,
            'ceiling_height':7,
            'aisle_width': 2,
            'seat_distribution':[3,6,3]
        },
        {
            'seat_width': 1.5,
            'seat_depth': 2.6,
            'seat_height': 4,
            'headroom_height':1,
            'ceiling_height':7,
            'aisle_width': 2,
            'seat_distribution':[3,5,3]
        },
        {
            'seat_width': 2.5,
            'seat_depth': 2.6,
            'seat_height': 4,
            'headroom_height':1,
            'ceiling_height':7,
            'aisle_width': 2,
            'seat_distribution':[1,2,1]
        },

    ]
    galleys = [
        {'length': 20, 'number':2, 'floor': 1},
        {'length': 20, 'number':2, 'floor': 2}
    ]

    engines = [
    {
        'radius': 5,
        'length': 20,
        'positions': [
            (-20, 50, 0),  # (x, y, height) from centerline
            (15, 100, 5)
        ]
    }
    ]

    
    plotter = AircraftPlotter(
        total_passengers=1250,
        seat_configs=seat_configs,
        galleys=galleys,
        wingspan=300, #f
        sweep=37.5, #deg
        wing_chord_data=[(0, 65), (30, 35), (100, 10)],
        wing_thickness=0.12, dihedral=5,
        v_span=20, 
        v_sweep=35,
        v_chord_data=[(0, 60), (100, 40)],
        h_span=100,
        h_sweep=35,
        h_chord_data=[(0, 40), (100, 10)],
        tail_configuration='low',
        wing_center_x=-50,  # Move wing forward
        wing_center_y=2,   # Adjust wing height
        cockpit_length=20,
        engines=engines
    )
    import pprint
    pprint.pprint(plotter.to_dict())
    # Generate plots
    fig, ax = plotter.plot_top_view()
    plt.savefig('assets/aircraft_top_view.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    fig, ax = plotter.plot_side_view()
    plt.savefig('assets/aircraft_side_view.png', dpi=300, bbox_inches='tight')
    plt.close()

    fig, ax = plotter.plot_front_view()
    plt.savefig('assets/aircraft_front_view.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    fig, ax = plotter.plot_cross_section()
    plt.savefig('assets/aircraft_cross_section.png', dpi=300, bbox_inches='tight')
    plt.close()

    fig = plotter.plot_three_view()
    plt.savefig('assets/aircraft_three_view.png', dpi=300, bbox_inches='tight')
    plt.close()
    

if __name__ == "__main__":
    main()
    
    # DONE = fix wing placement along fuselage (top view)
    # add wing engines
    # CP and CG calculation
    # overhead bins
    # cargo volume check/assertion
    # thickness on the walls and floor
    
    # DONE = add dihedral (side view)
    # add tail dihedral (side view)
    # add sections on top view
    # add partial second floor
    # control surfaces
    # landing gear

    # CD_0 correlation:
        # component by component estimation
        # find renoylds number and coefficient of friction
    # C_L correlation:
        # Find c_bar
        # find wetted area 
    # raymer 48,52,57,63