import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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
    
    # For each y-level, keep only the point with maximum absolute x
    def clean_points(pts):
        cleaned = []
        n = len(pts)
        
        for i in range(n):
            current_point = pts[i]
            if i > 0 and i < n-1:
                prev_point = pts[i-1]
                # Keep point if:
                # - x magnitude increases, or
                # - x magnitude decreases but next point has lower x magnitude
                if abs(prev_point[0]) <= abs(current_point[0]) or \
                   (abs(prev_point[0]) > abs(current_point[0]) and \
                    (abs(current_point[0]) > abs(pts[i+1][0]))):
                    cleaned.append(current_point)
            else:
                cleaned.append(current_point)
                
        return pts #np.array(cleaned)
    
    left_points = clean_points(left_points)
    right_points = clean_points(right_points)
    
    return left_points, right_points


class AircraftPlotter:
    def __init__(self, total_passengers, seat_configs, galleys, wingspan=30, sweep=25,
                 wing_chord_data=[(0, 4), (50, 3), (100, 2)], wing_thickness=0.12, dihedral=5,
                 v_span=6, v_sweep=35, v_chord_data=[(0, 4), (100, 2)],
                 h_span=12, h_sweep=30, h_chord_data=[(0, 3), (100, 1.5)], tail_configuration='high',
                 cockpit_length=5):
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
            cockpit_length: float, cockpit length in meters
        """
        # Calculate required rows and fuselage dimensions
        total_seats_per_row = sum(
            config['seats_per_section'] * config['num_aisles']
            for config in seat_configs
            if not config.get('cargo', False)
        )
        print(f"seats per row: {total_seats_per_row}")
        self.required_rows = -(-total_passengers // total_seats_per_row)  # Ceiling division
        
        # Calculate maximum floor width
        max_floor_width = max(
            config['seats_per_section'] * config['seat_width'] * config['num_aisles'] +
            config['aisle_width'] * config['num_aisles']
            for config in seat_configs
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
    
    def plot_top_view(self):
        """Generate top view of the aircraft"""
        fig, ax = plt.subplots(figsize=(12, 8))
        ax.set_title('Aircraft Top View')
        
        # Plot fuselage as a rectangle
        fus_width = self.fuselage['width']
        fus_length = self.fuselage['length']
        ax.plot([-fus_length/2, fus_length/2, fus_length/2, -fus_length/2, -fus_length/2], 
                [-fus_width/2, -fus_width/2, fus_width/2, fus_width/2, -fus_width/2], 
                'k-', linewidth=2)
        
        # Plot cockpit as a pointed oval
        cockpit_length = self.cockpit['length']
        t = np.linspace(np.pi/2, 3*np.pi/2, 100)
        cockpit_x = -fus_length/2 + (cockpit_length/2) * np.cos(t)
        cockpit_y = (fus_width/2) * np.sin(t)
        ax.plot(cockpit_x, cockpit_y, 'k-', linewidth=2)
        
        # Plot wings with shape
        wing_points = self._generate_wing_points()
        
        # Create leading and trailing edge points
        leading_edge_points = [(x, y) for x, y, _ in wing_points]
        trailing_edge_points = [(x + chord, y) for x, y, chord in wing_points]
        
        # Plot positive wing
        x_le = [x for x, y in leading_edge_points]
        y_le = [y for x, y in leading_edge_points]
        x_te = [x for x, y in trailing_edge_points]
        y_te = [y for x, y in trailing_edge_points]
        
        ax.plot(x_le, y_le, 'b-', linewidth=2)
        ax.plot(x_te, y_te, 'b-', linewidth=2)
        
        # Plot negative wing (mirrored)
        ax.plot(x_le, [-y for y in y_le], 'b-', linewidth=2)
        ax.plot(x_te, [-y for y in y_te], 'b-', linewidth=2)
        
        # Connect wing tips
        ax.plot([x_le[-1], x_te[-1]], [y_le[-1], y_te[-1]], 'b-', linewidth=2)
        ax.plot([x_le[-1], x_te[-1]], [-y_le[-1], -y_te[-1]], 'b-', linewidth=2)
        
        # Plot tail fuselage (top view - isosceles triangle)
        tail_length = self.vtail['chord_data'][0][1]  # Use vertical tail root chord as length
        tail_width = fus_width
        
        ax.plot([fus_length/2, fus_length/2 + tail_length], [-tail_width/2, 0], 'k-', linewidth=2)
        ax.plot([fus_length/2, fus_length/2 + tail_length], [tail_width/2, 0], 'k-', linewidth=2)

        
        # Plot horizontal tail in a similar way to the wing
        htail_le_points = []
        htail_te_points = []
        htail_root_x = fus_length/2 - self.htail['chord_data'][0][1]  # Start from tail end
        
        for pos, chord in self.htail['chord_data']:
            y = pos * self.htail['span'] / 2 / 100
            x = htail_root_x + tail_length+ y * np.tan(self.htail['sweep'])
            htail_le_points.append((x, y))
            htail_te_points.append((x + chord, y))
        
        # Plot horizontal tail leading and trailing edges
        htail_x_le = [x for x, y in htail_le_points]
        htail_y_le = [y for x, y in htail_le_points]
        htail_x_te = [x for x, y in htail_te_points]
        htail_y_te = [y for x, y in htail_te_points]
        
        ax.plot(htail_x_le, htail_y_le, 'r-', linewidth=2)
        ax.plot(htail_x_te, htail_y_te, 'r-', linewidth=2)
        ax.plot(htail_x_le, [-y for y in htail_y_le], 'r-', linewidth=2)
        ax.plot(htail_x_te, [-y for y in htail_y_te], 'r-', linewidth=2)
        
        # Connect tail tips
        ax.plot([htail_x_le[-1], htail_x_te[-1]], [htail_y_le[-1], htail_y_te[-1]], 'r-', linewidth=2)
        ax.plot([htail_x_le[-1], htail_x_te[-1]], [-htail_y_le[-1], -htail_y_te[-1]], 'r-', linewidth=2)

            
        # Plot vertical tail base
        ax.plot([fus_length/2 + tail_length - self.vtail['span']/2, fus_length/2 +tail_length], 
                [0, 0], 'g-', linewidth=1)
        
        ax.axis('equal')
        ax.grid(True)
        return fig, ax


    def plot_side_view(self):
        """Generate side view of the aircraft"""
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.set_title('Aircraft Side View')
        
        # Plot fuselage
        height = self.fuselage['height']
        length = self.fuselage['length']
        ax.plot([-length/2, length/2], [0, 0], 'k-', linewidth=2)
        ax.plot([-length/2, length/2], [height, height], 'k-', linewidth=2)
        ax.plot([-length/2, -length/2], [0, height], 'k-', linewidth=2)
        ax.plot([length/2, length/2], [0, height], 'k-', linewidth=2)
        
        # Plot cockpit as right hemi-ellipse
        t = np.linspace(np.pi/2, 3*np.pi/2, 100)
        x = -length/2 + self.cockpit['length'] * np.cos(t)
        y = height/2 + (height/2) * np.sin(t)
        ax.plot(x, y, 'k-')
        
        # Plot wing root chord location
        wing_root_chord = self.wing['chord_data'][0][1]
        wing_start_x = -length/4  # Assuming wing starts at 1/4 of fuselage
        ax.plot([wing_start_x, wing_start_x + wing_root_chord], 
                [height/2, height/2], 'b-', linewidth=2)


               
        # Plot tail fuselage (side view - right triangle)
        tail_length = vtail_root = self.vtail['chord_data'][0][1]  # Use vertical tail root chord as length
        tail_height = height
        ax.plot([length/2, length/2+tail_length], [height - tail_height, height], 'k-', linewidth=2)
        ax.plot([length/2, length/2+tail_length], [height, height], 'k-', linewidth=2)
        
        # Plot vertical tail
        vtail_height = self.vtail['span']
        vtail_sweep = self.vtail['sweep']
        vtail_root = self.vtail['chord_data'][0][1]
        vtail_tip = self.vtail['chord_data'][-1][1]
        
     
        ax.plot([length/2, length/2 + vtail_height*np.tan(vtail_sweep)], [height, height + vtail_height], 'g-')
        ax.plot([length/2+vtail_root, length/2 + vtail_height*np.tan(vtail_sweep)+vtail_tip], [height, height + vtail_height], 'g-')
        ax.plot([length/2 + vtail_height*np.tan(vtail_sweep), length/2 + vtail_height*np.tan(vtail_sweep)+vtail_tip], [height + vtail_height, height + vtail_height], 'g-')


        # Plot horizontal tail root chord location
        htail_root_chord = self.htail['chord_data'][0][1]
        if self.htail['configuration'] is 'high':
            htail_start_x = length/2 + vtail_height*np.tan(vtail_sweep)  # At the tail end
            htail_y = height + vtail_height
        elif self.htail['configuration'] is 'low':
            htail_start_x = length/2 + vtail_root - htail_root_chord  # At the tail end
            htail_y = 0
        else:
            raise ValueError("Unknown configuration. Please use either low or high")
        #elif type(self.htail['configuration']) is float:

        ax.plot([htail_start_x, htail_start_x + htail_root_chord], 
                [htail_y, htail_y], 'r-', linewidth=2)
        
        ax.axis('equal')
        ax.grid(True)
        return fig, ax

    def plot_front_view(self):
        """Generate front view of the aircraft"""
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.set_title('Aircraft Front View')
        
        # Plot fuselage as an ellipse
        width = self.fuselage['width']
        height = self.fuselage['height']
        t = np.linspace(0, 2*np.pi, 100)
        ax.plot(width/2 * np.cos(t), height/2 + height/2 * np.sin(t), 'k-', linewidth=2)
        
        # Plot wings with dihedral
        half_span = self.wing['span'] / 2
        wing_height = half_span * np.tan(self.wing['dihedral'])
        ax.plot([0, half_span], [height/2, height/2 + wing_height], 'b-', linewidth=2)
        ax.plot([0, -half_span], [height/2, height/2 + wing_height], 'b-', linewidth=2)
        
        # Plot horizontal tail
        if self.htail['configuration'] is 'high':
            htail_height = height + self.vtail['span']
        elif self.htail['configuration'] is 'low':
            htail_height = 0
        else:
            raise ValueError("Unknown configuration. Please use either low or high")
        htail_span = self.htail['span'] / 2
        ax.plot([-htail_span, htail_span], [htail_height, htail_height], 'r-', linewidth=2)
        
        # Plot vertical tail
        ax.plot([0, 0], [height, height + self.vtail['span']], 'g-', linewidth=2)
        
        # Plot floor lines
        for h in np.cumsum(self.fuselage['floor_heights'])[:-1]:
            ax.plot([-width/2, width/2], [h, h], 'k--', linewidth=1)
            
        ax.axis('equal')
        ax.grid(True)
        return fig, ax

    def plot_cross_section(self):
        """Generate cross-section view of the aircraft with smooth fuselage shape"""
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.set_title('Aircraft Cross Section')
        
        width = self.fuselage['width']
        floor_heights = self.fuselage['floor_heights']
        y_positions = np.cumsum([0] + floor_heights)
        
        control_points = []
        print(y_positions)
        for floor_idx in range(len(floor_heights)):
            floor_y = y_positions[floor_idx]
            config = self.seating['configs'][floor_idx]
            print(floor_idx,floor_y)
            
            # Calculate seating layout
            seat_width = config['seat_width']
            seat_spacing = self.seating['spacing']
            effective_seat_width = seat_width + seat_spacing
            aisle_width = config['aisle_width']
            seats_per_section = config['seats_per_section']
            num_aisles = config['num_aisles']
            effective_seat_height = config['seat_height'] + config['headroom_height']
            # Calculate total width needed for seats and aisles
            section_width = seats_per_section * effective_seat_width
            total_width = section_width * (num_aisles) + aisle_width * num_aisles
            
            
            # Start position for first seat (centered)
            start_x = -total_width/2
            if floor_y < y_positions[-1] / 2:
                # Add control points at the base of each seat
                control_points.extend([
                    (-total_width/2, floor_y),
                    (total_width/2, floor_y)
                ])

            if (floor_y + effective_seat_height) > (y_positions[-1] / 2):
                print(floor_y)
                control_points.extend([
                    (-total_width/2, floor_y + effective_seat_height),
                    (total_width/2, floor_y + effective_seat_height)
                ])
            # Draw floor line
            ax.plot([-total_width/2, total_width/2], [floor_y, floor_y], 'k--', linewidth=1)
            

            # Draw seats and aisles
            for section in range(num_aisles):
                section_start = start_x + section * (section_width + aisle_width)
                
                # Draw first half of seats in this section
                for seat in range(int(seats_per_section/2)):
                    seat_x = section_start + seat * effective_seat_width + seat_spacing
                    rect = ax.Rectangle(
                        (seat_x, floor_y),
                        seat_width,
                        config['seat_height'],
                        fill=True,
                        color='blue',
                        alpha=0.5
                    )
                    ax.gca().add_patch(rect)
                
                aisle_start = seat_x + seat_width
                ax.plot(
                    [aisle_start, aisle_start + aisle_width],
                    [floor_y, floor_y],
                    'r-',
                    linewidth=2
                )
                for seat in range(int(seats_per_section/2),seats_per_section):
                    seat_x = section_start + seat * effective_seat_width + aisle_width
                    rect = ax.Rectangle(
                        (seat_x, floor_y),
                        seat_width,
                        config['seat_height'],
                        fill=True,
                        color='blue',
                        alpha=0.5
                    )
                    ax.gca().add_patch(rect)
        # Curves
        control_points.extend([
                (0, y_positions[0]-floor_heights[0]/2-1e-3),
                (0, y_positions[-1]+1e-3),
            ])
        control_points = np.array(control_points) # convert to numpy array so that indexing will work
        #print(control_points)
        # Generate smooth curve through control points
        from scipy.interpolate import make_interp_spline
        
        # Process control points
        left_points, right_points = process_control_points(control_points)
        print(left_points,right_points)
        # Generate smooth curves
        t_left = np.linspace(0, 1, len(left_points))
        t_right = np.linspace(0, 1, len(right_points))
        
        left_spline = make_interp_spline(t_left, left_points, k=3)
        right_spline = make_interp_spline(t_right, right_points, k=3)
        
        # Generate points along the curves
        t_smooth = np.linspace(0, 1, 100)
        left_smooth = left_spline(t_smooth)
        right_smooth = right_spline(t_smooth)
        
        # Plot fuselage outline
        ax.plot(left_smooth[:, 0], left_smooth[:, 1], 'k-', linewidth=2)
        ax.plot(right_smooth[:, 0], right_smooth[:, 1], 'k-', linewidth=2)

        ax.axis('equal')
        ax.grid(True)
        return fig, ax

    def plot_three_view(self):
        """Generate a three-view drawing of the aircraft with shared axes"""
        # Create figure with gridspec
        fig = plt.figure(figsize=(15, 15))
        gs = fig.add_gridspec(2, 2)
        
        # Create axes with shared scales
        ax_top = fig.add_subplot(gs[0, 0])    # Top view
        ax_side = fig.add_subplot(gs[0, 1])   # Side view
        ax_front = fig.add_subplot(gs[1, 0])  # Front view
        
        # Plot individual views
        _, ax_top = self.plot_top_view()
        _, ax_side = self.plot_side_view()
        _, ax_front = self.plot_front_view()
        
        # Set equal aspect ratio for all views
        ax_top.set_aspect('equal')
        ax_side.set_aspect('equal')
        ax_front.set_aspect('equal')
        
        # Ensure consistent scaling across views
        max_dim = max(
            self.fuselage['length'],
            self.wing['span'],
            self.fuselage['height'] * 2
        )
        
        # Set limits for each view
        margin = max_dim * 0.1  # 10% margin
        
        # Top view limits
        ax_top.set_xlim([-self.fuselage['length']/2 - margin, 
                         self.fuselage['length']/2 + margin])
        ax_top.set_ylim([-self.wing['span']/2 - margin, 
                         self.wing['span']/2 + margin])
        
        # Side view limits (share x-axis with top view)
        ax_side.set_xlim(ax_top.get_xlim())
        ax_side.set_ylim([0 - margin, 
                          self.fuselage['height'] + self.vtail['span'] + margin])
        
        # Front view limits (share y-axis with top view)
        ax_front.set_xlim([-self.wing['span']/2 - margin, 
                           self.wing['span']/2 + margin])
        ax_front.set_ylim(ax_side.get_ylim())
        
        # Add labels
        ax_top.set_xlabel('Length (m)')
        ax_top.set_ylabel('Span (m)')
        ax_side.set_xlabel('Length (m)')
        ax_side.set_ylabel('Height (m)')
        ax_front.set_xlabel('Span (m)')
        ax_front.set_ylabel('Height (m)')
        
        # Adjust layout
        plt.tight_layout()
        return fig

# Example usage:
def main():
    # Add seating configuration
    seat_configs = [
        {
            'seat_width': 9,
            'seat_depth': 2,    # meters
            'seat_height': 6,   # meters
            'headroom_height':0,
            'ceiling_height':7,
            'aisle_width': 1,   # meters
            'seats_per_section': 2,
            'num_aisles': 1,
            'cargo':True
        },
        
        {
            'seat_width': 1.5,
            'seat_depth': 2.6,
            'seat_height': 4,
            'headroom_height':1,
            'ceiling_height':7,
            'aisle_width': 2,
            'seats_per_section': 6,
            'num_aisles': 2,
        },
        {
            'seat_width': 1.5,
            'seat_depth': 2.6,
            'seat_height': 4,
            'headroom_height':1,
            'ceiling_height':7,
            'aisle_width': 2,
            'seats_per_section': 6,
            'num_aisles': 2,
        },
        {
            'seat_width': 2.5,
            'seat_depth': 2.6,
            'seat_height': 4,
            'headroom_height':1,
            'ceiling_height':7,
            'aisle_width': 2,
            'seats_per_section': 2,
            'num_aisles': 2,
        },

    ]
    galleys = [
        {'length': 20, 'number':2, 'floor': 1},
        {'length': 20, 'number':2, 'floor': 2}
    ]

    plotter = AircraftPlotter(
        total_passengers=1200,
        seat_configs=seat_configs,
        galleys=galleys,
        wingspan=311, #f
        sweep=37.5, #deg
        wing_chord_data=[(0, 65), (40, 45), (100, 15)],
        wing_thickness=0.12, dihedral=5,
        v_span=20, 
        v_sweep=35,
        v_chord_data=[(0, 60), (100, 40)],
        h_span=100,
        h_sweep=35,
        h_chord_data=[(0, 40), (100, 10)],
        tail_configuration='high',
        cockpit_length=20
    )

    print(plotter.to_dict())
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
    
    #fig, ax = plotter.plot_cross_section()
    #plt.savefig('assets/aircraft_cross_section.png', dpi=300, bbox_inches='tight')
    #plt.close()

    fig = plotter.plot_three_view()
    plt.savefig('assets/aircraft_three_view.png', dpi=300, bbox_inches='tight')
    plt.close()
    

if __name__ == "__main__":
    main()
    
    # fix wing placement along fuselage (top view)
    # add wing engines
    # CP and CG calculation
    # overhead bins
    # cargo volume check/assertion
    # thickness on the walls and floor
    
    # add dihedral (side view)
    # add sections on top view
    # add partial second floor
    # control surfaces
    # landing gear