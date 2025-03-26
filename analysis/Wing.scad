// Wing Generator with NASA SC(2)-0714 Airfoil
// This script generates a wing using airfoil data from a .dat file

// Parameters for the wing
span = 315;             // Wing span in mm
tube_radius = 1;        // Radius of the quarter chord tube
num_sections = 500;      // Number of sections to create the wing
sweepback_angle = 32.5; // Sweepback angle in degrees
dihedral_angle = 4;     // Dihedral angle in degrees
show_quarter_chord = false; // Set to false to hide the quarter chord line

// Winglet parameters
winglet_start = 5;     // Distance from tip where winglet begins
winglet_angle = 60;     // Angle between winglet and main wing (degrees)
winglet_curvature = -.8; // How curved the winglet is (0=straight, 1=very curved)
one_wing = true;// whether to generate both or one wing (for use in simulations)


// Function for quarter chord point location [x, y, z] based on span position (y)
function quarter_chord_location(y) = 
    let(
        // Absolute position from wing root
        abs_y = abs(y),
        // Spanwise distance from tip
        tip_distance = span/2 - abs_y,
        // Normal wing sweep and dihedral calculations
        normal_x = abs(tan(sweepback_angle) * y),
        normal_z = abs(tan(dihedral_angle) * y),
        // Determine if we're in the winglet region
        is_winglet = tip_distance < winglet_start,
        // If in winglet, calculate how far into the winglet region we are (0-1)
        winglet_ratio = is_winglet ? (1 - tip_distance / winglet_start) : 0,
        // Apply curved transition using a sine function for smooth blend
        winglet_curve = is_winglet ? sin(90 * winglet_ratio * winglet_curvature) : 0,
        // Calculate winglet height with curved transition
        winglet_height = is_winglet ? 
            tan(winglet_angle) * winglet_ratio * winglet_start * (1 - winglet_curve) + 
            winglet_ratio * winglet_start * winglet_curve : 0,
        // Apply winglet effect to z component
        final_z = normal_z + winglet_height
    )
    [normal_x, y, final_z];

root_chord_length = 49.53;
tip_chord_length = 9.906;

// Function for chord length based on span position (y)
function chord_length(y) = 
    root_chord_length * (1 - abs(2*y/span)) + tip_chord_length * abs(2*y/span);
    

// NASA SC(2)-0714 Airfoil data
// Upper surface coordinates (x, z)
upper_surface = [
    [0.000000, 0.000000],
    [0.002000, 0.010770],
    [0.005000, 0.016580],
    [0.010000, 0.022400],
    [0.020000, 0.029600],
    [0.030000, 0.034600],
    [0.040000, 0.038300],
    [0.050000, 0.041400],
    [0.060000, 0.044000],
    [0.070000, 0.046300],
    [0.080000, 0.048400],
    [0.090000, 0.050200],
    [0.100000, 0.051900],
    [0.110000, 0.053500],
    [0.120000, 0.054900],
    [0.130000, 0.056200],
    [0.140000, 0.057400],
    [0.150000, 0.058600],
    [0.160000, 0.059700],
    [0.170000, 0.060700],
    [0.180000, 0.061600],
    [0.190000, 0.062500],
    [0.200000, 0.063300],
    [0.210000, 0.064100],
    [0.220000, 0.064800],
    [0.230000, 0.065400],
    [0.240000, 0.066000],
    [0.250000, 0.066500],
    [0.260000, 0.067000],
    [0.270000, 0.067500],
    [0.280000, 0.067900],
    [0.290000, 0.068300],
    [0.300000, 0.068600],
    [0.310000, 0.068900],
    [0.320000, 0.069200],
    [0.330000, 0.069400],
    [0.340000, 0.069600],
    [0.350000, 0.069700],
    [0.360000, 0.069800],
    [0.370000, 0.069900],
    [0.380000, 0.069900],
    [0.390000, 0.069900],
    [0.400000, 0.069900],
    [0.410000, 0.069800],
    [0.420000, 0.069700],
    [0.430000, 0.069600],
    [0.440000, 0.069500],
    [0.450000, 0.069300],
    [0.460000, 0.069100],
    [0.470000, 0.068900],
    [0.480000, 0.068600],
    [0.490000, 0.068300],
    [0.500000, 0.068000],
    [0.510000, 0.067600],
    [0.520000, 0.067200],
    [0.530000, 0.066800],
    [0.540000, 0.066300],
    [0.550000, 0.065800],
    [0.560000, 0.065300],
    [0.570000, 0.064700],
    [0.580000, 0.064100],
    [0.590000, 0.063500],
    [0.600000, 0.062800],
    [0.610000, 0.062100],
    [0.620000, 0.061300],
    [0.630000, 0.060500],
    [0.640000, 0.059700],
    [0.650000, 0.058800],
    [0.660000, 0.057900],
    [0.670000, 0.056900],
    [0.680000, 0.055900],
    [0.690000, 0.054800],
    [0.700000, 0.053700],
    [0.710000, 0.052500],
    [0.720000, 0.051300],
    [0.730000, 0.050000],
    [0.740000, 0.048700],
    [0.750000, 0.047300],
    [0.760000, 0.045800],
    [0.770000, 0.044300],
    [0.780000, 0.042700],
    [0.790000, 0.041100],
    [0.800000, 0.039400],
    [0.810000, 0.037600],
    [0.820000, 0.035800],
    [0.830000, 0.033900],
    [0.840000, 0.031900],
    [0.850000, 0.029900],
    [0.860000, 0.027800],
    [0.870000, 0.025600],
    [0.880000, 0.023400],
    [0.890000, 0.021100],
    [0.900000, 0.018700],
    [0.910000, 0.016200],
    [0.920000, 0.013700],
    [0.930000, 0.011100],
    [0.940000, 0.008400],
    [0.950000, 0.005600],
    [0.960000, 0.002700],
    [0.970000, -0.000200],
    [0.980000, -0.003200],
    [0.990000, -0.006300],
    [1.000000, -0.009500]
];

// Lower surface coordinates (x, z)
lower_surface = [
    [0.000000, 0.000000],
    [0.002000, -0.010770],
    [0.005000, -0.016580],
    [0.010000, -0.022400],
    [0.020000, -0.029600],
    [0.030000, -0.034500],
    [0.040000, -0.038200],
    [0.050000, -0.041300],
    [0.060000, -0.043900],
    [0.070000, -0.046200],
    [0.080000, -0.048300],
    [0.090000, -0.050100],
    [0.100000, -0.051800],
    [0.110000, -0.053400],
    [0.120000, -0.054900],
    [0.130000, -0.056200],
    [0.140000, -0.057400],
    [0.150000, -0.058600],
    [0.160000, -0.059700],
    [0.170000, -0.060700],
    [0.180000, -0.061600],
    [0.190000, -0.062500],
    [0.200000, -0.063300],
    [0.210000, -0.064100],
    [0.220000, -0.064800],
    [0.230000, -0.065500],
    [0.240000, -0.066100],
    [0.250000, -0.066700],
    [0.260000, -0.067200],
    [0.270000, -0.067700],
    [0.280000, -0.068100],
    [0.290000, -0.068500],
    [0.300000, -0.068800],
    [0.310000, -0.069100],
    [0.320000, -0.069300],
    [0.330000, -0.069500],
    [0.340000, -0.069600],
    [0.350000, -0.069700],
    [0.360000, -0.069700],
    [0.370000, -0.069700],
    [0.380000, -0.069600],
    [0.390000, -0.069500],
    [0.400000, -0.069300],
    [0.410000, -0.069100],
    [0.420000, -0.068800],
    [0.430000, -0.068500],
    [0.440000, -0.068100],
    [0.450000, -0.067700],
    [0.460000, -0.067200],
    [0.470000, -0.066700],
    [0.480000, -0.066100],
    [0.490000, -0.065400],
    [0.500000, -0.064600],
    [0.510000, -0.063700],
    [0.520000, -0.062700],
    [0.530000, -0.061600],
    [0.540000, -0.060400],
    [0.550000, -0.059100],
    [0.560000, -0.057700],
    [0.570000, -0.056200],
    [0.580000, -0.054600],
    [0.590000, -0.052900],
    [0.600000, -0.051100],
    [0.610000, -0.049200],
    [0.620000, -0.047300],
    [0.630000, -0.045300],
    [0.640000, -0.043300],
    [0.650000, -0.041200],
    [0.660000, -0.039100],
    [0.670000, -0.037000],
    [0.680000, -0.034800],
    [0.690000, -0.032600],
    [0.700000, -0.030400],
    [0.710000, -0.028200],
    [0.720000, -0.026000],
    [0.730000, -0.023800],
    [0.740000, -0.021600],
    [0.750000, -0.019400],
    [0.760000, -0.017300],
    [0.770000, -0.015200],
    [0.780000, -0.013200],
    [0.790000, -0.011300],
    [0.800000, -0.009500],
    [0.810000, -0.007900],
    [0.820000, -0.006400],
    [0.830000, -0.005000],
    [0.840000, -0.003800],
    [0.850000, -0.002800],
    [0.860000, -0.002000],
    [0.870000, -0.001400],
    [0.880000, -0.001000],
    [0.890000, -0.000800],
    [0.900000, -0.000900],
    [0.910000, -0.001200],
    [0.920000, -0.001700],
    [0.930000, -0.002500],
    [0.940000, -0.003600],
    [0.950000, -0.005000],
    [0.960000, -0.006700],
    [0.970000, -0.008700],
    [0.980000, -0.011000],
    [0.990000, -0.013600],
    [1.000000, -0.016500]
];

// Create a 2D airfoil polygon from the data points
module airfoil_2d(scale_factor = 1) {
    scaled_upper = [for (point = upper_surface) [point[0] * scale_factor, point[1] * scale_factor]];
    scaled_lower = [for (point = lower_surface) [point[0] * scale_factor, point[1] * scale_factor]];
    
    // Create polygon points by combining upper and lower surfaces
    points = concat(
        scaled_upper,
        [for (i = [len(scaled_lower)-1:-1:0]) scaled_lower[i]]
    );
    
    polygon(points);
}

// Generate the quarter chord line tube for visualization
module quarter_chord_line() {
    if (show_quarter_chord) {
        for (i = [0:num_sections-2]) {
            // Calculate the span positions for this segment
            y1 = -span/2 + i * span / (num_sections-1);
            y2 = -span/2 + (i+1) * span / (num_sections-1);
            
            // Get the quarter chord points
            qc1 = quarter_chord_location(y1);
            qc2 = quarter_chord_location(y2);
            
            // Create a hull between spheres at each point to form a tube segment
            color("red")
            hull() {
                translate([qc1[0], qc1[1], qc1[2]])
                sphere(r=tube_radius, $fn=16);
                
                translate([qc2[0], qc2[1], qc2[2]])
                sphere(r=tube_radius, $fn=16);
            }
        }
    }
}

// Generate the wing by creating airfoil sections
module wing() {
    // Define the span range based on the one_wing flag
    y_start = one_wing ? 0 : -span/2;
    y_end = span/2;
    
    // Create several cross-sections along the span
    for (i = [0:num_sections-2]) {
        // Calculate the span positions for this segment
        y1 = y_start + i * (y_end - y_start) / (num_sections-1);
        y2 = y_start + (i+1) * (y_end - y_start) / (num_sections-1);
        
        // Get the quarter chord points and chord lengths at these positions
        qc1 = quarter_chord_location(y1);
        qc2 = quarter_chord_location(y2);
        chord1 = chord_length(y1);
        chord2 = chord_length(y2);
        
        // Create a hull between adjacent airfoil cross-sections
        hull() {
            // First airfoil section
            translate([qc1[0] - 0.25 * chord1, qc1[1], qc1[2]])
            //rotate([0, 0, 90]) // Rotate to align with Y axis
            rotate([90, 0, 0]) // Rotate to make airfoil stand vertically
            linear_extrude(height=0.1)
            airfoil_2d(chord1);
            
            // Second airfoil section
            translate([qc2[0] - 0.25 * chord2, qc2[1], qc2[2]])
            //rotate([0, 0, 90]) // Rotate to align with Y axis
            rotate([90, 0, 0]) // Rotate to make airfoil stand vertically
            linear_extrude(height=0.1)
            airfoil_2d(chord2);
        }
    }
}

// Mark span positions with small spheres
module mark_span_positions() {
    points = 5;  // Number of marked positions
    for (i = [0:points]) {
        y = -span/2 + i * span / points;
        qc = quarter_chord_location(y);
        translate([qc[0], qc[1], qc[2]])
        color("blue")
        sphere(r=tube_radius*1.5, $fn=16);
        
        // Optionally display the chord length at this point
        echo(str("At y=", y, " chord length=", chord_length(y), "mm"));
    }
}

// Render the wing and quarter chord line
wing();
quarter_chord_line();

// Uncomment to see marked positions along the span
// mark_span_positions();