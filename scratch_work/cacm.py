import numpy as np

def cl(weight, rho, v, s=9812.2):
    cl = 2 * weight / (rho * v**2 * s)
    #print(f"cl: {cl}")
    return cl


def cd(weight, rho, v, s=9812.2, cd0=0.017, e=0.88, ar=10.112):
    cd = 0.5 * cd0 + cl(weight, rho, v, s)**2 / (np.pi * e * ar)
    #print(f"cd: {cd}")
    return cd

def cl_cd_ratio(weight_full, weight_empty, rho, v, s=9812.2, cd0=0.017, e=0.88, ar=10.112):
    cl_full = cl(weight_full, rho, v, s)
    cl_empty = cl(weight_empty, rho, v, s)
    cd_full = cd(weight_full, rho, v, s, cd0, e, ar)
    cd_empty = cd(weight_empty, rho, v, s, cd0, e, ar)
    #print(f"cl_full: {cl_full}, cl_empty: {cl_empty}, cd_full: {cd_full}, cd_empty: {cd_empty}")
    return 0.5 * (cl_full**(1/2)/cd_full + cl_empty**(1/2)/cd_empty)


def cruise_range(weight_full, weight_empty, rho, v, s=9812.2, tsfc=0.455, cd0=0.017, e=0.88, ar=10.112):
    cl_cd = cl_cd_ratio(weight_full, weight_empty, rho, v, s, cd0, e, ar)
    return (2*3600/(tsfc)) * (2/(rho*s))**(1/2) * cl_cd * ((weight_full)**(1/2)-(weight_empty)**(1/2))


def plot_range_vs_profit(weight_empty, weight_max, rho, v, s=9812.2, tsfc=0.455, cd0=0.017, e=0.88, ar=10.112, 
                         cost_per_nm=62.02, price_per_cargo_lb_nm=0.0004, range_values=None, fuel_density=6.7):
    """
    Plot range on the x-axis and profit on the y-axis.
    
    Parameters:
    -----------
    weight_empty : float
        Empty weight of the aircraft in lbs
    weight_max : float
        Maximum takeoff weight of the aircraft in lbs
    rho : float
        Air density in slugs/ft^3
    v : float
        Cruise velocity in ft/s
    s : float, optional
        Wing area in ft^2
    tsfc : float, optional
        Thrust specific fuel consumption in lb/(lbf·hr)
    cd0 : float, optional
        Zero-lift drag coefficient
    e : float, optional
        Oswald efficiency factor
    ar : float, optional
        Aspect ratio
    cost_per_nm : float, optional
        Cost to fly one nautical mile in dollars
    price_per_cargo_lb_nm : float, optional
        Price charged per pound of cargo per nautical mile in dollars
    range_values : array-like, optional
        Range values to evaluate in nautical miles. If None, a default range is used.
    fuel_density : float, optional
        Density of fuel in lbs/gallon
        
    Returns:
    --------
    tuple
        (range_values, profit_values) arrays for plotting
    """
    import matplotlib.pyplot as plt
    from scipy.optimize import minimize
    
    # Convert nautical miles to feet for calculations
    nm_to_feet = 6076.12
    
    if range_values is None:
        # Default range values from 0 to maximum possible range
        max_range_nm = cruise_range(weight_max, weight_empty, rho, v, s, tsfc, cd0, e, ar) / nm_to_feet
        print(max_range_nm)
        range_values = np.linspace(0, min(max_range_nm*.99,1.5e4), 100) # 95% of max range to avoid numerical issues
    profit_values = []
    cargo_weights = []
    fuel_weights = []
    
    for target_range_nm in range_values:
        target_range_ft = target_range_nm * nm_to_feet
        
        # Function to solve for the fuel weight needed to achieve the target range
        def range_equation(fuel_weight):
            if fuel_weight <= 0 or fuel_weight >= (weight_max - weight_empty):
                return np.inf  # Return a large value for invalid fuel weights
            
            weight_with_cargo = weight_max - fuel_weight
            return (cruise_range(weight_max, weight_with_cargo, 
                               rho, v, s, tsfc, cd0, e, ar) - target_range_ft)**2
        
        # Initial guess: half of available weight for fuel
        initial_guess = (weight_max - weight_empty) * .8
        
        try:
            # Solve for required fuel weight
            fuel_weight = minimize(range_equation, initial_guess, method='Nelder-Mead').x[0]
            
            # Ensure fuel weight is within valid bounds
            #fuel_weight = max(0, min(fuel_weight, weight_max - weight_empty))
            
            # Calculate cargo weight
            cargo_weight = weight_max - weight_empty - fuel_weight
            
            # Calculate revenue from cargo
            revenue = cargo_weight * price_per_cargo_lb_nm * target_range_nm
            
            # Calculate cost of flight
            cost = cost_per_nm * target_range_nm
            
            # Calculate profit
            profit = revenue - cost
            
            profit_values.append(profit/1e6)
            cargo_weights.append(cargo_weight)
            fuel_weights.append(fuel_weight)
            
        except:
            # If optimization fails, append NaN
            profit_values.append(np.nan)
            cargo_weights.append(np.nan)
            fuel_weights.append(np.nan)
    
    # Create the plot
    plt.figure(figsize=(12, 8))
    
    # Plot profit vs range
    plt.subplot(2, 1, 1)
    plt.plot(range_values, profit_values, 'b-', linewidth=2)
    plt.grid(True)
    plt.xlabel('Range (nautical miles)')
    plt.ylabel('Profit ($ MM)')
    plt.title('Profit vs Range')
    
    # Plot cargo and fuel weights
    plt.subplot(2, 1, 2)
    plt.plot(range_values, cargo_weights, 'g-', label='Cargo Weight (lbs)')
    plt.plot(range_values, fuel_weights, 'r-', label='Fuel Weight (lbs)')
    plt.plot(range_values, np.array(fuel_weights) / fuel_density, 'r--', label='Fuel Volume (gallons)')
    plt.grid(True)
    plt.xlabel('Range (nautical miles)')
    plt.ylabel('Weight (lbs) / Volume (gallons)')
    plt.legend()
    plt.title('Cargo and Fuel Allocation vs Range')
    
    plt.tight_layout()
    plt.savefig('assets/range_vs_profit.png')
    
    return range_values, profit_values


def find_max_profitability(weight_empty, weight_max, rho, v, s=9812.2, tsfc=0.455, cd0=0.017, e=0.88, ar=10.112, 
                         cost_per_nm=62.02, price_per_cargo_lb_nm=0.0004, fuel_density=6.7):
    """
    Find the range that maximizes profitability and return the associated metrics.
    
    Parameters:
    -----------
    Same as plot_range_vs_profit function
    
    Returns:
    --------
    dict
        Dictionary containing:
        - optimal_range_nm: Range at maximum profit (nautical miles)
        - max_profit: Maximum profit achieved ($)
        - total_revenue: Total revenue at maximum profit ($)
        - total_cost: Total cost at maximum profit ($)
        - cargo_weight: Cargo weight at maximum profit (lbs)
        - fuel_weight: Fuel weight at maximum profit (lbs)
        - fuel_volume: Fuel volume at maximum profit (gallons)
    """
    import numpy as np
    from scipy.optimize import minimize
    
    # Convert nautical miles to feet for calculations
    nm_to_feet = 6076.12
    
    def calculate_profit(target_range_nm):
        target_range_ft = target_range_nm * nm_to_feet
        
        # Function to solve for the fuel weight needed to achieve the target range
        def range_equation(fuel_weight):
            if fuel_weight <= 0 or fuel_weight >= (weight_max - weight_empty):
                return np.inf  # Return a large value for invalid fuel weights
            
            weight_with_cargo = weight_max - fuel_weight
            return (cruise_range(weight_max, weight_with_cargo, 
                               rho, v, s, tsfc, cd0, e, ar) - target_range_ft)**2
        
        # Initial guess: half of available weight for fuel
        initial_guess = (weight_max - weight_empty) * 0.8
        
        try:
            # Solve for required fuel weight
            fuel_weight = minimize(range_equation, initial_guess, method='Nelder-Mead').x[0]
            
            # Calculate cargo weight
            cargo_weight = weight_max - weight_empty - fuel_weight
            
            # Calculate revenue from cargo
            revenue = cargo_weight * price_per_cargo_lb_nm * target_range_nm
            
            # Calculate cost of flight
            cost = cost_per_nm * target_range_nm
            
            # Return negative profit for minimization
            return -(revenue - cost)
            
        except:
            return np.inf
    
    # Find the range that maximizes profit
    initial_range_guess = 5000  # Initial guess in nautical miles
    result = minimize(calculate_profit, initial_range_guess, method='Nelder-Mead')
    optimal_range_nm = result.x[0]
    
    # Calculate final metrics at optimal range
    target_range_ft = optimal_range_nm * nm_to_feet
    
    def range_equation(fuel_weight):
        weight_with_cargo = weight_max - fuel_weight
        return (cruise_range(weight_max, weight_with_cargo, 
                           rho, v, s, tsfc, cd0, e, ar) - target_range_ft)**2
    
    fuel_weight = minimize(range_equation, (weight_max - weight_empty) * 0.8, method='Nelder-Mead').x[0]
    cargo_weight = weight_max - weight_empty - fuel_weight
    total_revenue = cargo_weight * price_per_cargo_lb_nm * optimal_range_nm
    total_cost = cost_per_nm * optimal_range_nm
    max_profit = total_revenue - total_cost
    
    return {
        'optimal_range_nm': optimal_range_nm,
        'max_profit': max_profit,
        'total_revenue': total_revenue,
        'total_cost': total_cost,
        'cargo_weight': cargo_weight,
        'fuel_weight': fuel_weight,
        'fuel_volume': fuel_weight / fuel_density
    }

def plot_range_vs_altitude_speed(weight_empty, weight_max, s=9812.2, tsfc=0.455, cd0=0.017, e=0.88, ar=10.112):
    """
    Create a 2D contour plot showing how range varies with altitude and speed.
    
    Parameters:
    -----------
    weight_empty : float
        Empty weight of the aircraft in lbs
    weight_max : float
        Maximum takeoff weight of the aircraft in lbs
    s : float, optional
        Wing area in ft^2
    tsfc : float, optional
        Thrust specific fuel consumption in lb/(lbf·hr)
    cd0 : float, optional
        Zero-lift drag coefficient
    e : float, optional
        Oswald efficiency factor
    ar : float, optional
        Aspect ratio
    """
    import matplotlib.pyplot as plt
    from ambiance import Atmosphere
    
    # Create grid of altitude and speed values
    altitudes = np.linspace(10000, 40000, 50)  # feet
    speeds = np.linspace(400, 1000, 50)  # ft/s
    
    # Create meshgrid for plotting
    H, V = np.meshgrid(altitudes, speeds)
    R = np.zeros_like(H)
    M = np.zeros_like(H)  # Mach number grid
    
    # Calculate range and Mach number for each combination
    for i in range(len(altitudes)):
        # Get atmospheric properties at this altitude
        atm = Atmosphere(altitudes[i] * 0.3048)  # Convert feet to meters
        rho = atm.density[0] * 0.062428  # Convert kg/m^3 to slugs/ft^3
        a = atm.speed_of_sound[0] * 3.28084  # Convert m/s to ft/s
        
        for j in range(len(speeds)):
            R[j,i] = cruise_range(weight_max, weight_empty, rho, speeds[j], s, tsfc, cd0, e, ar) / 6076.12  # Convert to nautical miles
            M[j,i] = speeds[j] / a  # Calculate Mach number
    
    # Create the plot
    plt.figure(figsize=(12, 8))
    
    # Create contour plot
    levels = np.linspace(np.nanmin(R), np.nanmax(R), 20)
    contour = plt.contourf(H, V, R, levels=levels, cmap='viridis')
    
    # Add contour lines
    CS = plt.contour(H, V, R, levels=levels, colors='k', linewidths=0.5)
    plt.clabel(CS, inline=True, fontsize=8)
    
    # Add Mach 1 line
    M1 = plt.contour(H, V, M, levels=[0.5,0.7,0.9, 1.0], colors='white', linewidths=2)
    plt.clabel(M1, inline=True, fontsize=10, fmt='M=%.2f')
    
    # Add colorbar
    cbar = plt.colorbar(contour)
    cbar.set_label('Range (nautical miles)')
    
    # Set labels
    plt.xlabel('Altitude (feet)')
    plt.ylabel('Speed (ft/s)')
    plt.title('Range vs Altitude and Speed')
    
    # Add grid
    plt.grid(True, linestyle='--', alpha=0.7)
    
    plt.tight_layout()
    plt.savefig('assets/range_vs_altitude_speed_contour.png')
    
    return H, V, R

if __name__ == "__main__":
    # Plot range vs altitude and speed
    plot_range_vs_altitude_speed(weight_empty=9.7212e5, weight_max=1.575e6)
    
    # Original plots
    #plot_range_vs_profit(weight_empty=7.07e5, weight_max=1.575e6, rho=0.0007103, v=871.830)
    #max_profit = find_max_profitability(weight_empty=7.07e5, weight_max=1.575e6, rho=0.0007103, v=871.830)
    #print(max_profit)