import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from scipy.interpolate import griddata
from aircraft_design.final_design.final_trade_studies.dynamic_stability_trade_study import analyze_aircraft_dynamic_stability
from aircraft_design.final_design.final_construction import Aircraft

def create_corner_plot(samples, output_values, setpoint, output_name, figsize=(10, 10), n_contours=20, n_grid=50):
    """
    Create a corner plot with contours showing how output varies with input pairs.
    """
    # Create diverging colormap centered at setpoint
    colors = ['darkred', 'red', 'lightcoral', 'white', 'lightgreen', 'green', 'darkgreen']
    custom_cmap = LinearSegmentedColormap.from_list('custom', colors, N=256)

    input_vars = list(samples.keys())
    n_vars = len(input_vars)

    # Create figure with small margins
    fig, axes = plt.subplots(n_vars, n_vars, figsize=figsize)
    plt.subplots_adjust(wspace=0.1, hspace=0.1)

    # Calculate differences and normalize
    differences = output_values - setpoint
    rel_diffs = differences / abs(setpoint) * 100  # Convert to percentage
    max_rel_diff = max(abs(np.min(rel_diffs)), abs(np.max(rel_diffs)))

    # Create symmetric levels around zero in percentage terms
    level_step = max_rel_diff / (n_contours // 2)
    levels = np.concatenate([
        np.arange(-max_rel_diff, 0, level_step),
        np.array([0]),
        np.arange(level_step, max_rel_diff + level_step, level_step)
    ])

    # Create the corner plot
    for i in range(n_vars):
        for j in range(n_vars):
            ax = axes[i, j]

            # Remove top and right spines for all plots
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)

            if i < j:  # Upper triangle - leave empty
                ax.axis('off')
                continue

            elif i == j:  # Diagonal - histogram
                ax.hist(samples[input_vars[i]], bins=30, density=True)
                if i == n_vars-1:  # Only bottom row gets x labels
                    ax.set_xlabel(input_vars[i])
                else:
                    ax.set_xticklabels([])

                if j == 0:  # Only first column gets y labels
                    ax.set_ylabel('Density')
                else:
                    ax.set_yticklabels([])

            else:  # Lower triangle - contour plots
                x = samples[input_vars[j]]
                y = samples[input_vars[i]]

                # Create grid for contour plot
                xi = np.linspace(x.min(), x.max(), n_grid)
                yi = np.linspace(y.min(), y.max(), n_grid)
                xi, yi = np.meshgrid(xi, yi)

                # Interpolate relative differences on the grid
                zi = griddata((x, y), rel_diffs, (xi, yi), method='cubic')

                # Create contour plot
                contour = ax.contourf(xi, yi, zi, levels=levels, cmap=custom_cmap, extend='both')

                # Add colorbar only for rightmost plots
                if j == n_vars-1:
                    cbar = plt.colorbar(contour, ax=ax, ticks=levels[::2])
                    cbar.set_label('% Difference from setpoint')

                # Add setpoint contour in black
                ax.contour(xi, yi, zi, levels=[0], colors='black', linestyles='dashed')

                # Only add x labels to bottom row
                if i == n_vars-1:
                    ax.set_xlabel(input_vars[j])
                else:
                    ax.set_xticklabels([])

                # Only add y labels to leftmost column
                if j == 0:
                    ax.set_ylabel(input_vars[i])
                else:
                    ax.set_yticklabels([])

    plt.suptitle(f'Parameter Effects on {output_name} (Setpoint: {setpoint})', y=1.02)
    return fig, axes

def analyze_longitudinal_gains(aircraft, gain_range=np.linspace(-1, 1, 50), target_values=None):
    """
    Analyze and plot the sensitivity of longitudinal gains.
    """
    # Define longitudinal gains and their components
    gains = {
        'ku_P': 'Velocity P gain',
        'ku_I': 'Velocity I gain',
        'kw_P': 'Vertical Velocity P gain',
        'kw_I': 'Vertical Velocity I gain',
        'kq_P': 'Pitch Rate P gain',
        'kq_I': 'Pitch Rate I gain',
        'ko_long_P': 'Longitudinal Offset P gain',
        'ko_long_I': 'Longitudinal Offset I gain'
    }
    
    # Initialize samples dictionary
    samples = {name: [] for name in gains.values()}
    results = {
        'short_nf': [],
        'short_df': [],
        'p_nf': [],
        'p_df': []
    }
    
    # Generate samples
    for ku_p in gain_range:
        for ku_i in gain_range:
            for kw_p in gain_range:
                for kw_i in gain_range:
                    for kq_p in gain_range:
                        for kq_i in gain_range:
                            for ko_p in gain_range:
                                for ko_i in gain_range:
                                    # Store sample values
                                    samples['Velocity P gain'].append(ku_p)
                                    samples['Velocity I gain'].append(ku_i)
                                    samples['Vertical Velocity P gain'].append(kw_p)
                                    samples['Vertical Velocity I gain'].append(kw_i)
                                    samples['Pitch Rate P gain'].append(kq_p)
                                    samples['Pitch Rate I gain'].append(kq_i)
                                    samples['Longitudinal Offset P gain'].append(ko_p)
                                    samples['Longitudinal Offset I gain'].append(ko_i)
                                    
                                    # Run analysis
                                    short_nf, short_df, p_nf, p_df, _, _, _, _ = analyze_aircraft_dynamic_stability(
                                        aircraft,
                                        ku=[ku_p, ku_i],
                                        kw=[kw_p, kw_i],
                                        kq=[kq_p, kq_i],
                                        ko_long=[ko_p, ko_i],
                                        kv=[0,0], kp=[0,0], kr=[0,0], ko_lat=[0,0]
                                    )
                                    
                                    # Store results
                                    results['short_nf'].append(short_nf)
                                    results['short_df'].append(short_df)
                                    results['p_nf'].append(p_nf)
                                    results['p_df'].append(p_df)
    
    # Create plots for each output
    for output_name, output_values in results.items():
        setpoint = target_values.get(output_name, 0) if target_values else 0
        fig, _ = create_corner_plot(samples, np.array(output_values), setpoint, output_name)
        plt.savefig(f'longitudinal_{output_name}_sensitivity.png')
        plt.close()

def analyze_lateral_gains(aircraft, gain_range=np.linspace(-1, 1, 50), target_values=None):
    """
    Analyze and plot the sensitivity of lateral gains.
    """
    # Define lateral gains and their components
    gains = {
        'kv_P': 'Lateral Velocity P gain',
        'kv_I': 'Lateral Velocity I gain',
        'kp_P': 'Roll Rate P gain',
        'kp_I': 'Roll Rate I gain',
        'kr_P': 'Yaw Rate P gain',
        'kr_I': 'Yaw Rate I gain',
        'ko_lat_P': 'Lateral Offset P gain',
        'ko_lat_I': 'Lateral Offset I gain'
    }
    
    # Initialize samples dictionary
    samples = {name: [] for name in gains.values()}
    results = {
        'dnf': [],
        'ddr': [],
        'Tr': [],
        'Ts': []
    }
    
    # Generate samples
    for kv_p in gain_range:
        for kv_i in gain_range:
            for kp_p in gain_range:
                for kp_i in gain_range:
                    for kr_p in gain_range:
                        for kr_i in gain_range:
                            for ko_p in gain_range:
                                for ko_i in gain_range:
                                    # Store sample values
                                    samples['Lateral Velocity P gain'].append(kv_p)
                                    samples['Lateral Velocity I gain'].append(kv_i)
                                    samples['Roll Rate P gain'].append(kp_p)
                                    samples['Roll Rate I gain'].append(kp_i)
                                    samples['Yaw Rate P gain'].append(kr_p)
                                    samples['Yaw Rate I gain'].append(kr_i)
                                    samples['Lateral Offset P gain'].append(ko_p)
                                    samples['Lateral Offset I gain'].append(ko_i)
                                    
                                    # Run analysis
                                    _, _, _, _, dnf, ddr, Tr, Ts = analyze_aircraft_dynamic_stability(
                                        aircraft,
                                        ku=[0,0], kw=[0,0], kq=[0,0], ko_long=[0,0],
                                        kv=[kv_p, kv_i],
                                        kp=[kp_p, kp_i],
                                        kr=[kr_p, kr_i],
                                        ko_lat=[ko_p, ko_i]
                                    )
                                    
                                    # Store results
                                    results['dnf'].append(dnf)
                                    results['ddr'].append(ddr)
                                    results['Tr'].append(Tr)
                                    results['Ts'].append(Ts)
    
    # Create plots for each output
    for output_name, output_values in results.items():
        setpoint = target_values.get(output_name, 0) if target_values else 0
        fig, _ = create_corner_plot(samples, np.array(output_values), setpoint, output_name)
        plt.savefig(f'lateral_{output_name}_sensitivity.png')
        plt.close()

if __name__ == "__main__":
    # Create aircraft instance
    aircraft = Aircraft()
    
    # Define target values
    target_values = {
        'short_nf': 3.0,  # Target natural frequency for short period mode
        'short_df': 0.7,  # Target damping ratio for short period mode
        'p_nf': 1e-9,     # Target natural frequency for phugoid mode
        'p_df': 0.04,     # Target damping ratio for phugoid mode
        'dnf': 0.5,       # Target natural frequency for Dutch roll mode
        'ddr': 0.08,      # Target damping ratio for Dutch roll mode
        'Tr': 1.4,        # Target time constant for roll mode
        'Ts': 28.9        # Target time constant for spiral mode
    }
    
    # Analyze longitudinal and lateral gains separately
    print("Analyzing longitudinal gains...")
    analyze_longitudinal_gains(aircraft, gain_range=np.linspace(-1, 1, 10), target_values=target_values)
    
    print("Analyzing lateral gains...")
    analyze_lateral_gains(aircraft, gain_range=np.linspace(-1, 1, 10), target_values=target_values) 