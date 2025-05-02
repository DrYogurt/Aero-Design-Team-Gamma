import matplotlib.pyplot as plt
import networkx as nx
from typing import Optional, Dict, Any
from aircraft_design.core.base import Component

def plot_component_tree(component: Component, 
                       max_depth: int = 3,
                       figsize: tuple = (12, 8),
                       save_path: Optional[str] = None) -> None:
    """
    Plot a hierarchical tree representation of the aircraft components.
    
    Args:
        component: The root component (usually the Aircraft instance)
        max_depth: Maximum depth of the tree to display
        figsize: Figure size as (width, height)
        save_path: Optional path to save the figure
    """
    # Create a directed graph
    G = nx.DiGraph()
    
    def add_component_to_graph(current_component: Component, 
                             parent_name: Optional[str] = None, 
                             depth: int = 0) -> None:
        """Recursively add components to the graph"""
        if depth > max_depth:
            return
            
        # Add current component as a node
        component_name = '_'.join(current_component.name.split('_')[:2])
        G.add_node(component_name)
        
        # Add edge from parent if exists
        if parent_name is not None:
            G.add_edge(parent_name, component_name)
            
        # Add children recursively
        for child in current_component.children:
            add_component_to_graph(child, component_name, depth + 1)
    
    # Build the graph
    add_component_to_graph(component)
    
    # Create the plot
    plt.figure(figsize=figsize)
    
    # Use hierarchical layout
    pos = nx.spring_layout(G, k=2, iterations=50)
    
    # Draw the graph
    # Use hierarchical layout instead of spring layout
    pos = nx.nx_agraph.graphviz_layout(G, prog='dot')
    
    nx.draw(G, pos, 
            with_labels=True,
            node_color='lightblue',
            node_size=5000,
            font_size=10,
            font_weight='bold',
            arrows=True,
            edge_color='gray')
    plt.title(f"Aircraft Component Hierarchy (Max Depth: {max_depth})")
    
    # Save or show the plot
    if save_path:
        plt.savefig(save_path, bbox_inches='tight', dpi=300)
    else:
        plt.show()
    
    plt.close()


if __name__ == "__main__":
    from aircraft_design.final_design.final_construction import Aircraft
    plot_component_tree(Aircraft(),max_depth=2,save_path='assets/component_tree.png')