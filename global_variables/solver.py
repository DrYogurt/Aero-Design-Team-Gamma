from typing import Dict, List, Set, Tuple, Callable
import networkx as nx
from sympy import solve, symbols, lambdify, parse_expr
import re
from ambiance import Atmosphere

class EquationGraph:
    def __init__(self, registry):
        self.registry = registry
        self.graph = nx.DiGraph()
        self._build_graph()
    
    def _build_graph(self):
        for symbol, (path, var) in self.registry._symbol_map.items():
            self.graph.add_node(symbol, variable=var)
            
            if var.definitions:
                for eq in var.definitions:
                    for dep_symbol in self.registry._extract_symbols(eq):
                        if dep_symbol != symbol:
                            self.graph.add_edge(dep_symbol, symbol)
            
            if var.function:
                for arg in var.function_args:
                    self.graph.add_edge(arg, symbol)
    
    def get_required_variables(self, targets: Set[str]) -> Tuple[Set[str], Set[str]]:
        required = set()
        intermediates = set()
        
        for target in targets:
            predecessors = nx.ancestors(self.graph, target)
            required.update(predecessors)
            intermediates.update(
                sym for sym in predecessors 
                if self.registry._symbol_map[sym][1].definitions
            )
        
        required -= intermediates
        return required, intermediates

class EquationSystem:
    def __init__(self, registry, targets: Set[str]):
        self.registry = registry
        self.targets = targets
        self.graph = EquationGraph(registry)
        self.inputs, self.intermediates = self.graph.get_required_variables(targets)
        self.equations = self._collect_equations()
    
    def _collect_equations(self) -> Dict[str, List[str]]:
        equations = {}
        for var in self.intermediates | self.targets:
            var_obj = self.registry._symbol_map[var][1]
            if var_obj.definitions:
                equations[var] = var_obj.definitions
        return equations
    
    def create_solver(self) -> Dict[str, callable]:
        # Create sympy symbols for all variables
        syms = {
            name: symbols(name.replace('{', '').replace('}', '')) 
            for name in self.inputs | self.intermediates | self.targets
        }
        
        # Create equations list
        eqns = []
        for var, def_list in self.equations.items():
            var_obj = self.registry._symbol_map[var][1]
            lhs = syms[var]
            
            if var_obj.function:
                # Handle function-based variable
                args_str = ", ".join(var_obj.function_args)
                rhs = eval(f"lambda {args_str}: {var_obj.function_str}")
            else:
                # Handle equation-based variable
                for def_eq in def_list:
                    try:
                        rhs = parse_expr(def_eq)
                        eqns.append(lhs - rhs)
                    except Exception as e:
                        raise ValueError(f"Failed to parse equation '{def_eq}': {str(e)}")
        
        # Solve system
        solution = solve(eqns, [syms[t] for t in self.targets])
         
        # Create lambda functions
        solvers = {}
        input_syms = [syms[v] for v in sorted(self.inputs)]
        function_namespace = {'Atmosphere': Atmosphere}
        
        for target, expr in zip(self.targets, solution):
            solvers[target] = lambdify(
                input_syms,
                expr,
                [function_namespace, "numpy"]
            )
        
        return solvers, solution
