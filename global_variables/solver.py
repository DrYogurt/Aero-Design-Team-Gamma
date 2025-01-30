from typing import Dict, List, Set, Tuple, Optional 
import networkx as nx
import sympy
from sympy import solve, symbols, lambdify, parse_expr
import re
from ambiance import Atmosphere
import warnings

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
            
            if var._function:
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
        """
        Creates a solver that handles intermediate values by solving equations in topological order.
        """
        # Create sympy symbols for all variables
        syms = {
            name: symbols(name.replace('{', '').replace('}', '')) 
            for name in self.inputs | self.intermediates | self.targets
        }
    
        # Get topological sort of variables to solve in correct order
        sorted_vars = list(nx.topological_sort(self.graph.graph))
        # Filter to only keep variables we need to solve
        sorted_vars = [v for v in sorted_vars if v in (self.intermediates | self.targets)]
    
        # Store intermediate solutions
        solutions = {}
    
        # Solve equations in order
        for var in sorted_vars:
            var_obj = self.registry._symbol_map[var][1]
            lhs = syms[var]
            
            if var_obj._function:
                # Handle function-based variable
                args_str = ", ".join(var_obj._function_args)
                solutions[var] = eval(f"lambda {args_str}: {var_obj.function_str}")
                continue
            
            # Get equations for this variable
            equations = self.equations.get(var, [])
            if not equations:
                continue
            
            # Try each equation definition until one works
            solved = False
            for eq in equations:
                try:
                    # Substitute any known solutions
                    eq_expr = parse_expr(eq)
                    for solved_var, solution in solutions.items():
                        if isinstance(solution, sympy.Expr):
                            eq_expr = eq_expr.subs(syms[solved_var], solution)
                
                    # Try to solve for current variable
                    solution = solve(lhs - eq_expr, lhs)
                    if solution:
                        solutions[var] = solution[0].simplify()  # Take first solution
                        # print(f"Solution for {var} found:\n{solution}")
                        solved = True
                        break
                except Exception as e:
                    print(f"Exception {e} occured while trying to solve variable {var}")
                    continue
                    
            if not solved:
                raise ValueError(f"Could not solve for variable {var}")
        
        # Create lambda functions for target variables
        solvers = {}
        input_syms = [syms[v] for v in sorted(self.inputs)]
        function_namespace = {'Atmosphere': Atmosphere}
        
        for target in self.targets:
            if target not in solutions:
                warnings.warn(f"No solution found for target {target}, treating as input")
                solvers[target] = lambda x: x  #treating as identity
            else:
                solvers[target] = lambdify(
                    input_syms,
                    solutions[target],
                    modules=[function_namespace, "numpy"]
                )
    
        return solvers#, solutions


    def complete_definitions(self, variables: Set[str]) -> Dict[str, Set[str]]:
        """
        For given variables:
        1. Take their definitions and extract all symbols used in them
        2. Solve each definition for those symbols
        3. Add unique solutions as new definitions
        
        Args:
            variables: Set of variable symbols to process definitions for
            
        Returns:
            Dict mapping variable symbols to their new definitions
        """
        new_definitions = {}
        
        # Create sympy symbols for all variables
        syms = {
            name: symbols(name.replace('{', '').replace('}', '')) 
            for name in self.inputs | self.intermediates | self.targets
        }
        
        # For each variable we want to process
        for var in variables:
            if var not in self.equations:
                continue
                
            # Get all definitions for this variable
            definitions = self.equations[var]
            
            # For each definition
            for eq in definitions:
                try:
                    # Parse equation and get all symbols used in it
                    expr = parse_expr(eq)
                    eq_symbols = {str(sym) for sym in expr.free_symbols}
                    
                    # Convert to standard form: expr = var
                    std_expr = expr - syms[var]
                    
                    # Try to solve for each symbol in the equation
                    for sym in eq_symbols:
                        if sym not in new_definitions:
                            new_definitions[sym] = set()
                            
                        try:
                            solutions = solve(std_expr, syms[sym])
                            if solutions:
                                # Add each unique solution
                                for sol in solutions:
                                    try:
                                        simplified = sol.simplify()
                                        # Check if this solution is unique
                                        solution_str = str(simplified)
                                        if solution_str not in self.registry._symbol_map[sym][1].definitions :
                                         new_definitions[sym].add(solution_str)
                                    except Exception as e:
                                        print(f"Failed to simplify solution for {sym}: {e}")
                        except Exception as e:
                            print(f"Failed to solve for {sym} in equation {eq}: {e}")
                except Exception as e:
                    print(f"Failed to parse equation {eq}: {e}")
     
        # Remove empty sets from new_definitions
        return {k: v for k, v in new_definitions.items() if v}
    
    
