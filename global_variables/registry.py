import os
import yaml
import re
from typing import Dict, Optional, List, Union, Callable, Set
import warnings
import sympy

class Variable:
    def __init__(
        self,
        name: str,
        symbol: str,
        description: Optional[str] = None,
        minimum: Optional[float] = None,
        maximum: Optional[float] = None,
        definitions: Optional[List[str]] = None,
        function_str: Optional[str] = None,
        function_args: Optional[List[str]] = None,
        assumed_value: Optional[float] = None,
        real_value: Optional[float] = None
    ):
        self.name = name
        self.symbol = symbol
        self.description = description
        self.minimum = minimum
        self.maximum = maximum
        self.definitions = definitions or []
        self.function_str = function_str
        self.function_args = function_args or []
        self.assumed_value = assumed_value
        self.real_value = real_value
        self._function = None
        if function_str:
            self._function = eval(function_str)

    def to_dict(self) -> Dict:
        data = {
            'name': self.name,
            'symbol': self.symbol,
        }
        if self.description:
            data['description'] = self.description
        if self.minimum is not None:
            data['minimum'] = self.minimum
        if self.maximum is not None:
            data['maximum'] = self.maximum
        if self.definitions:
            data['definitions'] = self.definitions
        if self.function_str:
            data['function_str'] = self.function_str
            data['function_args'] = self.function_args
        if self.assumed_value is not None:
            data['assumed_value'] = self.assumed_value
        if self.real_value is not None:
            data['real_value'] = self.real_value
        return data

    @classmethod
    def from_dict(cls, data: Dict) -> 'Variable':
        return cls(
            name=data['name'],
            symbol=data['symbol'],
            description=data.get('description'),
            minimum=data.get('minimum'),
            maximum=data.get('maximum'),
            definitions=data.get('definitions'),
            function_str=data.get('function_str'),
            function_args=data.get('function_args'),
            assumed_value=data.get('assumed_value'),
            real_value=data.get('real_value')
        )
class RegistryNode:
    def __init__(self, parent=None, name=None):
        self._children = {}
        self._variable = None
        self._parent = parent
        self._name = name
        
    def _get_full_path(self):
        if self._parent is None or self._parent._name is None:
            return self._name or ""
        parent_path = self._parent._get_full_path()
        return f"{parent_path}.{self._name}" if parent_path else self._name

    def _collect_variables(self) -> List[tuple[str, Variable]]:
        variables = []
        if self._variable is not None:
            variables.append((self._get_full_path(), self._variable))
        for child in self._children.values():
            variables.extend(child._collect_variables())
        return variables

    def __getattr__(self, name: str) -> Union['RegistryNode', Variable]:
        if name.startswith('_'):
            return super().__getattr__(name)
        if name not in self._children:
            self._children[name] = RegistryNode(parent=self, name=name)
        return self._children[name]

    def __setattr__(self, name: str, value: Union[Variable, Dict]):
        if name.startswith('_'):
            super().__setattr__(name, value)
            return

        if isinstance(value, Variable):
            if name not in self._children:
                self._children[name] = RegistryNode(parent=self, name=name)
            self._children[name]._variable = value
            root = self
            while root._parent is not None:
                root = root._parent
            if isinstance(root, VariableRegistry):
                root._update_symbol_map()
                root._validate_symbols()
                root._save_registry()
        else:
            super().__setattr__(name, value)

    def to_dict(self) -> Dict:
        if self._variable is not None:
            return self._variable.to_dict()
        result = {}
        for name, child in self._children.items():
            result[name] = child.to_dict()
        return result

    @classmethod
    def from_dict(cls, data: Dict, parent=None, name=None) -> 'RegistryNode':
        node = cls(parent=parent, name=name)
        
        if 'name' in data and 'symbol' in data:
            node._variable = Variable.from_dict(data)
        else:
            for key, value in data.items():
                child = cls.from_dict(value, parent=node, name=key)
                node._children[key] = child
        
        return node

class VariableRegistry(RegistryNode):
    def __init__(self, yaml_name: str):
        super().__init__()
        registry_dir = os.path.dirname(os.path.abspath(__file__))
        self.yaml_path = os.path.join(registry_dir, yaml_name)
        self._symbol_map = {}
        self._load_or_create_registry()
        self._update_symbol_map()
        self._validate_symbols()

    def _load_or_create_registry(self):
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f) or {}
                for key, value in data.items():
                    self._children[key] = RegistryNode.from_dict(
                        value, parent=self, name=key
                    )
        else:
            warnings.warn(f"Creating new registry at {self.yaml_path}")
            self._save_registry()

    def _save_registry(self):
        with open(self.yaml_path, 'w') as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False)

    def _update_symbol_map(self):
        self._symbol_map.clear()
        for path, var in self._collect_variables():
            if var.symbol in self._symbol_map:
                raise ValueError(f"Duplicate symbol {var.symbol} found at {path}")
            self._symbol_map[var.symbol] = (path, var)

    def _extract_symbols(self, equation: str) -> Set[str]:
        """Extract symbols from equation using sympy."""
        try:
            expr = sympy.parse_expr(equation)
            return {symbol.name for symbol in expr.free_symbols}
        except Exception as e:
            raise ValueError(f"Failed to parse equation '{equation}': {str(e)}")

    def _validate_symbols(self):
        for path, var in self._collect_variables():
            if var.definitions:
                for equation in var.definitions:
                    symbols = self._extract_symbols(equation)
                    for symbol in symbols:
                        if symbol not in self._symbol_map:
                            raise ValueError(
                                f"Unknown symbol '{symbol}' in equation '{equation}' "
                                f"at {path}"
                            )

    @property
    def symbols(self) -> Dict[str, Variable]:
        return {symbol: var for symbol, (_, var) in self._symbol_map.items()}
    
    def update_registry_definitions(self, new_definitions: Dict[str, Set[str]], save: bool = True):
        """
            Updates the registry with new definitions and optionally saves it.
        
        Args:
                new_definitions: Dict mapping variable symbols to new definitions to add
            save: Whether to save the registry after updating
        """
        # Update registry with new definitions
        for symbol, definitions in new_definitions.items():
            var_path, var_obj = self.registry._symbol_map[symbol]
            # Convert to list and remove duplicates while preserving order
            existing_defs = list(dict.fromkeys(var_obj.definitions))
            new_defs = list(dict.fromkeys(definitions))
            var_obj.definitions = existing_defs + [d for d in new_defs if d not in existing_defs]
        
        # Save registry if requested
        if save:
            self.registry._save_registry()
            
        # Update equations dict with new definitions
        self.equations = self._collect_equations()
