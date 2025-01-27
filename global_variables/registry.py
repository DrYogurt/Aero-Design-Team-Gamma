import os
import yaml
import re
from typing import Dict, Optional, List, Union, Set
import warnings

class Variable:
    def __init__(
        self,
        name: str,
        symbol: str,
        description: Optional[str] = None,
        minimum: Optional[float] = None,
        maximum: Optional[float] = None,
        definitions: Optional[List[str]] = None
    ):
        self.name = name
        self.symbol = symbol
        self.description = description
        self.minimum = minimum
        self.maximum = maximum
        self.definitions = definitions or []

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
        return data

    @classmethod
    def from_dict(cls, data: Dict) -> 'Variable':
        return cls(
            name=data['name'],
            symbol=data['symbol'],
            description=data.get('description'),
            minimum=data.get('minimum'),
            maximum=data.get('maximum'),
            definitions=data.get('definitions')
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
        """Collect all variables in this node and its children."""
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
        # Get directory of the registry.py file
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
                    self._children[key] = RegistryNode.from_dict(value, parent=self, name=key)
        else:
            warnings.warn(f"Creating new registry at {self.yaml_path}")
            self._save_registry()

    def _save_registry(self):
        with open(self.yaml_path, 'w') as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False)

    def _update_symbol_map(self):
        """Update the symbol to variable mapping."""
        self._symbol_map.clear()
        for path, var in self._collect_variables():
            if var.symbol in self._symbol_map:
                raise ValueError(f"Duplicate symbol {var.symbol} found at {path}")
            self._symbol_map[var.symbol] = (path, var)

    def _extract_symbols(self, equation: str) -> Set[str]:
        """Extract symbols from equation by splitting on operators and filtering."""
        # Split on common operators and whitespace
        tokens = re.split(r'[\s\+\-\*/\(\)]', equation)
        # Keep only non-empty tokens that don't start with numbers
        return {token for token in tokens if token and not token[0].isdigit()}

    def _validate_symbols(self):
        """Validate all symbols in equations exist and are unique."""
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
        """Return a dictionary of all symbols to their variables."""
        return {symbol: var for symbol, (_, var) in self._symbol_map.items()}

# Example usage:
if __name__ == "__main__":
    registry = VariableRegistry("engineering_variables.yaml")
    
    registry.aircraft.performance.velocity.min = Variable(
        name="minimum",
        symbol="V_min",
        minimum=0,
        maximum=50
    )
    
    registry.aircraft.performance.velocity.stall = Variable(
        name="stall",
        symbol="V_s",
        description="Aircraft stall velocity",
        minimum=0,
        maximum=100,
        definitions=["1.2 * V_min"]  # This will validate that V_min exists
    )


    registry.aero.coefficients.lift.zero = Variable(
        name="zero lift coefficient",
        symbol="C_L_0",
        description="Lift coefficient at zero angle of attack"
    )

    registry.aero.coefficients.lift.max = Variable(
        name="max lift",
        symbol="C_L_max",
        definitions=["2.5 * C_L_0"]  # This will now validate correctly
    )
    # Access the symbol map
    print(registry.symbols)  # Shows all symbols and their variables
    
    # This would raise an error due to undefined symbol:
    # registry.aircraft.performance.velocity.cruise = Variable(
    #     name="cruise",
    #     symbol="V_c",
    #     definitions=["2.0 * V_unknown"]
    # )
