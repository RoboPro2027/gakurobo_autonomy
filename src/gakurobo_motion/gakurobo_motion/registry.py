"""Load the primitives of one robot from a YAML list."""

from importlib import import_module
from pathlib import Path

import yaml

from gakurobo_motion.primitive import Primitive


def load_class(reference: str) -> type:
    """Return the Primitive class named by ``module:Class``."""
    module_name, _, class_name = reference.partition(':')
    primitive = getattr(import_module(module_name), class_name)
    if not (isinstance(primitive, type) and issubclass(primitive, Primitive)):
        raise TypeError(f'{reference} is not a Primitive')
    return primitive


def load_primitives(paths, node, robot: str) -> dict:
    """Create the primitives of all files. A later file overrides an earlier one."""
    references = {}
    for path in paths:
        document = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
        references.update(document.get('primitives') or {})
    return {
        name: load_class(reference)(node, robot)
        for name, reference in references.items()
    }
