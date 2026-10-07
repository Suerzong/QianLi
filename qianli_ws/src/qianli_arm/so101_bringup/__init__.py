"""Keep the public module name while using one canonical overlay source tree."""
from so101_overlay import __path__ as _overlay_paths

__path__.extend(_overlay_paths)
