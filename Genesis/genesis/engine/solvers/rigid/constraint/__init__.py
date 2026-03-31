"""
Constraint solver submodule for rigid body simulation.

Contains constraint solving, island detection, and backward pass.
"""

import genesis as gs

from .solver import ConstraintSolver
from .solver_island import ConstraintSolverIsland

# first declare func_solve_body:
from . import solver

# The decomposed registration path is only used on non-CPU backends. Older
# quadrants builds reject its Python-level perf_dispatch registration shape, so
# skip importing it on CPU where it is never selected anyway.
if gs.backend is not gs.cpu:
    from . import solver_breakdown
