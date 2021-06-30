#! /usr/bin/env python3

__author__ = "Victor Freire"
__email__ = "freiremelgiz@wisc.edu"

"""
This program generates C source code for the OSQP library
to be embedded in the Crazyflie firmware to solve QP online
The output of this program is a folder containing the C
source code called "osqp" and it is placed in
CRAZYFLIE_BASE/src/lib/osqp
"""
import numpy as np
from scipy import sparse
import sys
try:
    import osqp
except ImportError as e:
    print()
    print("ERROR (python3):",e)
    print("Cannot generate OSQP source code. Either:")
    print("  1. Unset COMPILE_OSQP flag in 'config.mk' or")
    print("  2. Install with: 'pip3 install osqp'")
    print()
    sys.exit(1)


# Target folder for source code is passed as parameter
src = sys.argv[1]

# Define problem data
n = 3 # Variables
m = 2 # Constraints
P = sparse.csc_matrix(2*np.eye(n))
q = np.ones(n)
A = sparse.csc_matrix([[-1, 0, 0], [1, 0, 0]])#, [0, -1, 0], [0, 1, 0]])
l = -np.inf*np.ones(m)
u = np.inf*np.ones(m)


# Create an OSQP object
m = osqp.OSQP()

# Solver initialization
m.setup(P, q, A, l, u, check_termination=1)

# Generate code
m.codegen(src, force_rewrite=True, parameters='vectors',FLOAT=True,LONG=False)
