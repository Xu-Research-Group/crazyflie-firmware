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
P = sparse.csc_matrix(2*np.eye(4))
q = np.array([1, 1, 1, 1])
A = sparse.csc_matrix([[0, 1, 1, 1], [0, 0, 1, 1]])
l = np.array([-np.inf, -np.inf])
u = np.array([np.inf, np.inf])


# Create an OSQP object
m = osqp.OSQP()

# Solver initialization
m.setup(P, q, A, l, u, max_iter=20)

# Generate code
m.codegen(src, force_rewrite=True, project_type='Makefile',parameters='matrices',FLOAT=True,LONG=False)
