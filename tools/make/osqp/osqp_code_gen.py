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

import osqp
import numpy as np
from scipy import sparse

# Target folder for source code
src = './../../../src/lib/osqp'

# Define problem data
P = sparse.csc_matrix(2*np.eye(4))
q = np.array([1, 1, 1, 1])
A = sparse.csc_matrix([[0, 1, 1, 1], [0, 0, 1, 1]])
l = np.array([-9999, -9999])
u = np.array([9999, 9999])


# Create an OSQP object
m = osqp.OSQP()
# Solver initialization
m.setup(P, q, A, l, u)
# Generate code
m.codegen(src, force_rewrite=True, project_type='Makefile',parameters='matrices',FLOAT=True,LONG=False)
