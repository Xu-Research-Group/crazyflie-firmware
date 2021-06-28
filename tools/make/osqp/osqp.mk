OSQP_MK_DIR := $(CRAZYFLIE_BASE)/tools/make/osqp
OSQP_DIR := $(LIB)/osqp
OSQP_SRC := $(OSQP_DIR)/src/osqp
OSQP_OBJ := auxil.o error.o lin_alg.o osqp.o proj.o qdldl.o qdldl_interface.o scaling.o util.o workspace.o
OSQP_INCLUDE := $(OSQP_DIR)/include
OSQP_CODE_GEN := $(OSQP_MK_DIR)/osqp_code_gen.py
