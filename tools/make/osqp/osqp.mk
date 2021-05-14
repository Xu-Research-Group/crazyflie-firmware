OSQP_DIR := $(LIB)/osqp
OSQP_SRC := $(OSQP_DIR)/src/osqp
OSQP_OBJ := auxil.o error.o kkt.o lin_alg.o osqp.o proj.o qdldl.o qdldl_interface.o scaling.o util.o workspace.o
OSQP_INCLUDE := -I$(OSQP_DIR)/include
