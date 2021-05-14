# OSQP Targets

# Rule to make the osqp source %.c files if they are missing
$(patsubst %.o,$(OSQP_SRC)/%.c,$(OSQP_OBJ)): $(OSQP_DIR)

# Rule to make the osqp automatic codegen directory from Python
$(OSQP_DIR):$(OSQP_CODE_GEN)
	@$(OSQP_CODE_GEN) $(OSQP_DIR)

auxil.o: $(OSQP_SRC)/auxil.c
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)
	@rm -rf $(CRAZYFLIE_BASE)/*.so

error.o: $(OSQP_SRC)/error.c
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)

kkt.o: $(OSQP_SRC)/kkt.c
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)

lin_alg.o: $(OSQP_SRC)/lin_alg.c
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)

osqp.o: $(OSQP_SRC)/osqp.c
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)

proj.o: $(OSQP_SRC)/proj.c
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)

qdldl.o: $(OSQP_SRC)/qdldl.c
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)

qdldl_interface.o: $(OSQP_SRC)/qdldl_interface.c
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)

scaling.o: $(OSQP_SRC)/scaling.c
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)

util.o: $(OSQP_SRC)/util.c
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)

workspace.o: $(OSQP_SRC)/workspace.c
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)

osqp_clean:
	-rm -rf $(OSQP_DIR)
.PHONY: osqp_clean
