# OSQP Targets
auxil.o: $(OSQP_SRC)/auxil.c
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)

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
