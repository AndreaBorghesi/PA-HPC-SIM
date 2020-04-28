# Copyright 2014 <andrea.borghesi3@unibo.it>
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ------------------------------------------------------------------------------
# Include the configuration info
# ------------------------------------------------------------------------------

include Makefile.conf

# ------------------------------------------------------------------------------
# Main commands
# ------------------------------------------------------------------------------

all: core_test FixedStartSchedule_test ListScheduler_test
	
.PHONY: all clean core_test FixedStartSchedule_test ListScheduler_test

# ------------------------------------------------------------------------------
# Rules to build the core constraint files
# ------------------------------------------------------------------------------

$(OUT_FOLDER)/core_test.o: $(OUT_FOLDER) \
							$(SRC_FOLDER)/Optimizer.hpp \
							$(SRC_FOLDER)/OptimizerImpl.hpp \
							$(SRC_FOLDER)/core_test.cpp
	$(CCC) $(CFLAGS) -o $(OUT_FOLDER)/core_test.o -c $(SRC_FOLDER)/core_test.cpp

core_test: $(OUT_FOLDER)/core_test.o
	$(CCC) -o core_test $(OUT_FOLDER)/core_test.o $(LDFLAGS)


$(OUT_FOLDER)/FixedStartSchedule_test.o: $(OUT_FOLDER) \
							$(SRC_FOLDER)/Optimizer.hpp \
							$(SRC_FOLDER)/OptimizerImpl.hpp \
							$(SRC_FOLDER)/FixedStartSchedule.hpp \
							$(SRC_FOLDER)/FixedStartSchedule_test.cpp
	$(CCC) $(CFLAGS) -o $(OUT_FOLDER)/FixedStartSchedule_test.o -c $(SRC_FOLDER)/FixedStartSchedule_test.cpp

FixedStartSchedule_test: $(OUT_FOLDER)/FixedStartSchedule_test.o
	$(CCC) -o FixedStartSchedule_test $(OUT_FOLDER)/FixedStartSchedule_test.o $(LDFLAGS)
	

$(OUT_FOLDER)/ListScheduler_test.o: $(OUT_FOLDER) \
							$(SRC_FOLDER)/Optimizer.hpp \
							$(SRC_FOLDER)/OptimizerImpl.hpp \
							$(SRC_FOLDER)/FixedStartSchedule.hpp \
							$(SRC_FOLDER)/Rankers.hpp \
							$(SRC_FOLDER)/ListScheduler.hpp \
							$(SRC_FOLDER)/ListScheduler_test.cpp
	$(CCC) $(CFLAGS) -o $(OUT_FOLDER)/ListScheduler_test.o -c $(SRC_FOLDER)/ListScheduler_test.cpp

ListScheduler_test: $(OUT_FOLDER)/ListScheduler_test.o
	$(CCC) -o ListScheduler_test $(OUT_FOLDER)/ListScheduler_test.o $(LDFLAGS)

	
# ------------------------------------------------------------------------------
# A rule to clean everything
# ------------------------------------------------------------------------------

clean:
	rm -f $(OUT_FOLDER)/*
	rm -f FixedStartSchedule_test
	rm -f ListScheduler_test
	rm -f core_test

