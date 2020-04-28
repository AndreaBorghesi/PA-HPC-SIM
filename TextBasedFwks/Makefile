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

all: online_dispatcher 
	
.PHONY: all clean online_dispatcher 

# ------------------------------------------------------------------------------
# Rules to build the core constraint files
# ------------------------------------------------------------------------------

RW_HEADERS := $(SRC_FOLDER)/Job.hpp $(SRC_FOLDER)/JobQueue.hpp \
				$(SRC_FOLDER)/JobQueueReader.hpp $(SRC_FOLDER)/JobReader.hpp \
				$(SRC_FOLDER)/JobWriter.hpp $(SRC_FOLDER)/Node.hpp \
				$(SRC_FOLDER)/NodeReader.hpp $(SRC_FOLDER)/Util.hpp

OPT_HEADERS := $(SLV_FOLDER)/Optimizer.hpp $(SLV_FOLDER)/OptimizerImpl.hpp

$(OUT_FOLDER)/online_dispatcher.o: $(OUT_FOLDER) \
							$(OPT_HEADERS) $(RW_HEADERS) \
							$(SLV_FOLDER)/FixedStartSchedule.hpp \
							$(SLV_FOLDER)/Rankers.hpp \
							$(SLV_FOLDER)/Predictor.hpp \
							$(SLV_FOLDER)/ListScheduler.hpp \
							$(SLV_FOLDER)/BackFillingScheduler.hpp \
							$(SLV_FOLDER)/StaticDVFSScheduler.hpp \
							$(SLV_FOLDER)/InstanceAnalyser.hpp \
							$(SLV_FOLDER)/OnlineDispatcher.hpp \
							$(SRC_FOLDER)/online_dispatcher.cpp

	$(CCC) $(CFLAGS) -o $(OUT_FOLDER)/online_dispatcher.o -c $(SRC_FOLDER)/online_dispatcher.cpp

online_dispatcher: $(OUT_FOLDER)/online_dispatcher.o 
	$(CCC) -o online_dispatcher $(OUT_FOLDER)/online_dispatcher.o $(LDFLAGS) 

# ------------------------------------------------------------------------------
# A rule to clean everything
# ------------------------------------------------------------------------------

clean:
	rm -f $(OUT_FOLDER)/*
	rm -f online_dispatcher

