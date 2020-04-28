# PA-HPC-SIM: a Power-Aware HPC Simulator

Brief: a simulation framework for job dispatching in HPC systems with a limited
budget for power consumption.

Author: Andrea Borghesi, University of Bologna

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License.

You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.  See the License for the
specific language governing permissions and limitations under the License.

Prerequisites
=============

1) python (>= 2.7), any C++ compiler (gcc, clang++, etc)

Build steps
===========

1) cd to TextBasedFwks/ folder

2) Modify Makefile.conf 
    a) change python installation directory (if needed)

3) Compile
    a) make online_dispatcher

Test
====
1) Check if it works
    a) cd to TextBasedFwks/ folder
    b) launch the following command:

./online_dispatcher ../test_data/system_configs/nodes_#64 ../test_data/system_configs/system_queues ../test_data/job_instances/synthetic_workloads/jobs_100/ws_900/rndm_100_900_0.log_crypt ../test_data/eurora_power_predictions/power_predictions 0 ../test_data/variable_power_caps/power_caps_noVariations 101 4 7881 7881 400 -1 0

    c) look at the generated output
        I) there should be no error and there should be a final schedule ("FINAL SCHEDULE"), containing start times and execution nodes for all jobs

Detailed Info
============

1) see README contained in subdirectories for more detailed information

Test-cases of the PA-HPC-SIM
============================

PA-HPC-SIM has been used in several research works aiming at studying issues
concerning HPC system with limited power budget. Selected examples:
- "Predictive modeling for job power consumption in HPC systems", A. Borghesi,
  A. Bartolini, M. Lombardi, M. Milano, L. Benini, International Conference on
High Performance Computing, 2016
- "Power capping in high performance computing systems", Andrea Borghesi,
  Francesca Collina, Michele Lombardi, Michela Milano, Luca Benini, CP2015
- "Scheduling-based power capping in high performance computing systems", Andrea
  Borghesi, Andrea Bartolini, Michele Lombardi, Michela Milano, Luca Benini,
Sustainable Computing: Informatics and Systems, 2018
- "Frequency Assignment in High Performance Computing Systems", Andrea Borghesi,
  Michela Milano, Luca Benini, International Conference of the Italian
Association for Artificial Intelligence, 2019




