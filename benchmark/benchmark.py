"""
this file will test the implementation at full speed on the test_scenarii already defined and will make sort of a report
the main aim is to have an easy way to assert how improvements enhance time of simulations
:author: Maxence Barré
:date: 2025
"""

import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from core_calculation.body_definition import Body
from core_calculation.core_simulation import Simulation
from core_calculation.force_definition import *
import json
import time
from annexe_benchmark import test_computer_speed

print("Starting benchmark...")
print("[INFO] Benchmarking computer speed...")
benchmark_of_computer = test_computer_speed()
print("[INFO] Computer speed benchmarked.")

report_to_save = {}
report_to_save["comment"] = "Write here your comments"
report_to_save["date"] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
report_to_save["computer_info"] = benchmark_of_computer

nb_of_dt = 1e4  # number of dt to simulate for the benchmark


# gathering the liste of test scenarii
list_test_scenarii = [f for f in os.listdir("scenarii_examples/") if f.endswith(".json")]


for file in list_test_scenarii:
    print(f"Running benchmark for {file}")
    # loading the scenario
    # the idea is : load the scenario, change the time_simulation to N*dt, then run and time the simulation
    # finally "write" the report

    # loading the scenario
    with open("scenarii_examples/" + file, 'r') as f:
        scenario_data = json.load(f)
        dt = scenario_data["parameters"]["dt"]
        scenario_data["parameters"]["duration"] = nb_of_dt * dt
    with open("benchmark/temp.json", 'w') as temp_f:
        json.dump(scenario_data, temp_f, indent=4)

    sim = Simulation(json_file="benchmark/temp.json")

    os.remove("benchmark/temp.json")
    
    # running the simulation
    start_time = time.time()
    for _ in sim.run(reduce_speed=False):
        pass
    end_time = time.time()
    report_to_save[file] = {
        "time_taken": end_time - start_time,
        "nb_of_dt": nb_of_dt,
        "time_per_dt": (end_time - start_time) / nb_of_dt,
        "time_taken_normalized": (end_time - start_time) / benchmark_of_computer['score']
    }
    
    print(f"Simulation for {file} completed in {end_time - start_time:.2f} seconds for {nb_of_dt} steps.")

# saving the report
current_time = time.strftime("%Y%m%d_%H%M%S", time.localtime())
with open(f"benchmark/report_benchmark_{current_time}.json", 'w') as report_f:
    json.dump(report_to_save, report_f, indent=4)
print("Benchmark completed. Report saved to benchmark/report_benchmark.json")