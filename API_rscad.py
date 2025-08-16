
"""
Author: Sabbani Assabban Iman
Date: 29/07/2025
Project: Contribution of Demand Response to Frequency Control:
PHIL Setup – Frequency Tracking and Measurement using RSCAD FX
Description:
  - This script connects to the RSCAD FX simulation environment.
  - It loads a predefined list of frequency values to simulate grid variation.
  - For each frequency, the simulation is updated, and voltage/current/power data is captured.
  - All measurements are saved in CSV format and tagged with the corresponding frequency.
  - Used to validate the PHIL system's ability to react to fast frequency deviations.
"""

import rtds.rscadfx
import time
import csv
import os

# Load frequency profile and time steps from CSV
freq = []
timestep = []


# Connect to RSCAD FX environment
with rtds.rscadfx.remote_connection() as app:
    case_id1 = app.open_case(r"midi de la recherche.rtfx")

    # Retrieve graphical signals and control object
    graph_voltage = case_id1.get_object(54)
    graph_current = case_id1.get_object(52)
    graph_current_CT = case_id1.get_object(119)
    graph_sim_voltage = case_id1.get_object(265)
    graph_sim_current = case_id1.get_object(267)
    graph_sim_power = case_id1.get_object(269)
    rt_slider_freq = case_id1.get_object(50)

    # Set initial frequency and start simulation
    rt_slider_freq.value = 50.0
    case_id1.run()

    # Create folder for temp files
    os.makedirs("exp1/mesures_rscad/temp", exist_ok=True)

    output_files = [
        "exp1/mesures_rscad/voltage.csv",
        "exp1/mesures_rscad/current.csv",
        "exp1/mesures_rscad/current_CT.csv",
        "exp1/mesures_rscad/sim_voltage.csv",
        "exp1/mesures_rscad/sim_current.csv",
        "exp1/mesures_rscad/sim_power.csv"
    ]

    temp_files = [
        "exp1/mesures_rscad/temp/voltage_tmp.csv",
        "exp1/mesures_rscad/temp/current_tmp.csv",
        "exp1/mesures_rscad/temp/current_CT_tmp.csv",
        "exp1/mesures_rscad/temp/sim_voltage_tmp.csv",
        "exp1/mesures_rscad/temp/sim_current_tmp.csv",
        "exp1/mesures_rscad/temp/sim_power_tmp.csv"
    ]

    graphs = [
        graph_voltage,
        graph_current,
        graph_current_CT,
        graph_sim_voltage,
        graph_sim_current,
        graph_sim_power
    ]

    # Open output files once and prepare writers
    files = [open(f, "w", newline='') for f in output_files]
    writers = [csv.writer(f) for f in files]
    header_written = [False] * len(writers)

    for i, frequence in enumerate(freq):
        # Apply frequency to the RT slider
        rt_slider_freq.value = float(frequence)
        time.sleep(0.01)

        # Save graphs to temporary files
        for graph, tmp_file in zip(graphs, temp_files):
            graph.save_data("CSV", tmp_file, separator=",")

        # Append data with frequency to final CSVs
        for j, (file_path, writer) in enumerate(zip(temp_files, writers)):
            with open(file_path, "r", newline='') as f:
                r = csv.reader(f)
                headers = next(r)

                if not header_written[j]:
                    new_headers = [headers[0]]  # Time
                    new_headers.insert(1, "Frequency")
                    new_headers += headers[1:]
                    writer.writerow(new_headers)
                    header_written[j] = True

                for row in r:
                    new_row = [row[0]]
                    new_row.insert(1, frequence)
                    new_row += row[1:]
                    writer.writerow(new_row)

        print(f"Data at t = {timestep[i]} s (f = {frequence} Hz) saved")

    for f in files:
        f.close()

    case_id1.stop()
    print("Finished")
