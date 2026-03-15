# Transformer Dataset Generation Plan

This document outlines the target specifications and simulation strategies required to generate a high-quality, transformer-ready dataset using OMNeT++ and SUMO.

## 1. Dataset Targets

- **Total Distinct Vehicles:** 1,000 to 3,000+ cars across the entire dataset.
  - _Reasoning:_ Transformers need large, diverse datasets to properly learn temporal dynamics, generalize paths, and avoid overfitting to specific intersections or behaviors.
- **Rows per Vehicle:** 60 to 300 rows (representing 1 to 5 minutes of active driving at 1 Hz frequency).
  - _Minimum threshold:_ Discard sequences with fewer than 30 rows during dataset preprocessing. Short sequences lack sufficient temporal context for attention mechanisms.

## 2. Simulation Time

- **Simulation Duration:** Run for **3,600 to 7,200 simulation seconds** (1 to 2 hours of simulated time).

## 3. Spawning & Configuration Strategy

- **Spawn Cutoff:** Stop spawning new vehicles approximately 600 seconds before the simulation ends. This ensures the last batch of spawned cars has sufficient time to generate long, usable trajectory sequences (>= 60 rows) before the simulator shuts down.
- **Concurrent Limits:** To prevent the simulation from overloading and to control traffic density, establish a concurrent vehicle cap in the `.sumocfg` (e.g., `<max-num-vehicles value="300"/>`).
- **Global Scaling:** To broadly decrease traffic without manually editing massive `.rou.xml` files, use the `<scale value="0.x"/>` parameter within the `<processing>` block of the `.sumocfg`.

## 4. Post-Processing Requirements

- **Stationary Filtering:** Downsample or flag long continuous sequences of stopped traffic (`Speed = 0`) where `X, Y` coordinates do not change. Unprocessed, these stretches will heavily bias the transformer model towards predicting stationary states.
- **Sanity Checks:** Clean up data anomalies, such as normalizing `AngularVelocity` wraparounds (e.g., near 360°) and double-checking the meaning/validity of the `AvgMsgDelay` feature.

## 5. Data Volume Estimation

A simple rule:

$$
\text{rows} \approx \text{avg active cars} \times \text{sim seconds}
$$

Target at least 500k–1M useful rows after preprocessing. So wait until you reach that scale (not just raw rows with many repeated zero-speed points).

## 6. Project Overview

- **SUMO runs from `17000s` to `72000s`** (omnetpp.ini + SUMO config in README/docs).
- Each vehicle uses `DataCollectorApp`.
- On app init, it creates one CSV file per car:
  `results/raw/data_car_<index>_t<simTime>.csv`.
- Logging happens in `handlePositionUpdate()` at mobility update rate (**1 Hz**), so roughly **1 row/sec while the car exists**.
- Every row logs:
  - ego kinematics (`X,Y,Speed,Acceleration,Heading,...`)
  - lane context (`LaneID`, `LaneDist`)
  - up to 3 nearest neighbors (relative features)
  - communication metrics (`AvgDistToSender`, `AvgMsgDelay`, `PacketLossRate`)
- Beacons are broadcast every 1s (after random initial offset), neighbors are kept for 5s.
- Long `Speed=0` stretches are expected because logging is time-based, even when stopped.

## 7. What Is Happening Overall

- This setup generates a **global-time, per-vehicle, multivariate sequence dataset** for trajectory + V2V behavior.
- Different cars have different row counts because they spawn/despawn at different times.
- `AvgMsgDelay` values near simulation time suggest timestamp handling is likely not meaningful for delay as currently logged.

## 8. Recommended Data Volume for Transformer Training

For efficient results (after cleaning redundant stationary repeats):

- **Minimum usable:** ~`100k` timesteps
- **Recommended:** **`500k – 1M` timesteps**
- **Strong/generalizable:** `2M+` timesteps across varied traffic densities/routes

Practical target for this setup: generate about **`1–2M raw rows`**, then preprocess to keep about **`500k–1M high-quality rows`**.  
That is usually a solid range for transformer-based trajectory forecasting.

## 9. Summary of Current Run Setup

- **Scenario Map:** Monaco SUMO Traffic (MoST) Scenario
- **Logging Frequency:** 1 Hz (1 row per simulated second per vehicle)
- **Execution Window:** Simulation ran from `17000s` to `22470s` (a duration of 5,470 simulation seconds).
