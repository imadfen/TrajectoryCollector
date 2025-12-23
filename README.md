# TrajectoryCollector

A vehicular network simulation project combining **OMNeT++** (network simulator) and **SUMO** (traffic simulator) to collect vehicle trajectory data and analyze V2V (Vehicle-to-Vehicle) communication patterns.

## 📋 Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [How It Works](#how-it-works)
- [File Descriptions](#file-descriptions)
- [Configuration](#configuration)
- [Building and Running](#building-and-running)
- [Output Data](#output-data)

## 🔍 Overview

This project simulates a realistic traffic scenario where vehicles:

1. Move according to real-world traffic patterns (SUMO)
2. Communicate with each other using IEEE 802.11p protocol (OMNeT++)
3. Collect trajectory and communication metrics for analysis
4. Output structured data for machine learning applications

**Key Features:**

- Real-time vehicle trajectory logging (position, speed, acceleration, heading)
- V2V communication with packet loss and delay metrics
- Neighbor detection and tracking
- Large-scale urban scenario simulation
- CSV output for data analysis and ML training

## 📁 Project Structure

```
TrajectoryCollector/
├── custom-scripts/               # Utility scripts
│   ├── processData.py            # Post-processing script
│   └── sumo-launchd.py           # SUMO launcher wrapper
│
├── docs/                         # Documentation
│   └── DATASET_DESCRIPTION.md    # Dataset documentation
│
├── results/                      # Simulation outputs
│   ├── raw/                      # CSV trajectory data
│   │   └── data_car_*.csv        # Per-vehicle trajectory files (97 files)
│   ├── vectors/                  # OMNeT++ vector files (empty)
│   ├── General-#0.sca            # Scalar results
│   ├── General-#0.vci            # Vector index
│   └── General-#0.vec            # Vector results
│
├── simulations/                  # Simulation launch scripts
│   ├── antenna.xml               # Antenna characteristics
│   ├── config.xml                # PHY layer configuration
│   ├── package.ned               # Network package definition
│   └── run                       # Execution script
│
├── src/                          # Source code
│   ├── trajectories/
│   │   ├── DataCollectorApp.cc    # Main application logic
│   │   ├── DataCollectorApp.h     # Application header
│   │   ├── DataCollectorApp.ned   # OMNeT++ network description
│   │   ├── TrajSafetyMessage.msg  # Custom message definition
│   │   ├── TrajSafetyMessage_m.cc # Generated message code
│   │   └── TrajSafetyMessage_m.h  # Generated message header
│   ├── Makefile                   # Build configuration
│   └── TrajectoryCollector        # Compiled binary
│
├── sumo/                         # All SUMO traffic simulation files
│   ├── basic.vType.xml           # Vehicle type definitions
│   ├── most.buses.flows.xml      # Bus traffic flows
│   ├── most.busstops.add.xml     # Bus stop infrastructure
│   ├── most.commercial.rou.xml   # Commercial vehicle routes
│   ├── most.highway.flows.xml    # Highway traffic flows
│   ├── most.parking.add.xml      # Parking infrastructure
│   ├── most.parking.allvisible.add.xml
│   ├── most.parking.norerouters.add.xml
│   ├── most.pedestrian.rou.xml   # Pedestrian routes
│   ├── most.poly.xml             # Polygons and visual elements
│   ├── most.special.rou.xml      # Special vehicle routes
│   ├── most.trains.flows.xml     # Train traffic flows
│   ├── most.trainstops.add.xml   # Train stop infrastructure
│   ├── myMap.net.xml             # Road network definition
│   ├── mySumoConfig.sumocfg      # Main SUMO configuration
│   └── traci.launch.xml          # SUMO-OMNeT++ coupling config
│
├── Makefile                      # Top-level build
├── omnetpp.ini                   # Main simulation configuration
└── README.md                     # This file
```

## ⚙️ How It Works

### 1. **Initialization Phase**

- OMNeT++ loads network configuration from `omnetpp.ini`
- SUMO traffic simulator starts via TraCI interface
- Each vehicle gets a `DataCollectorApp` module attached
- CSV files are created for each vehicle with unique filenames

### 2. **Simulation Loop** (runs every second)

**For each vehicle:**

```
a) SUMO updates vehicle position/speed
b) OMNeT++ receives position update
c) DataCollectorApp logs trajectory data
d) Vehicle broadcasts beacon message (TrajSafetyMessage)
e) Neighboring vehicles receive beacons
f) Communication metrics are calculated (delay, packet loss)
g) Data is written to CSV file
```

### 3. **Communication Flow**

```
Vehicle A                     Vehicle B
   |                              |
   |---(Beacon Seq#123)---------->|
   |                              | ✓ Received
   |                              | - Calculate delay
   |                              | - Update neighbor table
   |                              | - Check packet loss
   |                              | - Log to CSV
   |                              |
   |<---(Beacon Seq#456)----------|
   | ✓ Received                   |
   | - Process metrics            |
   | - Update neighbor data       |
   | - Log to CSV                 |
```

### 4. **Data Collection**

Each vehicle logs at every position update:

- **Own State:** Position (X,Y), Speed, Acceleration, Heading, Angular Velocity
- **Road Context:** Lane ID, Distance along lane
- **Neighbor Data:** Position, speed, heading of up to 3 nearest neighbors (relative coordinates)
- **Communication Metrics:** Average distance to sender, message delay, packet loss rate

### 5. **Finalization**

- Simulation ends after 72000s (20 hours) or manual stop
- CSV files are closed
- OMNeT++ generates statistics files (.sca, .vec)

## 📄 File Descriptions

### Source Code Files

| File                          | Purpose                                                                              |
| ----------------------------- | ------------------------------------------------------------------------------------ |
| **DataCollectorApp.cc/.h**    | Main application implementing trajectory collection and V2V communication logic      |
| **DataCollectorApp.ned**      | OMNeT++ module definition describing the app's interface and parameters              |
| **TrajSafetyMessage.msg**     | Custom message format extending DemoSafetyMessage with sequence numbers for tracking |
| **TrajSafetyMessage_m.cc/.h** | Auto-generated from .msg file (do not edit manually)                                 |

### SUMO Configuration Files

| File                        | Purpose                                                              |
| --------------------------- | -------------------------------------------------------------------- |
| **mySumoConfig.sumocfg**    | Main SUMO configuration orchestrating all traffic elements           |
| **myMap.net.xml**           | Complete road network with lanes, intersections, traffic lights      |
| **most.commercial.rou.xml** | Commercial vehicle routes (delivery trucks, vans)                    |
| **most.pedestrian.rou.xml** | Pedestrian movement patterns                                         |
| **most.special.rou.xml**    | Special vehicles (emergency, service)                                |
| **most.buses.flows.xml**    | Bus traffic flows and schedules                                      |
| **most.highway.flows.xml**  | Highway traffic patterns                                             |
| **most.trains.flows.xml**   | Train movements                                                      |
| **most.busstops.add.xml**   | Bus stop locations                                                   |
| **most.trainstops.add.xml** | Train station locations                                              |
| **most.parking.add.xml**    | Parking area definitions                                             |
| **most.poly.xml**           | Visual elements (buildings, parks, water)                            |
| **basic.vType.xml**         | Vehicle type characteristics (acceleration, deceleration, max speed) |

### OMNeT++ Configuration Files

| File                 | Purpose                                                               |
| -------------------- | --------------------------------------------------------------------- |
| **omnetpp.ini**      | Main simulation parameters (duration, network size, PHY/MAC settings) |
| **config.xml**       | Physical layer models (signal attenuation, interference)              |
| **antenna.xml**      | Antenna characteristics (gain, pattern)                               |
| **traci.launch.xml** | SUMO-OMNeT++ coupling configuration (TraCI settings)                  |

### Other Files

| File               | Purpose                                                             |
| ------------------ | ------------------------------------------------------------------- |
| **processData.py** | Python script to process CSV files and prepare training data for ML |
| **Makefile**       | Build instructions for compiling the project                        |
| **run**            | Shell script to execute the simulation                              |

## 🔧 Configuration

### Key Parameters (omnetpp.ini)

```ini
# Simulation Duration
sim-time-limit = 72000s           # 20 hours of simulated time

# Map Dimensions
*.playgroundSizeX = 25000m        # 25km x 25km area
*.playgroundSizeY = 25000m

# Communication Range
*.connectionManager.maxInterfDist = 300m   # V2V range

# Beacon Interval
# Defined in DataCollectorApp.cc: 1.0 second

# PHY Layer
*.node[*].nic.phy80211p.carrierFrequency = 5.890e9 Hz  # DSRC frequency
*.node[*].nic.mac1609_4.txPower = 20mW                 # Transmission power
*.node[*].nic.mac1609_4.bitrate = 6Mbps                # Data rate
```

### SUMO Time Window

```xml
<time>
    <begin value="17000"/>   <!-- Start at 17000s (4h 43min) -->
    <end value="72000"/>     <!-- End at 72000s (20 hours) -->
</time>
```

## 🔨 Building and Running

### Prerequisites

- OMNeT++ 5.6.2 or compatible
- SUMO 1.x
- Veins 2.5 framework (for V2V communication)
- GCC/Clang 13 compiler
- Python 3.x (for data processing)

### Build Steps

1. **Compile the project:**

   ```bash
   cd /home/massi/Documents/omnetpp-5.6.2/samples/TrajectoryCollector
   make clean
   make
   ```

2. **Run the simulation:**

   ```bash
   # From project root
   cd simulations
   ./run

   # Or directly with OMNeT++
   ../src/TrajectoryCollector -u Cmdenv -c General -n ../src:../../veins/src
   ```

3. **Process the results:**
   ```bash
   python3 processData.py
   ```

### Expected Output

- Multiple CSV files in `results/` directory: `data_car_1_t17106.csv`, `data_car_2_t17123.csv`, etc.
- OMNeT++ statistics files: `General-#0.sca`, `General-#0.vec`, `General-#0.vci`

## 📊 Output Data

### CSV File Format

Each vehicle generates a CSV file with the following columns:

| Column                   | Description                                 |
| ------------------------ | ------------------------------------------- |
| **Time**                 | Simulation time (seconds)                   |
| **X, Y**                 | Vehicle position (meters)                   |
| **Speed**                | Current speed (m/s)                         |
| **Acceleration**         | Current acceleration (m/s²)                 |
| **Heading**              | Direction angle (degrees)                   |
| **AngularVelocity**      | Rate of heading change (deg/s)              |
| **LaneID**               | Current lane identifier                     |
| **LaneDist**             | Distance traveled on current lane (m)       |
| **Neigh1_Rx, Neigh1_Ry** | Relative position of 1st neighbor (m)       |
| **Neigh1_RSpeed**        | Relative speed of 1st neighbor (m/s)        |
| **Neigh1_RHeading**      | Relative heading of 1st neighbor (deg)      |
| **Neigh2\_\***           | Same for 2nd neighbor                       |
| **Neigh3\_\***           | Same for 3rd neighbor                       |
| **AvgDistToSender**      | Average distance to all message senders (m) |
| **AvgMsgDelay**          | Average communication delay (s)             |
| **PacketLossRate**       | Percentage of lost packets (0-1)            |

### Example Row

```csv
17106.5,12450.3,8923.1,15.2,0.5,87.3,2.1,edge123_0,145.2,45.2,-12.3,18.4,5.2,-78.5,23.1,12.8,-3.4,120.4,56.7,22.1,-14.9,145.6,0.02,0.15
```
