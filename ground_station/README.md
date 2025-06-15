# Ground Station 25 <img src="https://github.com/user-attachments/assets/05a3d1d7-f5c2-4c9b-8a05-54f5ed727f80" alt="logo" width="50"/>

This tool offers an intuitive interface that allows users to monitor telemetry data and set route waypoints while the boat is in operation.

## Getting Started

### Prerequisites

Ensure that you have the following software installed on your system:

- [Go](https://go.dev/doc/install) (1.23.4)
- [Python](https://www.python.org/downloads/) (tested using version 3.13, but other versions may also work)

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/sailbot-vt/ground_station_25.git
   cd ground_station_25
   ```

2. Install the Python dependencies:

   ```bash
   pip install -r requirements.txt
   ```

3. Give the start script executable permissions:

   ```bash
   chmod +x run.sh
   ```

4. Run the program

   ```bash
   ./run.sh
   ```

### Usage

- Left click on the map to add waypoints.
- Right click to remove the waypoint closest to your mouse's cursor position.

### Demo (might be out of date with current iteration)

<https://github.com/user-attachments/assets/05fde0a0-8deb-4650-98ec-527798fddb3d>
