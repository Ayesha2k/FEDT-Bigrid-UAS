import time
import requests
import logging
import csv
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Microservice endpoints for the swarm
DRONES = {
    "cf0": "http://localhost:5000",
    "cf1": "http://localhost:5001",
    "cf2": "http://localhost:5002"
}

def load_trajectory_pieces(csv_path):
    """Natively parses a Crazyswarm2 polynomial trajectory CSV file into pieces dictionaries."""
    pieces = []
    path = Path(csv_path)
    if not path.exists():
        raise FileNotFoundError(f"Trajectory file not found at: {path}")
        
    with open(path, mode='r') as f:
        reader = csv.reader(f)
        header = next(reader, None)  # Skip header row if present
        
        for row in reader:
            if not row or len(row) < 33:
                continue
            
            duration = float(row[0])
            poly_x = [float(x) for x in row[1:9]]
            poly_y = [float(x) for x in row[9:17]]
            poly_z = [float(x) for x in row[17:25]]
            poly_yaw = [float(x) for x in row[25:33]]
            
            pieces.append({
                "duration": duration,
                "poly_x": poly_x,
                "poly_y": poly_y,
                "poly_z": poly_z,
                "poly_yaw": poly_yaw
            })
    logging.info(f"Loaded {len(pieces)} trajectory pieces from {path.name}")
    return pieces

def send_request_to_all(endpoint_suffix, payload=None):
    """Broadcasts an HTTP POST request to all drones concurrently/sequentially."""
    results = {}
    for drone_id, base_url in DRONES.items():
        url = f"{base_url}{endpoint_suffix}"
        try:
            if payload:
                response = requests.post(url, json=payload, timeout=5)
            else:
                response = requests.post(url, timeout=5)
            
            if response.status_code == 200:
                logging.info(f"[{drone_id}] {endpoint_suffix} -> Success: {response.json().get('message', 'OK')}")
                results[drone_id] = True
            else:
                logging.error(f"[{drone_id}] {endpoint_suffix} -> Failed ({response.status_code}): {response.text}")
                results[drone_id] = False
        except Exception as e:
            logging.error(f"[{drone_id}] {endpoint_suffix} -> Connection Error: {e}")
            results[drone_id] = False
    return results

def main():
    logging.info("Starting Multi-Trajectory Swarm Mission Orchestration...")

    # 1. Locate and parse local trajectory CSV files
    base_data_dir = Path("/workspace/cps.asset.crazyflie/controller/cf.PyControl/data/multi_trajectory")
    traj0_path = base_data_dir / "traj0.csv"
    traj1_path = base_data_dir / "traj1.csv"

    pieces_0 = load_trajectory_pieces(traj0_path)
    pieces_1 = load_trajectory_pieces(traj1_path)

    # 2. Activate Idle State
    logging.info("Step 1: Activating idle state across swarm...")
    send_request_to_all("/activate_idle")
    time.sleep(1.0)

    # 3. Synchronized Takeoff
    logging.info("Step 2: Commanding synchronized takeoff to 1.0m...")
    send_request_to_all("/begin_takeoff", {"height": 1.0, "duration": 2.0})
    time.sleep(4.0)  # Wait to stabilize at hover

    # 4. Upload Trajectory Pattern 1 (ID 0)
    logging.info("Step 3: Uploading Trajectory Pattern 1 (ID 0)...")
    upload_payload_0 = {
        "trajectory_id": 0,
        "pieces": pieces_0
    }
    send_request_to_all("/trajectory/upload", upload_payload_0)

    # 5. Execute Trajectory Pattern 1
    logging.info("Step 4: Executing Trajectory Pattern 1...")
    send_request_to_all("/trajectory/multi_start", {"trajectory_id": 0, "timescale": 1.0})
    time.sleep(3.0)  # Let it fly for a few seconds

    # 6. Mid-Flight Emergency Stop
    logging.info("Step 5: !!! TRIGGERING MID-FLIGHT EMERGENCY STOP (/emergency_stop) !!!")
    send_request_to_all("/emergency_stop")
    time.sleep(3.0)  # Hold position after stop

    # 7. Upload New Trajectory Pattern 2 (ID 1)
    logging.info("Step 6: Uploading New Trajectory Pattern 2 (ID 1)...")
    upload_payload_1 = {
        "trajectory_id": 1,
        "pieces": pieces_1
    }
    send_request_to_all("/trajectory/upload", upload_payload_1)

    # 8. Switch to and Execute Trajectory Pattern 2
    logging.info("Step 7: Executing New Trajectory Pattern 2...")
    send_request_to_all("/trajectory/multi_start", {"trajectory_id": 1, "timescale": 1.0})
    time.sleep(4.0)

    # 9. Safe Landing
    logging.info("Step 8: Initiating Safe Landing...")
    send_request_to_all("/begin_landing", {"duration": 2.0})
    time.sleep(3.0)

    logging.info("Swarm mission completed successfully!")

if __name__ == "__main__":
    main()