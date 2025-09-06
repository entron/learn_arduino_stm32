import serial
import matplotlib.pyplot as plt
from collections import deque

# === Serial Config ===
PORT = "/dev/ttyUSB0"
BAUD = 115200
ser = serial.Serial(PORT, BAUD)

# === Buffers ===
max_points = 500
tgt_data   = deque(maxlen=max_points)
vel_data   = deque(maxlen=max_points)
mech_data  = deque(maxlen=max_points)
elec_data  = deque(maxlen=max_points)
samples    = deque(maxlen=max_points)

# === Matplotlib Setup ===
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

# Top subplot: target vs measured velocity
line_tgt, = ax1.plot([], [], label="Target vel [rad/s]", color="tab:red")
line_vel, = ax1.plot([], [], label="Measured vel [rad/s]", color="tab:blue")
ax1.set_ylabel("Velocity [rad/s]")
ax1.legend(loc="upper right")
ax1.grid(True)

# Bottom subplot: mechanical angle + electrical angle with twin y-axis
line_mech, = ax2.plot([], [], label="Mech angle [rad]", color="tab:green")
ax2.set_ylabel("Mech angle [rad]")
ax2.grid(True)

ax2_twin = ax2.twinx()
line_elec, = ax2_twin.plot([], [], label="Elec angle [rad]", color="tab:purple")
ax2_twin.set_ylabel("Elec angle [rad]")

# Combine legends for bottom subplot
lines = [line_mech, line_elec]
labels = [l.get_label() for l in lines]
ax2.legend(lines, labels, loc="upper right")

ax2.set_xlabel("Samples")

plt.tight_layout()

# === Main Loop ===
while True:
    try:
        # Expecting serial format like:
        # tgt=12.3 vel=11.8 angle=1.57 elA(rad)=5.49
        raw = ser.readline().decode(errors="ignore").strip()

        # Parse key=value pairs
        try:
            parts = {k: float(v) for k, v in (p.split("=") for p in raw.split() if "=" in p)}
            tgt   = parts.get("tgt", 0.0)
            vel   = parts.get("vel", 0.0)
            angle = parts.get("angle", 0.0)
            elA   = parts.get("elA(rad)", parts.get("elA", 0.0))
        except:
            continue  # skip malformed lines

        # Update buffers
        samples.append(len(samples))
        tgt_data.append(tgt)
        vel_data.append(vel)
        mech_data.append(angle)
        elec_data.append(elA)

        # Update top subplot (tgt vs vel)
        line_tgt.set_data(samples, tgt_data)
        line_vel.set_data(samples, vel_data)
        ax1.relim()
        ax1.autoscale_view()

        # Update bottom subplot (angle vs elA)
        line_mech.set_data(samples, mech_data)
        ax2.relim()
        ax2.autoscale_view()

        line_elec.set_data(samples, elec_data)
        ax2_twin.relim()
        ax2_twin.autoscale_view()

        # Refresh plots
        plt.pause(0.01)

    except KeyboardInterrupt:
        print("\nStopped by user.")
        break

ser.close()
