import csv
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

DATA_DIR = Path(__file__).parent

STEADY_STATE_FRACTION = 0.20
SETTLING_TOLERANCE = 0.05
COMMAND_DELAY = 2.0

def load_data(filename, velocity_column):
    times = []
    velocities = []

    with open(DATA_DIR / filename, "r") as f:
        reader = csv.DictReader(f, skipinitialspace=True)

        for row in reader:
            times.append(float(row["time_sec"]))
            velocities.append(float(row[velocity_column]))

    times = np.array(times)
    velocities = np.array(velocities)

    return times, velocities


def plot_velocity(filename, velocity_column, title, ylabel):
    times, velocities = load_data(filename, velocity_column)

    tail_start = int(len(velocities) * (1 - STEADY_STATE_FRACTION))
    steady_value = np.mean(velocities[tail_start:])                                             # mean of end 20% of data
    steady_std = np.std(velocities[tail_start:])

    threshold = 0.90 * steady_value
    direction = np.sign(steady_value)

    crossed = (times >= COMMAND_DELAY) & (direction * velocities >= direction * threshold)      # condition to find first data point that crosses 90% of its final value

    rise_time = None
    crossing_time = None

    if np.any(crossed):                                                                         # checks if any measurement crossed 90% of its final value
        index = np.flatnonzero(crossed)[0]                                                      # finds the first index that is True
        crossing_time = times[index]                                                            # gets the timestamp of the measurement that is the first index of True
        rise_time = crossing_time - COMMAND_DELAY                                               # measures how long after the command the first index of True was reached

    tolerance = SETTLING_TOLERANCE * abs(steady_value)
    within_band = np.abs(velocities - steady_value) <= tolerance

    settling_time = None                                                                        # will hold how long it takes the velocity to reach and stay within 5% of its final value
    settling_time_absolute = None

    for i in range(len(times)):                                                                 # find the settling time
        if times[i] < COMMAND_DELAY:
            continue

        if np.all(within_band[i:]):                                                             # find if current measurement and all measurements after it are within the 5% band
            settling_time_absolute = times[i]
            settling_time = times[i] - COMMAND_DELAY
            break

    print(f"\n{title}")
    print(f"Estimated settled velocity: {steady_value:.6f}")
    print(f"Final-region standard deviation: {steady_std:.6f}")

    if rise_time is not None:
        print(f"90% threshold: {threshold:.6f}")
        print(f"90% rise time: {rise_time:.6f} seconds")
    else:
        print("90% rise time: Not reached.")

    if settling_time is not None:
        print(f"Settling time (5% band): {settling_time:.6f} seconds")
    else:
        print("Settling time: Not settled within the recording.")

#     plt.figure(figsize=(10, 5))
#     plt.plot(times, velocities, label="Measured velocity")
#     plt.xlabel("Time (s)")
#     plt.ylabel(ylabel)
#     plt.title(title)
#     plt.grid(True)
#     plt.legend()
#     plt.tight_layout()
#     plt.show()


# plot_velocity(
#     "x_linear_data.txt",
#     "velocity",
#     "Linear Velocity Step Response",
#     "Linear velocity (m/s)"
# )

# plot_velocity(
#     "z_angular_data.txt",
#     "velocity",
#     "Angular Velocity Step Response",
#     "Angular velocity (rad/s)"
# )
    plt.figure(figsize=(10, 5))
    plt.plot(times, velocities, label="Measured velocity")

    plt.axvline(
        COMMAND_DELAY,
        linestyle=":",
        label="Command starts"
    )
    plt.axhline(
        steady_value,
        linestyle="--",
        label=f"Settled estimate = {steady_value:.4f}"
    )
    plt.axhline(
        threshold,
        linestyle=":",
        label="90% threshold"
    )

    if crossing_time is not None:
        plt.scatter(
            crossing_time,
            velocities[index],
            zorder=3,
            label=f"90% rise time = {rise_time:.3f} s"
        )

    if settling_time_absolute is not None:
        plt.axvline(
            settling_time_absolute,
            linestyle="-.",
            label=f"Settling time = {settling_time:.3f} s"
        )

    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

plot_velocity(
    "x_linear_data.txt",
    "velocity",
    "Linear Velocity Step Response",
    "Linear velocity (m/s)"
)

plot_velocity(
    "z_angular_data.txt",
    "velocity",
    "Angular Velocity Step Response",
    "Angular velocity (rad/s)"
)

plt.show()
