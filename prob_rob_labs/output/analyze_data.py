from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

DATA_DIR = Path.home() / "ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/output"

linear_file = DATA_DIR / "x_data.txt"
angular_file = DATA_DIR / "z_data.txt"


def load_data(path):
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")

    data = np.atleast_1d(np.loadtxt(path, dtype=float))
    data = data[np.isfinite(data)]

    if data.size == 0:
        raise ValueError(f"No valid data in {path}")

    return data


linear = load_data(linear_file)
angular = load_data(angular_file)


def print_statistics(name, data, unit):
    print(f"\n{name}")
    print(f"Number of samples: {len(data)}")
    print(f"Mean: {np.mean(data):.8f} {unit}")

    if len(data) > 1:
        print(f"Standard deviation (sample): {np.std(data, ddof=1):.8f} {unit}")
    else:
        print("Sample standard deviation requires at least two samples.")


print_statistics("Linear velocity (x)", linear, "m/s")
print_statistics("Angular velocity (z)", angular, "rad/s")

fig, axes = plt.subplots(1, 2, figsize=(12, 5))

axes[0].hist(linear, bins="auto", edgecolor="black")
axes[0].axvline(
    np.mean(linear), linestyle="--",
    label=f"Mean = {np.mean(linear):.5f}"
)
axes[0].set_title("Ground-Truth Linear Velocity")
axes[0].set_xlabel("Linear velocity x (m/s)")
axes[0].set_ylabel("Frequency")
axes[0].legend()
axes[0].grid(axis="y", alpha=0.25)

axes[1].hist(angular, bins="auto", edgecolor="black")
axes[1].axvline(
    np.mean(angular), linestyle="--",
    label=f"Mean = {np.mean(angular):.5f}"
)
axes[1].set_title("Ground-Truth Angular Velocity")
axes[1].set_xlabel("Angular velocity z (rad/s)")
axes[1].set_ylabel("Frequency")
axes[1].legend()
axes[1].grid(axis="y", alpha=0.25)

plt.tight_layout()

output_plot = DATA_DIR / "velocity_histograms.png"
plt.savefig(output_plot, dpi=200, bbox_inches="tight")
print(f"\nHistograms saved to: {output_plot}")

plt.show()