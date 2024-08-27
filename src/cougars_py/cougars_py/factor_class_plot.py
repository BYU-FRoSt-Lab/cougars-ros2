import matplotlib.pyplot as plt

class Series:
    def __init__(self, name, color='r', alpha=0.7):
        self.name = name
        self.color = color
        self.alpha = alpha
        self.timestamps = []
        self.values = []
        self.pose_keys = []
        self.line = None
        self.scatter = None
        self.keyed_lines = []

    def add_measurement(self, value, timestamp, pose_key=None):
        """Add a new measurement with a specific timestamp and optionally link to a pose key."""
        self.timestamps.append(timestamp)
        self.values.append(value)
        self.pose_keys.append(pose_key)
        if pose_key is not None:
            try:
                idx = self.pose_keys.index(posekey)
                line = self.ax.plot([self.gps_timestamps[-1], self.timestamps[idx]],
                                    [self.gps_values[-1], self.values[idx]], 'g--')[0]
                self.gps_keyed_lines.append(line)
            except ValueError:
                print(f"Pose key {posekey} not found in delta measurements.")

    def add_delta_measurement(self, delta_value, timestamp, pose_key=None):
        """Add a new delta measurement with a specific timestamp and pose key."""
        self.timestamps.append(timestamp)
        if self.values:
            new_value = self.values[-1] + delta_value
        else:
            new_value = delta_value
        self.values.append(new_value)
        self.pose_keys.append(pose_key)

    def plot(self, ax):
        """Plot the series on the given axis."""
        if self.line is None:
            self.line, = ax.plot([], [], '-', color=self.color, alpha=self.alpha, label=self.name)
            self.scatter = ax.scatter([], [], color=self.color, alpha=0.8)

        print(self.timestamps, self.values)
        self.line.set_data(self.timestamps, self.values)
        self.scatter.set_offsets(list(zip(self.timestamps, self.values)))

        if len(self.timestamps) > 1:
            ax.set_xlim(min(self.timestamps), max(self.timestamps))
            ax.set_ylim(min(self.values) - 1, max(self.values) + 1)
        else:
            ax.set_xlim(self.timestamps[0] - 1, self.timestamps[0] + 1)
            ax.set_ylim(self.values[0] - 1, self.values[0] + 1)

        # Draw pose keys as text on the plot
        for i, (x, y, key) in enumerate(zip(self.timestamps, self.values, self.pose_keys)):
            if key is not None:
                ax.text(x, y, str(key), color='black', ha='center', va='center', fontsize=10, weight='bold')


class Plotter:
    def __init__(self, series_list):
        # Store the series objects
        self.series_list = series_list

        # Create figure and axis
        self.figure, self.ax = plt.subplots()

        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value')
        self.ax.grid(True)

        # Display the initial plot
        plt.ion()  # Interactive mode on
        plt.show()

    def update_plot(self):
        """Update the plot with new measurements."""
        for series in self.series_list:
            series.plot(self.ax)

        plt.draw()
        plt.pause(0.1)

    def show(self):
        """Display the plot."""
        plt.show(block=True)
