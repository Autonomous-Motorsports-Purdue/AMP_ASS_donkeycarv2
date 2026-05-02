import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

BUFFER_SIZE = 200

GROUPS = [
    (["accX", "accY", "accZ"], "Accelerometer", "m/s²"),
    (["gyrX", "gyrY", "gyrZ"], "Gyroscope",     "rad/s"),
    (["yaw", "pitch", "roll"], "Attitude",       "degrees"),
]

COLORS = ["#e74c3c", "#2ecc71", "#3498db"]


class PhoneSensorsVisualizer():
    def __init__(self):
        self.buffers = {}
        for keys, _, _ in GROUPS:
            for k in keys:
                self.buffers[k] = deque(maxlen=BUFFER_SIZE)
        self.step = 0

        self.fig, self.axes = plt.subplots(len(GROUPS), 1, figsize=(10, 7), squeeze=False)
        self.axes = [ax[0] for ax in self.axes]
        self.fig.suptitle("Phone Sensors", fontsize=14)

        self.lines = []
        for i, (keys, title, ylabel) in enumerate(GROUPS):
            ax = self.axes[i]
            ax.set_title(title)
            ax.set_ylabel(ylabel)
            ax.set_xlabel("Step")
            ax.grid(True, alpha=0.3)
            group_lines = []
            for j, k in enumerate(keys):
                (line,) = ax.plot([], [], label=k, color=COLORS[j], linewidth=1)
                group_lines.append((k, line))
            ax.legend(loc="upper left", fontsize=8)
            self.lines.append(group_lines)

        self.fig.tight_layout(rect=[0, 0, 1, 0.96])
        plt.ion()
        plt.show(block=False)

    def run(self, accX, accY, accZ, gyrX, gyrY, gyrZ, yaw, pitch, roll):
        values = {
            "accX": accX, "accY": accY, "accZ": accZ,
            "gyrX": gyrX, "gyrY": gyrY, "gyrZ": gyrZ,
            "yaw": yaw,   "pitch": pitch, "roll": roll,
        }

        self.step += 1
        for k, v in values.items():
            if v is not None:
                self.buffers[k].append(v)
            elif len(self.buffers[k]) > 0:
                self.buffers[k].append(self.buffers[k][-1])

        for i, (keys, _, _) in enumerate(GROUPS):
            ax = self.axes[i]
            for k, line in self.lines[i]:
                buf = list(self.buffers[k])
                if buf:
                    x = list(range(self.step - len(buf) + 1, self.step + 1))
                    line.set_data(x, buf)
            ax.relim()
            ax.autoscale_view()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()