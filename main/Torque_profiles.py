import numpy as np
import math
import matplotlib.pyplot as plt


class TorqueProfiles:

    def __init__(self, time_interval=1000, max_y=1, loop_frequency=200, trapezoid_portion = 5, pulse_portion = 6):
        self.time = time_interval  # [ms]
        self.max_y = max_y
        self.instances = int(time_interval * loop_frequency / 1000)
        self.x, self.step = np.linspace(0, self.time, self.instances + 1, retstep=True)  # Create x with elements for frequency of 200Hz
        self.l = len(self.x)
        self.loop_frequency = loop_frequency

        self.trapezoid(trapezoid_portion)
        self.triangle()
        self.sinus()
        self.pulse(pulse_portion)
        self.smoothed_trapezoid(trapezoid_portion)

    def trapezoid(self, width_portion=5):
        portion = width_portion
        y_trap = []
        edge = math.ceil(self.l / portion)
        middle = math.floor(self.l * (portion - 1) / portion)
        k = self.max_y / (self.step * edge)

        for i in range(self.l):
            if i < edge:
                y_trap.append(k * self.x[i])
            elif i < middle:
                y_trap.append(self.max_y)
            else:
                y_trap.append(-k * (self.x[i] - self.time))
        self.y_trap = y_trap
        return y_trap

    def triangle(self):
        y_tri = []
        k = self.max_y * 2 / self.time

        for i in range(self.l):
            if i < math.floor(self.l / 2):
                y_tri.append(k * self.x[i])
            else:
                y_tri.append(-k * (self.x[i] - self.time))
        self.y_tri = y_tri
        return y_tri

    def sinus(self):
        y_sin = self.max_y / 2 * (-np.cos(2 * np.pi * self.x / self.time) + 1)
        self.y_sin = y_sin
        return y_sin

    def pulse(self, width_portion=6):
        pulse_width = width_portion
        y_pulse = []

        for i in range(self.l):
            if pulse_width % 2 == 0:
                if i < (pulse_width / 2 - 0.5) * self.l / pulse_width or i > (pulse_width / 2 + 0.5) * self.l / pulse_width:
                    y_pulse.append(0)
                else:
                    y_pulse.append(self.max_y)
            else:
                if i < math.floor(math.floor(pulse_width / 2) * self.l / pulse_width) or i > math.floor(
                        math.ceil(pulse_width / 2) * self.l / pulse_width):
                    y_pulse.append(0)
                else:
                    y_pulse.append(self.max_y)
        self.y_pulse = y_pulse                    
        return y_pulse

    def smoothed_trapezoid(self, width_portion = 5):
        portion = width_portion
        y_smtrap = []
        edge = math.ceil(self.l / portion)
        middle = math.floor(self.l * (portion - 1) / portion)
        k = self.max_y / (self.step * edge)

        for i in range(self.l):
            if i < edge:
                y_smtrap.append((np.cos(np.pi * self.x[i] * portion / self.time + np.pi) + 1) * self.max_y / 2)
            elif i < middle:
                y_smtrap.append(self.max_y)
            else:
                if portion % 2 == 0:
                    y_smtrap.append((-np.cos(np.pi * self.x[i] * portion / self.time) + 1) * self.max_y / 2)
                else:
                    y_smtrap.append((np.cos(np.pi * self.x[i] * portion / self.time) + 1) * self.max_y / 2)
        self.y_smtrap = y_smtrap
        return y_smtrap


if __name__ == "__main__":
    t_prof = TorqueProfiles()
    y_trap = t_prof.y_trap
    y_tri = t_prof.y_tri
    y_sin = t_prof.y_sin
    y_pulse = t_prof.y_pulse
    y_smtrap = t_prof.y_smtrap
    x = t_prof.x

    # Create a 2x3 grid of subplots
    fig, axes = plt.subplots(2, 3, figsize=(12, 8))

    # First row
    axes[0, 0].plot(x, y_trap)
    axes[0, 0].set_title("Trapezoid Profile")
    axes[0, 0].set_xlabel("Time [ms]")

    axes[0, 1].plot(x, y_tri)
    axes[0, 1].set_title("Triangle Profile")
    axes[0, 1].set_xlabel("Time [ms]")

    axes[0, 2].plot(x, y_sin)
    axes[0, 2].set_title("Sinusoidal Profile")
    axes[0, 2].set_xlabel("Time [ms]")

    # Second row
    axes[1, 0].plot(x, y_pulse)
    axes[1, 0].set_title("Pulse Profile")
    axes[1, 0].set_xlabel("Time [ms]")

    axes[1, 1].plot(x, y_smtrap)
    axes[1, 1].set_title("Smoothed Trapezoid Profile")
    axes[1, 1].set_xlabel("Time [ms]")

    fig.delaxes(axes[1, 2])

    plt.tight_layout()
    plt.savefig("main/torque_profiles.png")
    # plt.show()
