import numpy as np
import math
import matplotlib.pyplot as plt


class TorqueProfiles:
    """
    Class for generating various torque profile shapes, such as trapezoid, triangle,
    sinusoidal, pulse, and smoothed trapezoid profiles. Useful for robotics, control,
    and signal processing applications.
    """

    def __init__(self, time_interval=1000, max_y=1, loop_frequency=200, trapezoid_portion=5, pulse_portion=6):
        """
        Initialize the TorqueProfiles object and precompute all profiles.

        Args:
            time_interval (int): Total time duration in milliseconds. Default is 1000.
            max_y (float): Maximum amplitude of the torque profile. Default is 1.
            loop_frequency (int): Sampling frequency in Hz. Default is 200.
            trapezoid_portion (int): Number of portions for the trapezoid profile. Default is 5.
            pulse_portion (int): Number of portions for the pulse profile. Default is 6.
        """
        self.time = time_interval  # [ms]
        self.max_y = max_y
        self.instances = int(time_interval * loop_frequency / 1000)  # Number of samples
        # Create time vector (x) and step size for the given frequency
        self.x, self.step = np.linspace(0, self.time, self.instances + 1, retstep=True)
        self.l = len(self.x)
        self.loop_frequency = loop_frequency

        # Precompute all profiles
        self.trapezoid(trapezoid_portion)
        self.triangle()
        self.sinus()
        self.pulse(pulse_portion)
        self.smoothed_trapezoid(trapezoid_portion)

    def trapezoid(self, width_portion=5):
        """
        Generate a trapezoid torque profile.

        Args:
            width_portion (int): Number of portions for ramp and flat regions.

        Returns:
            list: Trapezoid profile values.
        """
        portion = width_portion
        y_trap = []
        edge = math.ceil(self.l / portion)  # Length of ramp up/down
        middle = math.floor(self.l * (portion - 1) / portion)  # End of flat top
        k = self.max_y / (self.step * edge)  # Slope

        for i in range(self.l):
            if i < edge:
                # Ramp up
                y_trap.append(k * self.x[i])
            elif i < middle:
                # Flat region
                y_trap.append(self.max_y)
            else:
                # Ramp down
                y_trap.append(-k * (self.x[i] - self.time))
        self.y_trap = y_trap
        return y_trap

    def triangle(self):
        """
        Generate a triangle torque profile.

        Returns:
            list: Triangle profile values.
        """
        y_tri = []
        k = self.max_y * 2 / self.time  # Slope

        for i in range(self.l):
            if i < math.floor(self.l / 2):
                # Ramp up
                y_tri.append(k * self.x[i])
            else:
                # Ramp down
                y_tri.append(-k * (self.x[i] - self.time))
        self.y_tri = y_tri
        return y_tri

    def sinus(self):
        """
        Generate a sinusoidal torque profile.

        Returns:
            np.ndarray: Sinusoidal profile values.
        """
        # Sinusoidal profile from 0 to max_y
        y_sin = self.max_y / 2 * (-np.cos(2 * np.pi * self.x / self.time) + 1)
        self.y_sin = y_sin
        return y_sin

    def pulse(self, width_portion=6):
        """
        Generate a pulse torque profile.

        Args:
            width_portion (int): Number of portions to control pulse width.

        Returns:
            list: Pulse profile values.
        """
        pulse_width = width_portion
        y_pulse = []

        for i in range(self.l):
            if pulse_width % 2 == 0:
                # Even portion: define pulse center width
                if i < (pulse_width / 2 - 0.5) * self.l / pulse_width or i > (pulse_width / 2 + 0.5) * self.l / pulse_width:
                    y_pulse.append(0)
                else:
                    y_pulse.append(self.max_y)
            else:
                # Odd portion: define pulse center width
                if i < math.floor(math.floor(pulse_width / 2) * self.l / pulse_width) or i > math.floor(
                        math.ceil(pulse_width / 2) * self.l / pulse_width):
                    y_pulse.append(0)
                else:
                    y_pulse.append(self.max_y)
        self.y_pulse = y_pulse
        return y_pulse

    def smoothed_trapezoid(self, width_portion=5):
        """
        Generate a smoothed trapezoid torque profile using cosine ramps.

        Args:
            width_portion (int): Number of portions for ramp and flat regions.

        Returns:
            list: Smoothed trapezoid profile values.
        """
        portion = width_portion
        y_smtrap = []
        edge = math.ceil(self.l / portion)
        middle = math.floor(self.l * (portion - 1) / portion)

        for i in range(self.l):
            if i < edge:
                # Cosine ramp up
                y_smtrap.append((np.cos(np.pi * self.x[i] * portion / self.time + np.pi) + 1) * self.max_y / 2)
            elif i < middle:
                # Flat region
                y_smtrap.append(self.max_y)
            else:
                # Cosine ramp down
                if portion % 2 == 0:
                    y_smtrap.append((-np.cos(np.pi * self.x[i] * portion / self.time) + 1) * self.max_y / 2)
                else:
                    y_smtrap.append((np.cos(np.pi * self.x[i] * portion / self.time) + 1) * self.max_y / 2)
        self.y_smtrap = y_smtrap
        return y_smtrap


if __name__ == "__main__":
    # Example usage: instantiate and plot all torque profiles
    t_prof = TorqueProfiles()
    y_trap = t_prof.y_trap
    y_tri = t_prof.y_tri
    y_sin = t_prof.y_sin
    y_pulse = t_prof.y_pulse
    y_smtrap = t_prof.y_smtrap
    x = t_prof.x

    # Create a 2x3 grid of subplots for visualization
    fig, axes = plt.subplots(2, 3, figsize=(12, 8))

    # First row: Trapezoid, Triangle, Sinusoidal
    axes[0, 0].plot(x, y_trap)
    axes[0, 0].set_title("Trapezoid Profile")
    axes[0, 0].set_xlabel("Time [ms]")

    axes[0, 1].plot(x, y_tri)
    axes[0, 1].set_title("Triangle Profile")
    axes[0, 1].set_xlabel("Time [ms]")

    axes[0, 2].plot(x, y_sin)
    axes[0, 2].set_title("Sinusoidal Profile")
    axes[0, 2].set_xlabel("Time [ms]")

    # Second row: Pulse, Smoothed Trapezoid
    axes[1, 0].plot(x, y_pulse)
    axes[1, 0].set_title("Pulse Profile")
    axes[1, 0].set_xlabel("Time [ms]")

    axes[1, 1].plot(x, y_smtrap)
    axes[1, 1].set_title("Smoothed Trapezoid Profile")
    axes[1, 1].set_xlabel("Time [ms]")

    # Remove unused subplot
    fig.delaxes(axes[1, 2])

    plt.tight_layout()
    plt.savefig("./torque_profiles.png")
    # plt.show()