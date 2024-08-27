# Standard library
from matplotlib import pyplot as plt

# Internal library
from simulators.time_stepping import TimeStepping
from models.differential_drive import DifferentialDrive
from controllers.purepursuit.vfh_purpursuit import VFHPurePursuit
from trajectory_generators.simple_generator import SimpleGenerator


if __name__ == "__main__":
    wheel_base = 0.53

    model = DifferentialDrive(wheel_base)

    trajectory = SimpleGenerator(model)

    trajectory.generate("global_trajectory.csv", nx=3, nu=2,
                        is_derivative=True)

    controller = VFHPurePursuit(model, trajectory)

    simulator = TimeStepping(model, trajectory, controller, None, t_max=120)

    simulator.run(0.0)

    _, ax1 = plt.subplots(1, 1)

    ax1.set_box_aspect(1)

    print("simulator.x_out: ", simulator.x_out[:, -2])

    ax1.plot(simulator.x_out[0, :], simulator.x_out[1, :], "r",
             label="Tracking performance")

    ax1.plot(
        [path[0] for path in trajectory.x],
        [path[1] for path in trajectory.x],
        "--b", label="Trajectory",
    )
    ax1.legend()
    ax1.set_xlabel("X [m]")
    ax1.set_ylabel("Y [m]")
    plt.tight_layout()
    _, (ax2, ax3) = plt.subplots(1, 2)

    ax2.plot(simulator.t_out[:len(simulator.u_out[0, :])],
             simulator.u_out[0, :])

    ax3.plot(simulator.t_out[:len(simulator.u_out[0, :])],
             simulator.u_out[1, :])
    
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Velocity [m/s]")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Angular Velocity [rad/s]")
    plt.tight_layout()
    _, (ax4, ax5) = plt.subplots(1, 2)

    ax4.plot(simulator.t_out, simulator.dudt_out[0, :])

    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Acceleration [m/s^2]")

    ax5.plot(simulator.t_out, simulator.dudt_out[1, :])
    ax5.set_xlabel("Time [s]")
    ax5.set_ylabel("Angular Acceleration [rad/s^2]")
    plt.tight_layout()
    plt.show()
