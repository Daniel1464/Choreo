// Copyright (c) Choreo contributors

package choreo.auto;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

/**
 * An object that represents an autonomous routine.
 *
 * <p>This loop is used to handle autonomous trigger logic and schedule commands. This loop should
 * **not** be shared across multiple autonomous routines.
 *
 * @see AutoFactory#newRoutine Creating a routine from a AutoFactory
 */
public class AutoRoutine {
  /** The underlying {@link EventLoop} that triggers are bound to and polled */
  protected final EventLoop loop;

  /** The name of the auto routine this loop is associated with */
  protected final String name;

  /** A boolean utilized in {@link #running()} to resolve trueness */
  protected boolean isActive = false;

  /** A boolean that is true when the loop is killed */
  protected boolean isKilled = false;

  /** The amount of times the routine has been polled */
  protected int pollCount = 0;

  /** The auto factory of the routine. */
  protected final AutoFactory autoFactory;

  /**
   * Creates a new loop with a specific name
   *
   * @param name The name of the loop
   * @param factory The auto factory of the routine(can be null).
   * @see AutoFactory#newRoutine Creating a loop from a AutoFactory
   */
  public AutoRoutine(String name, AutoFactory factory) {
    this.autoFactory = factory;
    this.loop = new EventLoop();
    this.name = name;
  }

  /**
   * A constructor to be used when inhereting this class to instantiate a custom inner loop
   *
   * @param name The name of the loop
   * @param factory The auto factory of the routine(can be null).
   * @param loop The inner {@link EventLoop}
   */
  protected AutoRoutine(String name, EventLoop loop, AutoFactory factory) {
    this.autoFactory = factory;
    this.loop = loop;
    this.name = name;
  }

  /**
   * Returns a {@link Trigger} that is true while this autonomous routine is being polled.
   *
   * <p>Using a {@link Trigger#onFalse(Command)} will do nothing as when this is false the routine
   * is not being polled anymore.
   *
   * @return A {@link Trigger} that is true while this autonomous routine is being polled.
   */
  public Trigger running() {
    return new Trigger(loop, () -> isActive && DriverStation.isAutonomousEnabled());
  }

  /** Polls the routine. Should be called in the autonomous periodic method. */
  public void poll() {
    if (!DriverStation.isAutonomousEnabled() || isKilled) {
      isActive = false;
      return;
    }
    pollCount++;
    loop.poll();
    isActive = true;
  }

  /**
   * Gets the event loop that this routine is using.
   *
   * @return The event loop that this routine is using.
   */
  public EventLoop loop() {
    return loop;
  }

  /**
   * Gets the poll count of the routine.
   *
   * @return The poll count of the routine.
   */
  int pollCount() {
    return pollCount;
  }

  /**
   * Resets the routine. This can either be called on auto init or auto end to reset the routine
   * incase you run it again. If this is called on a routine that doesn't need to be reset it will
   * do nothing.
   */
  public void reset() {
    pollCount = 0;
    isActive = false;
  }

  /** Kills the loop and prevents it from running again. */
  public void kill() {
    CommandScheduler.getInstance().cancelAll();
    if (isKilled) {
      return;
    }
    reset();
    DriverStation.reportWarning("Killed An Auto Loop", true);
    isKilled = true;
  }

  /**
   * Creates a new auto trajectory to be used in an auto routine.
   *
   * @param trajectoryName The name of the trajectory to use.
   * @return A new auto trajectory.
   */
  public AutoTrajectory trajectory(String trajectoryName) {
    Optional<? extends Trajectory<?>> optTrajectory =
            autoFactory.trajectoryCache.loadTrajectory(trajectoryName);
    Trajectory<?> trajectory;
    if (optTrajectory.isPresent()) {
      trajectory = optTrajectory.get();
    } else {
      DriverStation.reportError("Could not load trajectory: " + trajectoryName, false);
      trajectory = new Trajectory<SwerveSample>(trajectoryName, List.of(), List.of(), List.of());
    }
    return trajectory(trajectory);
  }

  /**
   * Creates a new auto trajectory to be used in an auto routine.
   *
   * @param trajectoryName The name of the trajectory to use.
   * @param splitIndex The index of the split trajectory to use.
   * @return A new auto trajectory.
   */
  public AutoTrajectory trajectory(String trajectoryName, int splitIndex) {
    Optional<? extends Trajectory<?>> optTrajectory =
            autoFactory.trajectoryCache.loadTrajectory(trajectoryName, splitIndex);
    Trajectory<?> trajectory;
    if (optTrajectory.isPresent()) {
      trajectory = optTrajectory.get();
    } else {
      DriverStation.reportError("Could not load trajectory: " + trajectoryName, false);
      trajectory = new Trajectory<SwerveSample>(trajectoryName, List.of(), List.of(), List.of());
    }
    return trajectory(trajectory);
  }

  /**
   * Creates a new auto trajectory to be used in an auto routine.
   *
   * @param <SampleType> The type of the trajectory samples.
   * @param trajectory The trajectory to use.
   * @return A new auto trajectory.
   */
  @SuppressWarnings("unchecked")
  public <SampleType extends TrajectorySample<SampleType>> AutoTrajectory trajectory(
          Trajectory<SampleType> trajectory) {
    // type solidify everything
    final BiConsumer<Pose2d, SampleType> solidController =
        (BiConsumer<Pose2d, SampleType>) autoFactory.controller;
    final Optional<Choreo.TrajectoryLogger<SampleType>> solidLogger =
        autoFactory.trajectoryLogger.map(logger -> (Choreo.TrajectoryLogger<SampleType>) logger);
    return new AutoTrajectory(
      trajectory.name(),
      trajectory,
      autoFactory.poseSupplier,
      solidController,
      autoFactory.mirrorTrajectory,
      solidLogger,
      autoFactory.driveSubsystem,
      this,
      autoFactory.bindings);
  }

  /**
   * Creates a command that will poll this event loop and reset it when it is cancelled.
   *
   * @return A command that will poll this event loop and reset it when it is cancelled.
   * @see #cmd(BooleanSupplier) A version of this method that takes a condition to finish the loop.
   */
  public Command cmd() {
    return Commands.run(this::poll)
        .finallyDo(this::reset)
        .until(() -> !DriverStation.isAutonomousEnabled())
        .withName(name);
  }

  /**
   * Creates a command that will poll this event loop and reset it when it is finished or canceled.
   *
   * @param finishCondition A condition that will finish the loop when it is true.
   * @return A command that will poll this event loop and reset it when it is finished or canceled.
   * @see #cmd() A version of this method that doesn't take a condition and never finishes.
   */
  public Command cmd(BooleanSupplier finishCondition) {
    return Commands.run(this::poll)
        .finallyDo(this::reset)
        .until(() -> !DriverStation.isAutonomousEnabled() || finishCondition.getAsBoolean())
        .withName(name);
  }
}
