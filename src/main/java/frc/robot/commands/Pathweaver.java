// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.nio.file.Path;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pathweaver extends SequentialCommandGroup {
  String trajectoryJSON = "output/Unnamed.wpilib.json";
  String trajectoryJSON_0 = "output/Unnamed_0.wpilib.json";
  Trajectory trajectory = new Trajectory();
  Trajectory trajectory_0 = new Trajectory();

  // Create a voltage constraint to ensure we don't accelerate too fast
  DifferentialDriveVoltageConstraint autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        10);

  // Create config for trajectory
  TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

  // An example trajectory to follow.  All units in meters.
  Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0.25, 0.25), new Translation2d(0.5, -0.25)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0.75, 0, new Rotation2d(0)),
        // Pass config
        config);

  /** Creates a new FollowTrajectory. */
  public Pathweaver(Drivetrain drivetrain) {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      Path trajectoryPath_0 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON_0);
      trajectory_0 = TrajectoryUtil.fromPathweaverJson(trajectoryPath_0);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectories", ex.getStackTrace());
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> {
        drivetrain.resetOdometry(trajectory.getInitialPose());
      }).withTimeout(1),
      new RamseteCommand(
        trajectory, 
        drivetrain::getOdometry,
        new RamseteController(2.0, 0.7), 
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), 
        Constants.kDriveKinematics, 
        drivetrain::getWheelSpeeds, 
        new PIDController(Constants.kPDriveVel, 0, 0), 
        new PIDController(Constants.kPDriveVel, 0, 0),
        drivetrain::tankDriveVolts,
        drivetrain),
      new RunCommand(() -> {
        drivetrain.tankDriveVolts(-1, 1);
      }).withTimeout(1),
      new RunCommand(() -> {
        drivetrain.tankDriveVolts(0, 0);
      }).withTimeout(1.2),
      new RamseteCommand(
        trajectory_0, 
        drivetrain::getOdometry,
        new RamseteController(2.0, 0.7), 
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), 
        Constants.kDriveKinematics, 
        drivetrain::getWheelSpeeds, 
        new PIDController(Constants.kPDriveVel, 0, 0), 
        new PIDController(Constants.kPDriveVel, 0, 0),
        drivetrain::tankDriveVolts,
        drivetrain)
    );
  }
}
