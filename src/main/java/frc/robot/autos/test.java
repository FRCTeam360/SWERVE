// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class test extends SequentialCommandGroup {

  private Pose2d startingPose = new Pose2d(3, 4, new Rotation2d(90));

  public static final Trajectory phase1 = TrajectoryGenerator.generateTrajectory(
    new Pose2
    
    
    d(0, 0, new Rotation2d()), 
    List.of(), 
    new Pose2d(2, 0, new Rotation2d()), 
    AutoConstants.config);
  /** Creates a new test. */
  public test() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDrive(phase1, 0, startingPose)
    );
  }
}
