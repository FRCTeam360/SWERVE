// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class AutoDrive extends SwerveControllerCommand{
    private static DriveTrain driveTrain = DriveTrain.getInstance();
    public static double angleGoal;
    public AutoDrive(Trajectory trajectory, double angle) {
        
        //this constructor is a hellhole of functional interfaces
        //i dont know how i got it to work
        //so good luck repurposing or reimplementing it
        super(
            trajectory,
            driveTrain::getPose, //get from drivetrain
            driveTrain.getKinematics(), //get from drivetrain
            new PIDController(1, 0, 0), //FIXME might wanna tune this
            new PIDController(1, 0, 0), //FIXME might wanna tune this
            new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)), //FIXME add profiling for constraints, also tune
            AutoDrive::getAngle, 
            driveTrain::setStates,
            driveTrain
        );

        angleGoal = angle;

    }

    public static Rotation2d getAngle() {
        Rotation2d rot = new Rotation2d(angleGoal);

        return rot;
    }

    private void setStates(SwerveModuleState[] states){}
}
