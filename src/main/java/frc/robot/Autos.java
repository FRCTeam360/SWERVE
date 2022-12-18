// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class Autos {

    private static DriveTrain driveTrain = DriveTrain.getInstance();
    // This will load the file "FullAuto.path" and generate it with a max velocity
    // of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    private static ArrayList<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("five ball", new PathConstraints(4, 4));

    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.
    private static HashMap<String, Command> eventMap = new HashMap<>() {{
        put("marker 1", new PrintCommand("passed marker 1"));
        put("marker 2", new PrintCommand("passed marker 2"));
    }};

    // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    // eventMap.put("intakeDown", new IntakeDown());

    // Create the AutoBuilder. This only needs to be created once when robot code
    // starts, not every time you want to create an auto command. A good place to
    // put this is in RobotContainer along with your subsystems.
    private static SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            driveTrain::getPose, // Pose2d supplier
            driveTrain::setPose, // Pose2d consumer, used to reset odometry at the beginning of auto
            driveTrain.m_kinematics, // SwerveDriveKinematics
            new PIDConstants(2.9495, 0.0, 0.0), // PID constants to correct for translation error (used to create the X
                                                // and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation
                                             // controller)
            driveTrain::setStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            driveTrain // The drive subsystem. Used to properly set the requirements of path following
                       // commands
    );

    Command fullAuto = autoBuilder.fullAuto(pathGroup);

    public static Command getAuto(){
        return autoBuilder.fullAuto(pathGroup);
    }
}
