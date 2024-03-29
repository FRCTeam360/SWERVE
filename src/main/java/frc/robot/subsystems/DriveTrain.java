// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;

import static frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {
  private static DriveTrain instance;
  public static final double MAX_VOLTAGE = 12.0;

  private static final double ADJUSTMENT_FACTOR = 0.015;

  private PIDController pitchController = new PIDController(0.015, 0, 0);
  private PIDController rollController = new PIDController(0.015, 0, 0);

  public static Rotation2d[] stateAngles = {new Rotation2d(0.0),new Rotation2d(0.0),new Rotation2d(0.0),new Rotation2d(0.0)};

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
      * SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter()
      * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

  // By default we use a Pigeon for our gyroscope. But if you use another
  // gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating
  // the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
  private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(DRIVETRAIN_PIGEON_ID);
  // FIXME Uncomment if you are using a NavX
  // private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(m_kinematics, m_pigeon.getRotation2d());

  private Pose2d pose; 

  private ChassisSpeeds currentVelocity = new ChassisSpeeds();

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  public static DriveTrain getInstance() {
    if(instance == null) {
      instance = new DriveTrain();
    }
    return instance;
  }

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    // Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    // Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    // Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
    // and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    // Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
    // Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
    // class.

    // By default we will use Falcon 500s in standard configuration. But if you use
    // a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // FIXME Setup motor configuration
    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        // This parameter is optional, but will allow you to see the current state of
        // the module on the dashboard.
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        // This can either be STANDARD or FAST depending on your gear configuration
        Mk4iSwerveModuleHelper.GearRatio.L1,
        // This is the ID of the drive motor
        FRONT_LEFT_MODULE_DRIVE_MOTOR,
        // This is the ID of the steer motor
        FRONT_LEFT_MODULE_STEER_MOTOR,
        // This is the ID of the steer encoder
        FRONT_LEFT_MODULE_STEER_ENCODER,
        // This is how much the steer encoder is offset from true zero (In our case,
        // zero is facing straight forward)
        FRONT_LEFT_MODULE_STEER_OFFSET);

    // We will do the same for the other modules
    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
        Mk4iSwerveModuleHelper.GearRatio.L1,
        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        FRONT_RIGHT_MODULE_STEER_MOTOR,
        FRONT_RIGHT_MODULE_STEER_ENCODER,
        FRONT_RIGHT_MODULE_STEER_OFFSET);

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        Mk4iSwerveModuleHelper.GearRatio.L1,
        BACK_LEFT_MODULE_DRIVE_MOTOR,
        BACK_LEFT_MODULE_STEER_MOTOR,
        BACK_LEFT_MODULE_STEER_ENCODER,
        BACK_LEFT_MODULE_STEER_OFFSET);

    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
        Mk4iSwerveModuleHelper.GearRatio.L1,
        BACK_RIGHT_MODULE_DRIVE_MOTOR,
        BACK_RIGHT_MODULE_STEER_MOTOR,
        BACK_RIGHT_MODULE_STEER_ENCODER,
        BACK_RIGHT_MODULE_STEER_OFFSET);
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    // FIXME Remove if you are using a Pigeon
    // m_pigeon.setFusedHeading(0.0);
    m_pigeon.setYaw(0.0);

    // FIXME Uncomment if you are using a NavX
    // m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    // FIXME Remove if you are using a Pigeon
    // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

    return m_pigeon.getRotation2d();

    // // FIXME Uncomment if you are using a NavX
    // if (m_navx.isMagnetometerCalibrated()) {
    // // We will only get valid fused headings if the magnetometer is calibrated
    // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    // }
    
    // // We have to invert the angle of the NavX so that rotating the robot
    // // counter-clockwise makes the angle increase.
    // return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void adjustAnglePosition(){
    drive(
      new ChassisSpeeds(
        // m_pigeon.getPitch() * ADJUSTMENT_FACTOR, 
        // m_pigeon.getRoll() * ADJUSTMENT_FACTOR, 
        pitchController.calculate(m_pigeon.getPitch(), 0),
        rollController.calculate(m_pigeon.getRoll(), 0),
        0)
    );
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    setStates(m_kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public void setStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    boolean speedIsZero = false;
    for(int i = 0; i<states.length; i++) {
      if(states[i].speedMetersPerSecond == 0) {
        speedIsZero = true;
      } else {
        speedIsZero = false;
        break;
      }
    }
    if(!speedIsZero){
      m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
      m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
      m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
      m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
      for(int i = 0; i<states.length; i++) {
        stateAngles[i] = states[i].angle;
      }
    } else{
      // return angles with no speed
      m_frontLeftModule.set(0.0, stateAngles[0].getRadians());
      m_frontRightModule.set(0.0, stateAngles[1].getRadians());
      m_backLeftModule.set(0.0, stateAngles[2].getRadians());
      m_backRightModule.set(0.0, stateAngles[3].getRadians());
    }
    // pose = odometry.update(getGyroscopeRotation(), states);
  }

  public Pose2d getPose() {
    return pose;
  }

  public SwerveDriveKinematics getKinematics(){
    return m_kinematics;
  }

  public void setPose(Pose2d pose) {
    odometry.resetPosition(pose, getGyroscopeRotation());
  }

  public void resetPose(){
    odometry.resetPosition(new Pose2d(), getGyroscopeRotation());
  }

  public ChassisSpeeds getCurrentVelocity() {
    return currentVelocity;
  }

  public void resetSteerEncoders() {
    m_frontLeftModule.resetSteerEncoder();
    m_frontRightModule.resetSteerEncoder();
    m_backLeftModule.resetSteerEncoder();
    m_backRightModule.resetSteerEncoder();
  }

  @Override
  public void periodic() {

    // System.out.println("max vel " + MAX_VELOCITY_METERS_PER_SECOND);
SwerveModuleState currentFrontLeftModuleState = new SwerveModuleState(m_frontLeftModule.getDriveVelocity(),
    new Rotation2d(m_frontLeftModule.getSteerAngle()));
SwerveModuleState currentFrontRightModuleState = new SwerveModuleState(m_frontRightModule.getDriveVelocity(),
    new Rotation2d(m_frontRightModule.getSteerAngle()));
SwerveModuleState currentBackLeftModuleState = new SwerveModuleState(m_backLeftModule.getDriveVelocity(),
    new Rotation2d(m_backLeftModule.getSteerAngle()));
SwerveModuleState currentBackRightModuleState = new SwerveModuleState(m_backRightModule.getDriveVelocity(),
    new Rotation2d(m_backRightModule.getSteerAngle()));

currentVelocity = m_kinematics.toChassisSpeeds(currentFrontLeftModuleState, currentFrontRightModuleState,
    currentBackLeftModuleState, currentBackRightModuleState);

    pose = odometry.update(getGyroscopeRotation(), currentFrontLeftModuleState, currentFrontRightModuleState,
    currentBackLeftModuleState, currentBackRightModuleState);
    // System.out.println("ratio" + SdsModuleConfigurations.MK4I_L1.getDriveReduction());
    // System.out.println("meters per rotation" + SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI);
  }
}