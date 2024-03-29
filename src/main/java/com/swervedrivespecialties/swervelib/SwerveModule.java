package com.swervedrivespecialties.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    Object getDriveMotor();

    Object getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    /**
     * Get module drive velocity
     * 
     * @return drive velocity in m/s
     */
    double getDriveVelocity();

    /**
     * Get module drive distance
     * 
     * @return drive distance in meters
     */
    double getDriveDistance();

    /**
     * Get module steer angle
     * 
     * @return steer angle in radians from [0, 2pi)
     */
    double getSteerAngle();

    void resetSteerEncoder();

    default SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
    }

    void set(double driveVoltage, double steerAngle);
}
