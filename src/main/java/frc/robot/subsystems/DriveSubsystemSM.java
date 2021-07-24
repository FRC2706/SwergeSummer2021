// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Config.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import frc.robot.Config;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystemSM extends SubsystemBase {
    // Robot swerve modules
    private final SwerveModule m_singleModule = new SwerveModule(0);

    // The gyro sensor
    private final Gyro m_gyro = new ADXRS450_Gyro();

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Config.kDriveKinematics, m_gyro.getRotation2d());

    /** Creates a new DriveSubsystem. */
    public DriveSubsystemSM() {
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(m_gyro.getRotation2d(), m_singleModule.getState());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        //         fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
        //                 : new ChassisSpeeds(xSpeed, ySpeed, rot));
        // SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        // m_frontLeft.setDesiredState(swerveModuleStates[0]);
        // m_frontRight.setDesiredState(swerveModuleStates[1]);
        // m_rearLeft.setDesiredState(swerveModuleStates[2]);
        // m_rearRight.setDesiredState(swerveModuleStates[3]);



        //for the single module: chassis speed = single module state
        ChassisSpeeds chassisSpeed = fieldRelative ? 
                                     ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                                     : new ChassisSpeeds(xSpeed, ySpeed, rot);

        //convert the chassisSpeed to the module state
        SwerveModuleState desiredState = new SwerveModuleState(Math.sqrt(xSpeed*xSpeed+ySpeed*ySpeed), new Rotation2d(Math.atan(ySpeed/xSpeed) + rot));
        m_singleModule.setDesiredState(desiredState);

    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        //SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_singleModule.setDesiredState(desiredStates[0]);
        
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_singleModule.resetEncoders();
    
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (Config.kGyroReversed ? -1.0 : 1.0);
    }
}
