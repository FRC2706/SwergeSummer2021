// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.config.Config;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
    // Robot swerve modules
    //1 is front left
    private final SwerveModule m_frontLeft = new SwerveModule(1);

    //2 is rear left
    private final SwerveModule m_rearLeft = new SwerveModule(2);

    //3 is rear right
    private final SwerveModule m_rearRight = new SwerveModule(3);

    //4 is front right
    private final SwerveModule m_frontRight = new SwerveModule(4);

    // The gyro sensor
    private final Gyro m_gyro = new ADXRS450_Gyro();

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Config.kDriveKinematics, pigeonRotation());

    // Singleton instance of the DriveSubsytem class
    private static DriveSubsystem instance;
    private PigeonIMU m_pigeon;
    private PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
    private NetworkTableEntry xOdometry, yOdometry, rotOdometry;

    /** Creates a new DriveSubsystem. */
    private DriveSubsystem() {
        m_pigeon = new PigeonIMU(Config.OIConstants.PIGEON_ID);
    }

    public static DriveSubsystem getInstance() {
        if (instance == null) {
            instance = new DriveSubsystem();
        }
        return instance;
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        Pose2d newPose = m_odometry.update(pigeonRotation(), m_frontLeft.getState(), m_rearLeft.getState(),
                m_rearRight.getState(), m_frontRight.getState());

        var table = NetworkTableInstance.getDefault().getTable("DrivetrainOdometry");
        xOdometry = table.getEntry("xOdometry");
        yOdometry = table.getEntry("yOdometry");
        rotOdometry = table.getEntry("rotOdometry");

        xOdometry.setDouble(newPose.getX());
        yOdometry.setDouble(newPose.getY());
        rotOdometry.setDouble(newPose.getRotation().getDegrees());
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
        m_odometry.resetPosition(pose, pigeonRotation());
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
        var swerveModuleStates = Config.kDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeonRotation())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Config.AutoConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_rearLeft.setDesiredState(swerveModuleStates[1]);
        m_rearRight.setDesiredState(swerveModuleStates[2]);
        m_frontRight.setDesiredState(swerveModuleStates[3]);

    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Config.AutoConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_rearLeft.setDesiredState(desiredStates[1]);
        m_rearRight.setDesiredState(desiredStates[2]);
        m_frontRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void updateTurningEncodersFromLamprey() {
        m_frontLeft.updateTurningEncoderFromLamprey();
        m_rearLeft.updateTurningEncoderFromLamprey();
        m_rearRight.updateTurningEncoderFromLamprey();
        m_frontRight.updateTurningEncoderFromLamprey();
    }

    public void stopMotors() {
        m_frontLeft.stopMotors();
        m_rearLeft.stopMotors();
        m_rearRight.stopMotors();
        m_frontRight.stopMotors();
    }
    /** Zeroes the heading of the robot. */
    public void setHeading(double heading) {
        m_pigeon.setFusedHeading(heading);
    }

        /**
     * Returns true if the pigeon has been defined
     * @return True if the pigeon is defined, false otherwise
     */
    public final boolean hasPigeon() {
        return m_pigeon != null;
    }
    
    /**
     * A getter for the pigeon
     * @return The pigeon
     */
    public final PigeonIMU getPigeon() {
        return m_pigeon;
    }

    public Rotation2d pigeonRotation()
    {
        return Rotation2d.fromDegrees(getCurrentAngle());
    }
    
    /**
     * Tries to get the current angle as reported by the pigeon
     * @return The current heading (In degrees) or 0 if there is no pigeon.
     */
    private final double getCurrentAngle() {
        if (!hasPigeon()) return 0d;
        m_pigeon.getFusedHeading(fusionStatus);
        return fusionStatus.heading;
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return pigeonRotation().getDegrees();
    }

}