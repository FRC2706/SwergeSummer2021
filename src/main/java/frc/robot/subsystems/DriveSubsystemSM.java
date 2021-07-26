// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import frc.robot.Config;
import frc.robot.subsystems.SwerveModule;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystemSM extends SubsystemBase {
    // Robot swerve module
    private final SwerveModule m_singleModule = new SwerveModule(0);

    // The gyro sensor
    private final PigeonIMU m_pigeon;
    private PigeonIMU.FusionStatus fusionStatus; 
  
    // Odometry class for tracking robot pose
    // NOTE: for the single module, we use the differenctial drive odometry
    DifferentialDriveOdometry  m_odometry; 

    /** Creates a new DriveSubsystem. */
    public DriveSubsystemSM() {
 
        if( Config.OIConstants.PIGEON_ID != -1)
        {
            m_pigeon = new PigeonIMU(Config.OIConstants.PIGEON_ID);
            fusionStatus = new PigeonIMU.FusionStatus();
        }
        else
        {
            m_pigeon = null;
        }
        
        m_odometry = new DifferentialDriveOdometry( new Rotation2d(), new Pose2d() );
    }

    @Override
    public void periodic() {

        // Update the odometry in the periodic block
        m_odometry.update(Rotation2d.fromDegrees(getCurrentAngleDegrees()), 
                          m_singleModule.getCurrentDriveDistance(), 
                          m_singleModule.getCurrentDriveDistance());
    }

    public double getCurrentAngleDegrees()
    {
        if( m_pigeon != null )
        {
            return m_pigeon.getFusedHeading(fusionStatus) ;
         }
        else
        {
            return 0.0;
        }
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
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getCurrentAngleDegrees()));
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

        ChassisSpeeds chassisSpeeds = fieldRelative ? 
                                     ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, 
                                                                        Rotation2d.fromDegrees(getCurrentAngleDegrees()))
                                     : new ChassisSpeeds(xSpeed, ySpeed, rot);
        
        SwerveModuleState desiredState = convertChassisSpeedsToSMState( chassisSpeeds );
        m_singleModule.setDesiredState(desiredState);
    }

    /**
     * @brief Convert the chassisSpeed to the module state
     *        (for the single module: chassis speed --> single module state)
     *        
     * @param chassisSpeeds
     * @return single module state
     */
    public SwerveModuleState convertChassisSpeedsToSMState( ChassisSpeeds chassisSpeeds )
    {
        double xSpeed = chassisSpeeds.vxMetersPerSecond;
        double ySpeed = chassisSpeeds.vyMetersPerSecond;
        //double omega  = chassisSpeeds.omegaRadiansPerSecond;

        double speed = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed); 
        Rotation2d angle = new Rotation2d( Math.atan(ySpeed/xSpeed)); 

        SwerveModuleState moduleState = new SwerveModuleState(speed, angle);

        return moduleState;
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

    //  /**
    //  * Returns the turn rate of the robot.
    //  *
    //  * @return The turn rate of the robot, in degrees per second
    //  */
    // public double getTurnRate() {
    //     return m_gyro.getRate() * (Config.kGyroReversed ? -1.0 : 1.0);
    // }
}
