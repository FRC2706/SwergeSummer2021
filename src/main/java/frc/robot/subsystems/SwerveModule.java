// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.AnalogEncoder;

import frc.robot.Config;
import frc.robot.subsystems.ContinousPIDSparkMax;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SwerveModule {

    private int m_moduleIndex;

    private CANSparkMax m_driveMotor;
    private CANPIDController m_drivePIDController; 
    private CANEncoder m_driveEncoder;

    private CANSparkMax m_turningMotor;
    private CANEncoder m_turningEncoder;
    private CANPIDController m_turningPIDController;
  
    //@todo: lamprey encoder
    //AnalogEncoder m_analogEncoder;

    private int m_driveEncoderCPR;
    private int m_turningEncoderCPR;

    private NetworkTable      swerveModuleTable;
    private NetworkTableEntry desiredSpeedEntry ;
    private NetworkTableEntry desiredAngleEntry;
    private NetworkTableEntry currentSpeedEntry;
    private NetworkTableEntry currentAngleEntry;
    private NetworkTableEntry speedError;
    private NetworkTableEntry angleError;

    /**
     * Constructs a SwerveModule.
     *
     * @param moduleIndex   ID for the drive motor. (TBD)
     *        0 - frontLeft/single module
     *        1 - frontRight  
     *        2 - rearLeft
     *        3 - rearRight
     */
    public SwerveModule( int moduleIndex ) {
        //module index
        m_moduleIndex = moduleIndex;

        //driver motor controller
        m_driveMotor = new CANSparkMax(Config.moduleConstants[moduleIndex].driveConstants.kDriveMotorChannel, 
                                       MotorType.kBrushless);
        // Factory Default to prevent unexpected behaviour
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.setInverted(Config.moduleConstants[moduleIndex].driveConstants.kDriveMotorInverted);

        m_drivePIDController = m_driveMotor.getPIDController();
        m_drivePIDController.setOutputRange( Config.moduleConstants[moduleIndex].driveCANPIDConstants.minPower, 
                                             Config.moduleConstants[moduleIndex].driveCANPIDConstants.maxPower);
        m_drivePIDController.setFF(Config.moduleConstants[moduleIndex].driveCANPIDConstants.kFF);
        m_drivePIDController.setP(Config.moduleConstants[moduleIndex].driveCANPIDConstants.kP);
        m_drivePIDController.setI(Config.moduleConstants[moduleIndex].driveCANPIDConstants.kI);
        m_drivePIDController.setD(Config.moduleConstants[moduleIndex].driveCANPIDConstants.kD);
        m_drivePIDController.setIZone(Config.moduleConstants[moduleIndex].driveCANPIDConstants.kIZone);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoderCPR = m_driveEncoder.getCountsPerRevolution();
        m_driveEncoder.setVelocityConversionFactor(Config.moduleConstants[moduleIndex].encoderConstants.kVelocityConversionFactor);

        //turning motor controller
        m_turningMotor = new CANSparkMax(Config.moduleConstants[moduleIndex].driveConstants.kTurningMotorChannel,
                                         MotorType.kBrushless);
        m_turningMotor.restoreFactoryDefaults();
        m_turningMotor.setInverted(Config.moduleConstants[moduleIndex].driveConstants.kTurningMotorInverted);

        m_turningPIDController = m_turningMotor.getPIDController();
        m_turningPIDController.setOutputRange( Config.moduleConstants[moduleIndex].turnCANPIDConstants.minPower, 
                                             Config.moduleConstants[moduleIndex].turnCANPIDConstants.maxPower);
        m_turningPIDController.setFF(Config.moduleConstants[moduleIndex].turnCANPIDConstants.kFF);
        m_turningPIDController.setP(Config.moduleConstants[moduleIndex].turnCANPIDConstants.kP);
        m_turningPIDController.setI(Config.moduleConstants[moduleIndex].turnCANPIDConstants.kI);
        m_turningPIDController.setD(Config.moduleConstants[moduleIndex].turnCANPIDConstants.kD);
        m_turningPIDController.setIZone(Config.moduleConstants[moduleIndex].turnCANPIDConstants.kIZone);

        m_turningEncoder = m_turningMotor.getEncoder();
        m_turningEncoderCPR = m_turningEncoder.getCountsPerRevolution();
        m_turningEncoder.setPositionConversionFactor(Config.moduleConstants[moduleIndex].encoderConstants.kPositionConversionFactor );

        // network table entries
        String tableName = "SwerveModule " + m_moduleIndex;
        // Get the swerve module table with module index
        swerveModuleTable = NetworkTableInstance.getDefault().getTable(tableName);

        // Create the entries
        desiredSpeedEntry = swerveModuleTable.getEntry("Desired speed (m/s)");
        desiredAngleEntry = swerveModuleTable.getEntry("Desired angle (radians)");
        currentSpeedEntry = swerveModuleTable.getEntry("Current speed (m/s)");
        currentAngleEntry = swerveModuleTable.getEntry("Current angle (radians)");
        speedError = swerveModuleTable.getEntry("speed Error");
        angleError = swerveModuleTable.getEntry("angle Error");

    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getModuleCurrentSpeedMetersPerSecond(),
                                     getModuleCurrentAngle());
    }

    public double getModuleCurrentSpeedMetersPerSecond()
    {
        //m_driveEncoder.getVelocity();  //--> unit RPM --> m/s
        return (m_driveEncoder.getVelocity()
                * (Config.moduleConstants[m_moduleIndex].encoderConstants.kWheelDiameterMeters * Math.PI)
                / Config.CANDEncoderConstants.SECOND_PER_MINUTE);
    }

    public Rotation2d getModuleCurrentAngle()
    {
        //m_turningEncoder.getPosition(); //native units of rotation
        return (new Rotation2d(m_turningEncoder.getPosition() *  (2 * Math.PI)));
    }

    public double convertSpeedToRPM( double speedMetersPerSecond )
    {
        return (speedMetersPerSecond
                * Config.CANDEncoderConstants.SECOND_PER_MINUTE
                / (Config.moduleConstants[m_moduleIndex].encoderConstants.kWheelDiameterMeters * Math.PI) );
    }

    public double convertAngleToPos( double angleRadius )
    {
        return angleRadius / ( 2*Math.PI );
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getModuleCurrentAngle());

        m_drivePIDController.setReference(convertSpeedToRPM(state.speedMetersPerSecond), 
                                          ControlType.kVelocity);
   
        Rotation2d angleReference = ContinousPIDSparkMax.calculate(state.angle, getModuleCurrentAngle());

        m_turningPIDController.setReference(convertAngleToPos(angleReference.getRadians()), ControlType.kPosition);

        // send to the network table for debugging
        updateNetworkTable(state.speedMetersPerSecond, state.angle.getRadians());
    }

    public void updateNetworkTable( double desiredSpeed, double desiredAngle )
    {
        desiredSpeedEntry.setDouble(desiredSpeed);
        desiredAngleEntry.setDouble(desiredAngle);
        currentSpeedEntry.setDouble(getModuleCurrentSpeedMetersPerSecond());
        currentAngleEntry.setDouble(getModuleCurrentAngle().getRadians());
        speedError.setDouble(desiredSpeed - getModuleCurrentSpeedMetersPerSecond());
        angleError.setDouble(desiredAngle - getModuleCurrentAngle().getRadians());

    }

    /** Zeros all the SwerveModule encoders. */
    public void resetEncoders() {

        //not needed for velocity
        m_driveEncoder.setPosition(0);

        //@todo: set to the land field value
        m_turningEncoder.setPosition(0);
    }

    //only for the single module
    public double getCurrentDriveDistance()
    {
        return (m_driveEncoder.getPosition() * (Config.moduleConstants[m_moduleIndex].encoderConstants.kWheelDiameterMeters * Math.PI));

    }
}