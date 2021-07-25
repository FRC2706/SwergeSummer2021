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
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import frc.robot.Config;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SwerveModule extends SubsystemBase {

    private int m_moduleIndex;

    private CANSparkMax m_driveMotor;
    private CANPIDController m_drivePIDController; 
    private CANEncoder m_driveEncoder;

    private CANSparkMax m_turningMotor;
    private CANEncoder m_turningEncoder;

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    //It is slower than CANPIDController. However, it has the feature of smooth angle inputs.
    private final ProfiledPIDController m_turningPIDController;
 
    //@todo: lamprey encoder
    //AnalogEncoder m_analogEncoder;

    private int m_driveEncoderCPR;
    private int m_turningEncoderCPR;

    private double m_turningEncoderAccumPos;

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
     * @param moduleIndex   ID for the drive motor.
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

        m_turningPIDController = new ProfiledPIDController(
            Config.moduleConstants[moduleIndex].turnProfiledPIDConstants.kP, 
            Config.moduleConstants[moduleIndex].turnProfiledPIDConstants.kI, 
            Config.moduleConstants[moduleIndex].turnProfiledPIDConstants.kD,
            new TrapezoidProfile.Constraints( Config.moduleConstants[moduleIndex].turnProfiledPIDConstants.kMaxAngularSpeedRadiansPerSecond ,
                                              Config.moduleConstants[moduleIndex].turnProfiledPIDConstants.kMaxAngularAccelerationRadiansPerSecond));

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI); //missing from SparkMax
        m_turningPIDController.reset(0.0);

        m_turningEncoder = m_turningMotor.getEncoder();
        m_turningEncoderCPR = m_turningEncoder.getCountsPerRevolution();
        m_turningEncoder.setPositionConversionFactor(Config.moduleConstants[moduleIndex].encoderConstants.kPositionConversionFactor );
        m_turningEncoderAccumPos = 0;

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
                / (double) m_driveEncoderCPR)
                / Config.CANDEncoderConstants.SECOND_PER_MINUTE;
     }

    public Rotation2d getModuleCurrentAngle()
    {
        //m_turningEncoder.getPosition(); //native units of rotation
        return (new Rotation2d(m_turningEncoder.getPosition() *  (2 * Math.PI) / (double) m_turningEncoderCPR));
    }

    public double convertSpeedToRPM( double speedMetersPerSecond )
    {
        return (speedMetersPerSecond * (double) m_driveEncoderCPR
                * Config.CANDEncoderConstants.SECOND_PER_MINUTE
                / (Config.moduleConstants[m_moduleIndex].encoderConstants.kWheelDiameterMeters * Math.PI) );
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

        // Calculate the turning motor output from the turning PID controller.
        final var turnOutput = m_turningPIDController.calculate(getModuleCurrentAngle().getRadians(),
                                                                state.angle.getRadians());

        // Calculate the turning motor output from the turning PID controller.
        m_turningMotor.set(turnOutput);       

        //@todo: send to the network table for debugging
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

    @Override
    public void periodic() {
        //track the turning position here?
        m_turningEncoderAccumPos = m_turningEncoder.getPosition();

        //debugging


    }
}
