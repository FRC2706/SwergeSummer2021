// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Config;

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
  
    private AnalogPotentiometer m_lamprey;

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
     *      //1 is top left/single module
            //2 is bottom left
            //3 is bottom right
            //4 is top right
     */
    public SwerveModule( int moduleIndex ) {
        //module index
        m_moduleIndex = moduleIndex;

        //driver motor controller
        m_driveMotor = new CANSparkMax(Config.module(moduleIndex).driveConstants.kDriveMotorChannel, 
                                       MotorType.kBrushless);
        // Factory Default to prevent unexpected behaviour
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.setInverted(Config.module(moduleIndex).driveConstants.kDriveMotorInverted);

        m_drivePIDController = m_driveMotor.getPIDController();
        m_drivePIDController.setOutputRange( Config.module(moduleIndex).driveCANPIDConstants.minPower, 
                                             Config.module(moduleIndex).driveCANPIDConstants.maxPower);
        m_drivePIDController.setFF(Config.module(moduleIndex).driveCANPIDConstants.kFF);
        m_drivePIDController.setP(Config.module(moduleIndex).driveCANPIDConstants.kP);
        m_drivePIDController.setI(Config.module(moduleIndex).driveCANPIDConstants.kI);
        m_drivePIDController.setD(Config.module(moduleIndex).driveCANPIDConstants.kD);
        m_drivePIDController.setIZone(Config.module(moduleIndex).driveCANPIDConstants.kIZone);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoderCPR = m_driveEncoder.getCountsPerRevolution();
        m_driveEncoder.setVelocityConversionFactor(Config.module(moduleIndex).encoderConstants.kVelocityConversionFactor);

        //turning motor controller
        m_turningMotor = new CANSparkMax(Config.module(moduleIndex).driveConstants.kTurningMotorChannel,
                                         MotorType.kBrushless);
        m_turningMotor.restoreFactoryDefaults();
        m_turningMotor.setInverted(Config.module(moduleIndex).driveConstants.kTurningMotorInverted);

        m_turningPIDController = m_turningMotor.getPIDController();
        m_turningPIDController.setOutputRange( Config.module(moduleIndex).turnCANPIDConstants.minPower, 
                                             Config.module(moduleIndex).turnCANPIDConstants.maxPower);
        m_turningPIDController.setFF(Config.module(moduleIndex).turnCANPIDConstants.kFF);
        m_turningPIDController.setP(Config.module(moduleIndex).turnCANPIDConstants.kP);
        m_turningPIDController.setI(Config.module(moduleIndex).turnCANPIDConstants.kI);
        m_turningPIDController.setD(Config.module(moduleIndex).turnCANPIDConstants.kD);
        m_turningPIDController.setIZone(Config.module(moduleIndex).turnCANPIDConstants.kIZone);

        m_turningEncoder = m_turningMotor.getEncoder();
        m_turningEncoderCPR = m_turningEncoder.getCountsPerRevolution();
        m_turningEncoder.setPositionConversionFactor(Config.module(moduleIndex).encoderConstants.kPositionConversionFactor );

        // Lamprey encoder
        m_lamprey = new AnalogPotentiometer(Config.module(moduleIndex).driveConstants.kLampreyChannel, 2*Math.PI);
        
        // Check if real match. (ie. robot is competing on a field being controlled by a Field Management System)
        // If not a real match, enabled lamprey tuning through NetworkTables
        // Calls updateTurningEncoderFromLamprey when networktable entry is updated.
        if (DriverStation.getInstance().isFMSAttached() == false) {
            Config.module(m_moduleIndex).driveConstants.kLampreyOffset.registerListener((oldValue, newValue) -> updateTurningEncoderFromLamprey());
        }

        // network table entries
        String tableName = "Swerve Chassis/SwerveModule " + m_moduleIndex;
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

    public void updateTurningEncoderFromLamprey() {
        // Get offset value from networktables
        double offset = Config.module(m_moduleIndex).driveConstants.kLampreyOffset.get();
        double lampreyRadians = m_lamprey.get();
    
        // Give the value from Lamprey to the SparkMax
        m_turningEncoder.setPosition(convertAngleToPos(lampreyRadians + offset));
    }

    //only for the single module
    public double getCurrentDriveDistance()
    {
        return (m_driveEncoder.getPosition() * (Config.moduleConstants[m_moduleIndex].encoderConstants.kWheelDiameterMeters * Math.PI));

    }
}