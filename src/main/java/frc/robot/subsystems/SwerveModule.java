// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import frc.robot.Config.ModuleConstants;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule {

    private CANSparkMax m_driveMotor;
    private CANPIDController m_drivePIDController;
    private CANEncoder m_driveEncoder;

    private CANSparkMax m_turningMotor;
    private CANEncoder m_turningEncoder;

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            ModuleConstants.kPModuleTurningController, 0, 0,
            new TrapezoidProfile.Constraints(ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                    ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel   ID for the drive motor.
     * @param turningMotorChannel ID for the turning motor.
     */
    public SwerveModule(int driveMotorChannel, int turningMotorChannel) {

        /*
        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

        // Set whether drive encoder should be reversed or not
        m_driveEncoder.setReverseDirection(driveEncoderReversed);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * wpi::math::pi)
        // divided by the encoder resolution.
        m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

        // Set whether turning encoder should be reversed or not
        m_turningEncoder.setReverseDirection(turningEncoderReversed);
              
        */

        //=======================

        //driver motor controller
        m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);

        // Factory Default to prevent unexpected behaviour
        m_driveMotor.restoreFactoryDefaults();

        //???
        m_driveMotor.setInverted(true);

        m_drivePIDController = m_driveMotor.getPIDController();
        m_drivePIDController.setOutputRange(-1, 1);
        m_drivePIDController.setFF(1.0);
        m_drivePIDController.setP(1.0);
        m_drivePIDController.setI(0);
        m_drivePIDController.setD(0);

        m_driveEncoder = m_driveMotor.getEncoder();

        //tuning motor controller
        m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
        m_turningMotor.restoreFactoryDefaults();
        //????
        m_turningMotor.setInverted(true);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_turningPIDController.reset(0.0);

        m_turningEncoder = m_turningMotor.getEncoder();
          
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        //m_driveEncoder.getVelocity();  //--> unit RPM --> m/s
        //m_turningEncoder.getPosition(); //native units of rotation
        return new SwerveModuleState(m_driveEncoder.getVelocity() * ModuleConstants.kDriveEncoderDistancePerPulse/60.,
                                     new Rotation2d(m_turningEncoder.getPosition() * ModuleConstants.kTurningEncoderDistancePerPulse));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, 
                                  new Rotation2d(m_turningEncoder.getPosition() * ModuleConstants.kTurningEncoderDistancePerPulse));


        //m_driveMotor.set(0.0);
        m_drivePIDController.setReference(state.speedMetersPerSecond/ModuleConstants.kDriveEncoderDistancePerPulse * 60., 
                                          ControlType.kVelocity);


        // Calculate the turning motor output from the turning PID controller.
        final var turnOutput = m_turningPIDController.calculate(m_turningEncoder.getPosition() * ModuleConstants.kTurningEncoderDistancePerPulse, 
                               state.angle.getRadians());

        // Calculate the turning motor output from the turning PID controller.
        m_turningMotor.set(turnOutput);       
       
        //m_turningPIDController.setReference(state.angle.getRadians(), ControlType.kPosition);
    }

    /** Zeros all the SwerveModule encoders. */
    public void resetEncoders() {
        m_driveEncoder.setPosition(0);
        m_turningEncoder.setPosition(0);
    }
}
