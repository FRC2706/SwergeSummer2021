// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystemSM extends SubsystemBase {
    // Robot swerve modules
    private final SwerveModule m_singleModule = new SwerveModule(1);

    private static DriveSubsystemSM instance;

    /** Creates a new DriveSubsystem. */
    private DriveSubsystemSM() {
    }

    public static DriveSubsystemSM getInstance() {
        if (instance == null) {
            instance = new DriveSubsystemSM();
        }
        return instance;
    }

    public void updateTurningEncodersFromLamprey() {
        m_singleModule.updateTurningEncoderFromLamprey();
    }

    public void setModuleState(SwerveModuleState desiredState) {
        m_singleModule.setDesiredState(desiredState);
    }

    public void stopMotors() {
        m_singleModule.stopMotors();
    }

    @Override
    public void periodic()
    {
        m_singleModule.updateLamprey();
    }
}
