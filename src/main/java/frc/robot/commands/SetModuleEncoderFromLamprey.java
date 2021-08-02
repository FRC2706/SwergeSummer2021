// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystemSM;

public class SetModuleEncoderFromLamprey extends CommandBase {
    /** Creates a new SetModuleEncoderFromLamprey. */
    public SetModuleEncoderFromLamprey() {

        // No addRequirements() since all this is doing is reading the lamprey data and setting a new encoder value on the SparkMax
        // Hopefully this can be used while moving (likely not high speeds), so unscheduling a drive command because this command has the
        //      requirements would not be good.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        DriveSubsystemSM.getInstance().updateTurningEncodersFromLamprey();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
