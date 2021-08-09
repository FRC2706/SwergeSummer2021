// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystemSM;

public class ModuleAngleFromJoystick extends CommandBase {

    private final Supplier<Double> xAxis;
    private final Supplier<Double> yAxis;

    /** Creates a new AngleTest. */
    public ModuleAngleFromJoystick(Supplier<Double> xAxis, Supplier<Double> yAxis) {

        this.xAxis = xAxis;
        this.yAxis = yAxis;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Both axis of a single stick on a joystick
        double x = xAxis.get();
        double y = -yAxis.get();
        
        if (Math.abs(x) < 0.6 && Math.abs(y) < 0.6) {
            // Do nothing
        } else {
            Rotation2d angle = new Rotation2d(x, y);

            SwerveModuleState state = new SwerveModuleState(0, angle);

            DriveSubsystemSM.getInstance().setModuleState(state);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        DriveSubsystemSM.getInstance().stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
