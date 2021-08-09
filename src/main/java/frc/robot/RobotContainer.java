// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import frc.robot.config.Config.*;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystemSM;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.commands.SingleModuleOnce;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The driver's controller
    private Joystick driverStick;
    private Joystick controlStick;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        driverStick = new Joystick(OIConstants.kDriverControllerPort);
        controlStick = new Joystick(OIConstants.kOperatorControllerPort);

        // Drive command, A split-stick arcade command, with forward/backward controlled
        //      by the left hand, and turning controlled by the right.
        /** NOT FOR SINGLE MODULE
        Command driveCommand = new RunCommand(() -> DriveSubsystem.getInstance().drive(
            driverStick.getY(GenericHID.Hand.kLeft),
            driverStick.getX(GenericHID.Hand.kRight), 
            driverStick.getX(GenericHID.Hand.kLeft), false));
        DriveSubsystem.getInstance().setDefaultCommand(driveCommand); 
        */ 

        // Instantiate the command and bind it
        Command singleModuleOnce0 = new SingleModuleOnce(0, Rotation2d.fromDegrees(0));
        new JoystickButton(driverStick, XboxController.Button.kBumperLeft.value).whenHeld(singleModuleOnce0);

        Command singleModuleOnce1 = new SingleModuleOnce(0, Rotation2d.fromDegrees(90));
        new JoystickButton(driverStick, XboxController.Button.kBumperRight.value).whenHeld(singleModuleOnce1);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() { 
        return new InstantCommand(DriveSubsystemSM.getInstance()::stopMotors);
        
        /** NOT FOR SINGLE MODULE

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Config.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)), config);

        var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                Config.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0), thetaController, m_robotDrive::setModuleStates,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));


        */
    }
}
