// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Config {

    public class DriveConstants {
        public int kDriveMotorChannel;
        public int kTurningMotorChannel;

        public boolean kDriveMotorInverted;
        public boolean kTurningMotorInverted;

        // Note: encoder does not have inversion for the Brushless Mode.
        public Translation2d kTranslation2dKinematics;

        public int kLampreyChannel;
        public FluidConstant<Double> kLampreyOffset;
    }

    public class CANPIDConstants {
        public double minPower;
        public double maxPower;
        public double kFF;
        public double kP;
        public double kI;
        public double kD;
        public double kIZone;
    }

    public class CANDEncoderConstants {
        public static final int SECOND_PER_MINUTE = 60;
        public double kWheelDiameterMeters;
        public double kVelocityConversionFactor; // for gears
        public double kPositionConversionFactor;

    }

    public class ModuleConstants {
        // driveConstants
        public DriveConstants driveConstants;
        // drivePID
        public CANPIDConstants driveCANPIDConstants;
        // turningPID
        public CANPIDConstants turnCANPIDConstants;
        // encoders
        public CANDEncoderConstants encoderConstants;

        private int m_moduleIndex;

        public ModuleConstants(int moduleIndex) {
            m_moduleIndex = moduleIndex;

            driveConstants = new DriveConstants();
            driveConstants.kDriveMotorChannel = moduleSpecific(5, 3, 5, 7);
            driveConstants.kTurningMotorChannel = moduleSpecific(6, 4, 6, 8);
            //driveConstants.kDriveMotorChannel = moduleSpecific(6, 3, 5, 7);
            //driveConstants.kTurningMotorChannel = moduleSpecific(5, 4, 6, 8);
            driveConstants.kDriveMotorInverted = false; // All motors should spin in same direction if wired the same.
                                                        // Positive as counter-clockwise.
            driveConstants.kTurningMotorInverted = false; // If a motor doesn't spin the right way, swap 2 of the 3
                                                          // wires going to the brushless motor.
            driveConstants.kTranslation2dKinematics = moduleSpecific(new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                    new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                    new Translation2d(kWheelBase / 2, -kTrackWidth / 2));

            driveConstants.kLampreyChannel = moduleSpecific(0, 1, 2, 3); // 0-3 analog ports on roborio

            String tableName = "Swerve Chassis/SwerveModule " + m_moduleIndex;
            double lampreyOffset = moduleSpecific(0.0, 0.0, 0.0, 0.0);
            driveConstants.kLampreyOffset = new FluidConstant<>("Lamprey Offset Radians", lampreyOffset, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable(tableName));
            

            //For drive
            driveCANPIDConstants = new CANPIDConstants();
            driveCANPIDConstants.minPower = -1;
            driveCANPIDConstants.maxPower = 1;
            //driveCANPIDConstants.kFF = 0.00027; // These can also be module specific.
            //driveCANPIDConstants.kP = 0.002; // Hopefully they won't need to be.
            //driveCANPIDConstants.kI = 0.; // Depends on hardware differences.
            //driveCANPIDConstants.kD = 0.0002;
            driveCANPIDConstants.kIZone = 0;
            driveCANPIDConstants.kFF = 0.0013; // These can also be module specific.
            driveCANPIDConstants.kP = 0.000077; // Hopefully they won't need to be.
            driveCANPIDConstants.kI = 0.; // Depends on hardware differences.
            driveCANPIDConstants.kD = 0;

            //For rotation
            turnCANPIDConstants = new CANPIDConstants();
            turnCANPIDConstants.minPower = -1;
            turnCANPIDConstants.maxPower = 1;
            turnCANPIDConstants.kFF = 0;
            turnCANPIDConstants.kP = 1.2;
            //turnCANPIDConstants.kI = 0.0003;
            //turnCANPIDConstants.kD = 0.0005;
            turnCANPIDConstants.kI = 0.001;
            turnCANPIDConstants.kD = 5;
            turnCANPIDConstants.kIZone = 0.18; //5 degrees

            encoderConstants = new CANDEncoderConstants();
            encoderConstants.kWheelDiameterMeters = 0.1016; // 4in wheels
            encoderConstants.kVelocityConversionFactor = 1.0 / 7.615; // Wheel spins once = encoder spins 7.615 times
            encoderConstants.kPositionConversionFactor = 1.0 / 8; // Wheel spins once = encoder spins 8 times
        }

        /**
         * Returns one of the values passed based on the robot ID
         *
         * @param first The first value (default value)
         * @param more  Other values that could be selected
         * @param <T>   The type of the value
         * @return The value selected based on the ID of the robot
         */
        @SafeVarargs
        private <T> T moduleSpecific(T first, T... more) {
            int index = m_moduleIndex - 1;
            if (index < 1 || index > more.length) {
                return first;
            } else {
                return more[index - 1];
            }
        }
    }

    // Distance between centers of right and left wheels on robot (y direction)
    public static final double kTrackWidth = 0.5;
    // Distance between front and back wheels on robot (x direction)
    public static final double kWheelBase = 0.7;
    public static final boolean kGyroReversed = false;

    public static final String odometryNTTable = "Swerve Chassis/Odometry";

    public static ModuleConstants[] moduleConstants;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for
    // obtaining these
    // values for your robot.
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        // @todo: clean up
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    // @todo: clean up
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int PIGEON_ID = 27; // <-- TBD
    }

    public static SwerveDriveKinematics kDriveKinematics;

    public Config() {
        moduleConstants = new ModuleConstants[4];

        // 1 is top left/single module
        // 2 is bottom left
        // 3 is bottom right
        // 4 is top right
        moduleConstants[0] = new ModuleConstants(1);
        moduleConstants[1] = new ModuleConstants(2);
        moduleConstants[2] = new ModuleConstants(3);
        moduleConstants[3] = new ModuleConstants(4);

        SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            moduleConstants[0].driveConstants.kTranslation2dKinematics,
            moduleConstants[1].driveConstants.kTranslation2dKinematics,
            moduleConstants[2].driveConstants.kTranslation2dKinematics,
            moduleConstants[3].driveConstants.kTranslation2dKinematics);
    }


    // Valid module index is 1,2,3,4
    // 1 is top left
    // 2 is bottom left
    // 3 is bottom right
    // 4 is top right
    public static ModuleConstants module(int modIndex) {
        if (modIndex >= 1 && modIndex <= 4) {
            return moduleConstants[modIndex - 1];
        } else {
            System.out.println("Config module: Invalid module index");
            return null;
        }
    }

}