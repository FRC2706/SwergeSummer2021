// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Config {

  public class DriveConstants {
    public int kDriveMotorChannel;
    public int kTurningMotorChannel;
   
    public boolean kDriveMotorInverted; 
    public boolean kTurningMotorInverted;

    //Note: encoder does not have inversion for the Brushless Mode.
    public Translation2d kTranslation2dKinematics;
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
    public double kVelocityConversionFactor; //for gears
    public double kPositionConversionFactor;

  }

  public class ModuleConstants {
    //driveConstants
    public DriveConstants driveConstants;
    //drivePID
    public CANPIDConstants driveCANPIDConstants;
    //turningPID
    public CANPIDConstants turnCANPIDConstants;
    //encoders
    public CANDEncoderConstants encoderConstants;
  }
  
  // Distance between centers of right and left wheels on robot
  public static final double kTrackWidth = 0.5;
  // Distance between front and back wheels on robot
  public static final double kWheelBase = 0.7;
  public static final boolean kGyroReversed = false;

  public static ModuleConstants[] moduleConstants;

  // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  // These characterization values MUST be determined either experimentally or theoretically
  // for *your* robot's drive.
  // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
  // values for your robot.
  public static final class AutoConstants {
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    //@todo: clean up
    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }  

  //@todo: clean up
  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
    public static final int PIGEON_ID = 27;  //<-- TBD
  }
  
  public Config()
  {
    moduleConstants = new ModuleConstants[4];
 
    //module Index 0 : /single module
    moduleConstants[0].driveConstants = new DriveConstants();
    moduleConstants[0].driveConstants.kDriveMotorChannel = 0;
    moduleConstants[0].driveConstants.kTurningMotorChannel = 1;
    moduleConstants[0].driveConstants.kDriveMotorInverted = false;
    moduleConstants[0].driveConstants.kTurningMotorInverted = false;
    moduleConstants[0].driveConstants.kTranslation2dKinematics = new Translation2d(kWheelBase / 2, kTrackWidth / 2);

    moduleConstants[0].driveCANPIDConstants = new CANPIDConstants();
    moduleConstants[0].driveCANPIDConstants.minPower = -1;
    moduleConstants[0].driveCANPIDConstants.maxPower = 1;
    moduleConstants[0].driveCANPIDConstants.kFF = 1.;
    moduleConstants[0].driveCANPIDConstants.kP = 1.;
    moduleConstants[0].driveCANPIDConstants.kI = 0.;
    moduleConstants[0].driveCANPIDConstants.kD = 0.;
    moduleConstants[0].driveCANPIDConstants.kIZone = 0;

    moduleConstants[0].turnCANPIDConstants = new CANPIDConstants();
    moduleConstants[0].turnCANPIDConstants.minPower = -1;
    moduleConstants[0].turnCANPIDConstants.maxPower = 1;
    moduleConstants[0].turnCANPIDConstants.kFF = 1.;
    moduleConstants[0].turnCANPIDConstants.kP = 1.;
    moduleConstants[0].turnCANPIDConstants.kI = 0.;
    moduleConstants[0].turnCANPIDConstants.kD = 0.;
    moduleConstants[0].turnCANPIDConstants.kIZone = 0;

    moduleConstants[0].encoderConstants = new CANDEncoderConstants();
    moduleConstants[0].encoderConstants.kWheelDiameterMeters = 0.15;
    moduleConstants[0].encoderConstants.kVelocityConversionFactor = 1.;
    moduleConstants[0].encoderConstants.kPositionConversionFactor = 1.;

    //module Index 1: 
    moduleConstants[1].driveConstants = new DriveConstants();
    moduleConstants[1].driveConstants.kDriveMotorChannel = 0;
    moduleConstants[1].driveConstants.kTurningMotorChannel = 1;
    moduleConstants[1].driveConstants.kDriveMotorInverted = false;
    moduleConstants[1].driveConstants.kTurningMotorInverted = false;
    moduleConstants[1].driveConstants.kTranslation2dKinematics = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);

    moduleConstants[1].driveCANPIDConstants = new CANPIDConstants();
    moduleConstants[1].driveCANPIDConstants.minPower = -1;
    moduleConstants[1].driveCANPIDConstants.maxPower = 1;
    moduleConstants[1].driveCANPIDConstants.kFF = 1.;
    moduleConstants[1].driveCANPIDConstants.kP = 1.;
    moduleConstants[1].driveCANPIDConstants.kI = 0.;
    moduleConstants[1].driveCANPIDConstants.kD = 0.;
    moduleConstants[1].driveCANPIDConstants.kIZone = 0;

    moduleConstants[1].turnCANPIDConstants = new CANPIDConstants();
    moduleConstants[1].turnCANPIDConstants.minPower = -1;
    moduleConstants[1].turnCANPIDConstants.maxPower = 1;
    moduleConstants[1].turnCANPIDConstants.kFF = 1.;
    moduleConstants[1].turnCANPIDConstants.kP = 1.;
    moduleConstants[1].turnCANPIDConstants.kI = 0.;
    moduleConstants[1].turnCANPIDConstants.kD = 0.;
    moduleConstants[1].turnCANPIDConstants.kIZone = 0;

    moduleConstants[1].encoderConstants = new CANDEncoderConstants();
    moduleConstants[1].encoderConstants.kWheelDiameterMeters = 0.15;
    moduleConstants[1].encoderConstants.kVelocityConversionFactor = 1.;
    moduleConstants[1].encoderConstants.kPositionConversionFactor = 1.;

    //module Index 2: 
    moduleConstants[2].driveConstants = new DriveConstants();
    moduleConstants[2].driveConstants.kDriveMotorChannel = 0;
    moduleConstants[2].driveConstants.kTurningMotorChannel = 1;
    moduleConstants[2].driveConstants.kDriveMotorInverted = false;
    moduleConstants[2].driveConstants.kTurningMotorInverted = false;
    moduleConstants[2].driveConstants.kTranslation2dKinematics = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);

    moduleConstants[2].driveCANPIDConstants = new CANPIDConstants();
    moduleConstants[2].driveCANPIDConstants.minPower = -1;
    moduleConstants[2].driveCANPIDConstants.maxPower = 1;
    moduleConstants[2].driveCANPIDConstants.kFF = 1.;
    moduleConstants[2].driveCANPIDConstants.kP = 1.;
    moduleConstants[2].driveCANPIDConstants.kI = 0.;
    moduleConstants[2].driveCANPIDConstants.kD = 0.;
    moduleConstants[2].driveCANPIDConstants.kIZone = 0;

    moduleConstants[2].turnCANPIDConstants = new CANPIDConstants();
    moduleConstants[2].turnCANPIDConstants.minPower = -1;
    moduleConstants[2].turnCANPIDConstants.maxPower = 1;
    moduleConstants[2].turnCANPIDConstants.kFF = 1.;
    moduleConstants[2].turnCANPIDConstants.kP = 1.;
    moduleConstants[2].turnCANPIDConstants.kI = 0.;
    moduleConstants[2].turnCANPIDConstants.kD = 0.;
    moduleConstants[2].turnCANPIDConstants.kIZone = 0;

    moduleConstants[2].encoderConstants = new CANDEncoderConstants();
    moduleConstants[2].encoderConstants.kWheelDiameterMeters = 0.15;
    moduleConstants[2].encoderConstants.kVelocityConversionFactor = 1.;
    moduleConstants[2].encoderConstants.kPositionConversionFactor = 1.;

    //module Index 3: 
    moduleConstants[3].driveConstants = new DriveConstants();
    moduleConstants[3].driveConstants.kDriveMotorChannel = 0;
    moduleConstants[3].driveConstants.kTurningMotorChannel = 1;
    moduleConstants[3].driveConstants.kDriveMotorInverted = false;
    moduleConstants[3].driveConstants.kTurningMotorInverted = false;
    moduleConstants[3].driveConstants.kTranslation2dKinematics = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);

    moduleConstants[3].driveCANPIDConstants = new CANPIDConstants();
    moduleConstants[3].driveCANPIDConstants.minPower = -1;
    moduleConstants[3].driveCANPIDConstants.maxPower = 1;
    moduleConstants[3].driveCANPIDConstants.kFF = 1.;
    moduleConstants[3].driveCANPIDConstants.kP = 1.;
    moduleConstants[3].driveCANPIDConstants.kI = 0.;
    moduleConstants[3].driveCANPIDConstants.kD = 0.;
    moduleConstants[3].driveCANPIDConstants.kIZone = 0;

    moduleConstants[3].turnCANPIDConstants = new CANPIDConstants();
    moduleConstants[3].turnCANPIDConstants.minPower = -1;
    moduleConstants[3].turnCANPIDConstants.maxPower = 1;
    moduleConstants[3].turnCANPIDConstants.kFF = 1.;
    moduleConstants[3].turnCANPIDConstants.kP = 1.;
    moduleConstants[3].turnCANPIDConstants.kI = 0.;
    moduleConstants[3].turnCANPIDConstants.kD = 0.;
    moduleConstants[3].turnCANPIDConstants.kIZone = 0;

    moduleConstants[3].encoderConstants = new CANDEncoderConstants();
    moduleConstants[3].encoderConstants.kWheelDiameterMeters = 0.15;
    moduleConstants[3].encoderConstants.kVelocityConversionFactor = 1.;
    moduleConstants[3].encoderConstants.kPositionConversionFactor = 1.;
  }

  public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(  moduleConstants[0].driveConstants.kTranslation2dKinematics,
                                    moduleConstants[1].driveConstants.kTranslation2dKinematics,
                                    moduleConstants[2].driveConstants.kTranslation2dKinematics,
                                    moduleConstants[3].driveConstants.kTranslation2dKinematics);
  

}
