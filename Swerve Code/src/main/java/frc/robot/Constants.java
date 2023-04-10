// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// test command

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static class ModuleConstants {
      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
      public static final double kDriveMotorGearRatio = 1 / 8.14; 
      public static final double kTurningMotorGearRatio = 1 / 21.4285714286; 
      public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
      public static final double kPTurning_P = 0.5; // Porportional term of the PID Controller for the turning motor
      public static final double kPTurning_D = 0.1;
    }

    public static class DriveConstants {
      public static final double kTrackWidth = Units.inchesToMeters(23); // Distance between right and left wheels 
      public static final double kWheelBase = Units.inchesToMeters(23); // Distance between the front and back wheels
      
      // Locations of each Swerve Module on the Robot
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), // frontLeft
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // frontRight
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // backLeft
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // backRight

                // redo these ids at some point but it works like this
      public static final int kFrontLeftDriveMotorPort = 7; 
      public static final int kFrontRightDriveMotorPort = 4; 
      public static final int kBackLeftDriveMotorPort = 10; 
      public static final int kBackRightDriveMotorPort = 1; 

      public static final int kFrontLeftTurningMotorPort = 9; 
      public static final int kFrontRightTurningMotorPort = 6; 
      public static final int kBackLeftTurningMotorPort = 12; 
      public static final int kBackRightTurningMotorPort = 3; 

      public static final int kFrontLeftDriveAbsoluteEncoderPort = 8;  
      public static final int kFrontRightDriveAbsoluteEncoderPort = 5; 
      public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
      public static final int kBackRightDriveAbsoluteEncoderPort = 2; 

      // public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true; // Determine our values (neg = reversed)
      // public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true; // Determine our values (neg = reversed)
      // public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true; // Determine our values (neg = reversed)
      // public static final boolean kBackRightDriveAbsoluteEncoderReversed = true; // Determine our values (neg = reversed)

      public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(167.87); // backRight (its weird but it works)
      public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(103.00); // backLeft
      public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(120.67); // frontRight
      public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(80.59); // frontLeft

      // Max Speeds adn Accels
      public static final double kPhysicalMaxSpeedMetersPerSecond = 8; // Determine our values/Guess
      public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // Determine our values/Guess
      
      public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
      public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
              
      public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2; // Determine our values/Guess
      public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2; // Determine our values/Guess

      public static final double kBalP = 0.015;
      public static final double kBalI = 0.001;
      public static final double kBalD = 0.0035;

      public static final double kBalDegTol = 5;
      public static final double kBalDegPerSecTol = 5;

    }

    public static class JoystickConstants {
      public static final int rightStickPort = 0;
      public static final int leftStickPort = 1;

      public static final int kDriverXAxis = 0;
      public static final int kDriverYAxis = 1;
      public static final int kDriverRotAxis = 2;
      public static final int kDriverFieldOrientedButtonIdx = 5;
      public static final int kDriverZeroButton = 10;

      public static final int kArmYAxis = 1;
      
      // Find whatever button works the best

      public static final double kDeadband = 0.12;
    }

    public static final class AutoConstants {
      public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
      public static final double kMaxAngularSpeedRadiansPerSecond = 
              DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
      public static final double kPXController = 1.5;
      public static final double kPYController = 1.5;
      public static final double kPThetaController = 3;

      public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
              new TrapezoidProfile.Constraints(
                      kMaxAngularSpeedRadiansPerSecond,
                      kMaxAngularAccelerationRadiansPerSecondSquared);
  }



  }
}