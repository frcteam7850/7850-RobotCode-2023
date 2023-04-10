// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS; 
import edu.wpi.first.wpilibj.SerialPort;
import com.team2052.swervemodule.ModuleConfiguration;
import com.team2052.swervemodule.NeoSwerverModule;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  private final NeoSwerverModule frontLeftModule;
  private final NeoSwerverModule frontRightModule;
  private final NeoSwerverModule backLeftModule;
  private final NeoSwerverModule backRightModule;

  private final AHRS gyro;

  private final SwerveDriveOdometry odometry;

  /** Creates a new SwerveDrivetrainSubsystem. */
  public SwerveSubsystem() {
      frontLeftModule = new NeoSwerverModule(
          "front left",
          ModuleConfiguration.MK4I_L1,
          Constants.OperatorConstants.DriveConstants.kFrontLeftDriveMotorPort,
          Constants.OperatorConstants.DriveConstants.kFrontLeftTurningMotorPort,
          Constants.OperatorConstants.DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
          new Rotation2d(Constants.OperatorConstants.DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad)
      );
      frontRightModule = new NeoSwerverModule(
          "front right",
          ModuleConfiguration.MK4I_L1,
          Constants.OperatorConstants.DriveConstants.kFrontRightDriveMotorPort,
          Constants.OperatorConstants.DriveConstants.kFrontRightTurningMotorPort,
          Constants.OperatorConstants.DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
          new Rotation2d(Constants.OperatorConstants.DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad)
      );
      backLeftModule = new NeoSwerverModule(
          "back left",
          ModuleConfiguration.MK4I_L1,
          Constants.OperatorConstants.DriveConstants.kBackLeftDriveMotorPort,
          Constants.OperatorConstants.DriveConstants.kBackLeftTurningMotorPort,
          Constants.OperatorConstants.DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
          new Rotation2d(Constants.OperatorConstants.DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad)
      );
      backRightModule = new NeoSwerverModule(
          "back right",
          ModuleConfiguration.MK4I_L1,
          Constants.OperatorConstants.DriveConstants.kBackRightDriveMotorPort,
          Constants.OperatorConstants.DriveConstants.kBackRightTurningMotorPort,
          Constants.OperatorConstants.DriveConstants.kBackRightDriveAbsoluteEncoderPort,
          new Rotation2d(Constants.OperatorConstants.DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad)
      );

      gyro = new AHRS(SerialPort.Port.kUSB);
      gyro.reset();

      odometry = new SwerveDriveOdometry(
          Constants.OperatorConstants.DriveConstants.kDriveKinematics, 
          getRotation(),
          getModulePositions()
      );
  }

  @Override
  public void periodic() {

      
      SmartDashboard.putNumber("Robot Heading", gyro.getAngle());
      SmartDashboard.putString("Robot Location", getPosition().toString());
      SmartDashboard.putNumber("Gyro Roll", getGyroscopeRoll());

      debug();
  }

  /**
   * All parameters are taken in normalized terms of [-1.0 to 1.0].
   */
  public void drive(
      double normalizedXVelocity, 
      double normalizedYVelocity, 
      double normalizedRotationVelocity, 
      boolean fieldCentric
  ) {
      normalizedXVelocity = Math.copySign(
          Math.min(Math.abs(normalizedXVelocity), 1.0),
          normalizedXVelocity
      );
      normalizedYVelocity = Math.copySign(
          Math.min(Math.abs(normalizedYVelocity), 1.0),
          normalizedYVelocity
      );
      normalizedRotationVelocity = Math.copySign(
          Math.min(Math.abs(normalizedRotationVelocity), 1.0),
          normalizedRotationVelocity
      );

      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
          normalizedXVelocity * getMaxVelocityMetersPerSecond(), 
          normalizedYVelocity * getMaxVelocityMetersPerSecond(), 
          normalizedRotationVelocity * getMaxVelocityMetersPerSecond()
      );

      if (fieldCentric) {
          chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getRotation());
      }

      drive(chassisSpeeds);
  }

  /**
   * Autonomous commands still require a drive method controlled via a ChassisSpeeds object
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
      SwerveModuleState[] swerveModuleStates = Constants.OperatorConstants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      setModuleStates(swerveModuleStates);
  }

  public void stop() {
      drive(0, 0, 0, false);
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
      // Check if the wheels don't have a drive velocity to maintain the current wheel orientation.
      boolean hasVelocity = swerveModuleStates[0].speedMetersPerSecond != 0
          || swerveModuleStates[1].speedMetersPerSecond != 0 
          || swerveModuleStates[2].speedMetersPerSecond != 0
          || swerveModuleStates[3].speedMetersPerSecond != 0;

      frontLeftModule.setState(
          swerveModuleStates[0].speedMetersPerSecond, 
          hasVelocity ? swerveModuleStates[0].angle : frontLeftModule.getState().angle
      );
      frontRightModule.setState(
          swerveModuleStates[1].speedMetersPerSecond, 
          hasVelocity ? swerveModuleStates[1].angle : frontRightModule.getState().angle
      );
      backLeftModule.setState(
          swerveModuleStates[2].speedMetersPerSecond, 
          hasVelocity ? swerveModuleStates[2].angle : backLeftModule.getState().angle
      );
      backRightModule.setState(
          swerveModuleStates[3].speedMetersPerSecond, 
          hasVelocity ? swerveModuleStates[3].angle : backRightModule.getState().angle
      );
  }

  private SwerveModulePosition[] getModulePositions() {
      return new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          backLeftModule.getPosition(),
          backRightModule.getPosition()
      };
  }

  public void resetOdometry(Pose2d initialStartingPose) {
      odometry.resetPosition(getRotation(), getModulePositions(), initialStartingPose);
  }

  public void zeroGyro() {
      gyro.reset();
      //gyro.setAngleAdjustment(180);
  }

  public Pose2d getPosition() {
      return odometry.getPoseMeters();
  }

  public Rotation2d getRotation() {
     return gyro.getRotation2d();
  }

  public double getGyroscopeRoll() {
    return gyro.getRoll();
  }

  public static double getMaxVelocityMetersPerSecond() {
      return NeoSwerverModule.getMaxVelocityMetersPerSecond(ModuleConfiguration.MK4I_L1);
  }

  /**
   * For initial set up of swerve modules call this method from the periodic method, adjust the wheels
   * to be at 90 angles, set all offsets to 0, and record the encoder values put in SmartDashboard. These encoder values will
   * be the offsets for each SwerveModule respectively.
   */
  public void debug() {
      frontLeftModule.debug();
      frontRightModule.debug();   
      backLeftModule.debug();
      backRightModule.debug();
  }

  public void pointInwards() {
    frontLeftModule.setState(0, Rotation2d.fromDegrees(-135));
    frontRightModule.setState(0, Rotation2d.fromDegrees(135));
    backLeftModule.setState(0, Rotation2d.fromDegrees(-45));
    backRightModule.setState(0, Rotation2d.fromDegrees(-45));
  }
}
