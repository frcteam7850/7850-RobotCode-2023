// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroHeadingCmd;

import frc.robot.commands.Autos;
import frc.robot.commands.Autos2;
import frc.robot.commands.Autos3;

import frc.robot.commands.AutoBalancer;


import frc.robot.Constants.OperatorConstants.DriveConstants;
import frc.robot.Constants.OperatorConstants.AutoConstants;
import frc.robot.Constants.OperatorConstants.JoystickConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick rightStick = new Joystick(JoystickConstants.rightStickPort);
  private final Joystick leftStick = new Joystick(JoystickConstants.leftStickPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    
      swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
              swerveSubsystem,
              () -> -rightStick.getRawAxis(JoystickConstants.kDriverYAxis),
              () -> -rightStick.getRawAxis(JoystickConstants.kDriverXAxis),
              () -> rightStick.getRawAxis(JoystickConstants.kDriverRotAxis),
              () -> !rightStick.getRawButton(JoystickConstants.kDriverFieldOrientedButtonIdx))); // Defaults to field reference




              putToDashboard();
    // Configure the trigger bindings
    configureButtonBindings();

  }


  private void configureButtonBindings() {



  //zero's robot heading
    new JoystickButton(rightStick, JoystickConstants.kDriverZeroButton).onTrue(new ZeroHeadingCmd(swerveSubsystem));


  // auto balancer
  new JoystickButton(rightStick, 3).onTrue(new AutoBalancer(swerveSubsystem));

  
  }




  private void putToDashboard() {
    autoChooser.addOption("No Auto", new InstantCommand(swerveSubsystem::stop));

    autoChooser.addOption("Middle Balance Low", new SequentialCommandGroup(
      new Autos(swerveSubsystem)));
      


    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
       swerveSubsystem.zeroGyro();
      
       return autoChooser.getSelected();
    }
 }