// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// v1.1 test cmd

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;


/** An example command that uses an example subsystem. */
public class EncoderCalibrator extends CommandBase {
  
  private final ArmSubsystem armSubsystem;

 
  /**
   * @param robotArmSubsystem
   * @param d
   */
  public EncoderCalibrator (ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
  


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);


  }


private void addRequirements(ArmSubsystem armSubsystem) {
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
  }

  @Override
  public void execute() {
    armSubsystem.encoderCalibrator();
  }


  


  @Override
  public void end(boolean interrupted) {

    armSubsystem.stopHorizMotors();

  }
  
  @Override
  public boolean isFinished() {

    return true;

  }
}
