// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// v1.1 test cmd

package frc.robot.commands;

// import frc.robot.subsystems.RobotArmSubsystem;
import frc.robot.subsystems.RobotGrabberSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class GrabberToggleCmd extends CommandBase {
  private RobotGrabberSubsystem robotGrabberSubsystem;
  
  
  /**
   * @param robotArmSubsystem
   * @param d
   * @return 
   */
  public GrabberToggleCmd (RobotGrabberSubsystem robotGrabberSubsystem) {
    this.robotGrabberSubsystem = robotGrabberSubsystem;
   
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotGrabberSubsystem);


  }


private void addRequirements(RobotGrabberSubsystem robotGrabberSubsystem2) {
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotGrabberSubsystem.toggleGrabber();
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  
}
