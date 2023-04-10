package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class ArmInCmd extends CommandBase {
  
  private final ArmSubsystem armSubsystem;

 
  /**
   * @param robotArmSubsystem
   * @param d
   */
  public ArmInCmd (ArmSubsystem armSubsystem) {
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
    armSubsystem.testRetraction();
  }


  


  @Override
  public void end(boolean interrupted) {

    armSubsystem.stopHorizMotors();

  }
  
  @Override
  public boolean isFinished() {

    return false;

  }
}
