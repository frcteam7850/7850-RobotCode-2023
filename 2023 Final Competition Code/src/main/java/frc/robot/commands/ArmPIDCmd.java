package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.OperatorConstants.ArmConstants;

public class ArmPIDCmd extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final PIDController horizPID;
    private final PIDController vertPID;


    public ArmPIDCmd(ArmSubsystem armSubsystem, double vertSetpoint, double horizSetpoint) {
        this.armSubsystem = armSubsystem;
        this.horizPID = new PIDController(ArmConstants.hP, ArmConstants.hI, ArmConstants.hD);
        this.vertPID = new PIDController(ArmConstants.vP, ArmConstants.vI, ArmConstants.vD);

        vertPID.setSetpoint(vertSetpoint);
        horizPID.setSetpoint(horizSetpoint);

        vertPID.setTolerance(10, 1);
        horizPID.setTolerance(10, 1);

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        vertPID.reset();
        horizPID.reset();
    }

    @Override
    public void execute() {
        double vertSpeed = vertPID.calculate(armSubsystem.getVertEncoder());
        //double horizSpeed = horizPID.calculate(armSubsystem.getHorizEncoder());
        armSubsystem.setVertMotors(vertSpeed);
        //armSubsystem.setHorizMotors(horizSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        //robotArmSubsystem.stopVertMotors();
        //robotArmSubsystem.stopHorizMotors();
    }
        
  

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return (vertPID.atSetpoint() && horizPID.atSetpoint());

  }

}
    

