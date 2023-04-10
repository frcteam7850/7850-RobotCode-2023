package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class ArmAuto extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private double timeMoving = 0;
    private double targetTime = 40;

    public ArmAuto(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;

        
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        if (timeMoving < targetTime) timeMoving++;
        
        armSubsystem.setVertMotors(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopVertMotors();
    }
    
    public boolean isFinished() {
        return (timeMoving >= targetTime);
    }
}