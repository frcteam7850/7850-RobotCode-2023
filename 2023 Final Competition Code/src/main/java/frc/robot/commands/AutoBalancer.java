package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class AutoBalancer extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController balanceController;
    private double targetSpeedMetersPerSecond;
    private double timeBalanced;
    private double targetTime = 50;

    public AutoBalancer(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        balanceController = new PIDController(DriveConstants.kBalP, DriveConstants.kBalI, DriveConstants.kBalD);
        balanceController.setTolerance(DriveConstants.kBalDegTol, DriveConstants.kBalDegPerSecTol);
        
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        targetSpeedMetersPerSecond = balanceController.calculate(swerveSubsystem.getGyroscopeRoll() , 0) * -DriveConstants.kPhysicalMaxSpeedMetersPerSecond/4;
        
        //SmartDashboard.putNumber("Gyro Pitch", swerveSubsystem.getGyroscopePitch());
        SmartDashboard.putBoolean("IsAtTarget", balanceController.atSetpoint());
        
        if (balanceController.atSetpoint()) timeBalanced++; else timeBalanced = 0;
        
        swerveSubsystem.drive(new ChassisSpeeds(
            -targetSpeedMetersPerSecond, 0,
                 0
            ));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }
    
    public boolean isFinished() {
        return (timeBalanced >= targetTime);
    }
}