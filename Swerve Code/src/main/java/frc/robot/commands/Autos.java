// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//package frc.robot.auto;
package frc.robot.commands;


import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OperatorConstants.DriveConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autos extends SequentialCommandGroup {
    public Autos(SwerveSubsystem swerveSubsystem) {
        // The rotations defined in these Pose2d's are the rotations of the wheels not the robot!
        Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        // Next poses
    
        //Translation2d Pose2 = new Translation2d(1, 0);
        //Translation2d Pose3 = new Translation2d(1, 2);

        ArrayList<Translation2d> list = new ArrayList<Translation2d>();

        //list.add(Pose2);
        //list.add(Pose3);

        // End Pose
        Pose2d endPose = new Pose2d(2.75, 0, Rotation2d.fromDegrees(0));

        // Set your drivetrains odometry to the starting pose (if the starting pose is 0, 0, 0 you can skip this)
        swerveSubsystem.resetOdometry(startPose);

        SwerveControllerCommand swerveCommand = new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                /*
                 * ALWAYS MAKE SURE YOUR START POSE IS THE SAME AS YOUR LAST END POSE!
                 */
                // Start of path
                startPose,
                // Mid points of path
                list,
                // End of path
                endPose,
                // Max velocity and acceleration of path
                new TrajectoryConfig(2, 1).setKinematics(DriveConstants.kDriveKinematics)
            ),
            swerveSubsystem::getPosition,
            DriveConstants.kDriveKinematics,
            // Translational PID controllers (x, y)
            new PIDController(-0.25, -0.05, -0.1),
            new PIDController(0.25, 0.05, 0.1),
            // Rotational PID controller
            new ProfiledPIDController(
                1, 0.05, 0.1,
                new TrapezoidProfile.Constraints(
                    2 * Math.PI, 
                    2 * Math.PI
                )
            ),
            // Rotation of the chassis of the robot during the path
            () -> Rotation2d.fromDegrees(0),
            swerveSubsystem::setModuleStates,
            swerveSubsystem
        );

        

        addCommands(swerveCommand//,new AutoBalancer(swerveSubsystem)//new InstantCommand(swerveSubsystem::pointInwards, swerveSubsystem
        );
    }
}