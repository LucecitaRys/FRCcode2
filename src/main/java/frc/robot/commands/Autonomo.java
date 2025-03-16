// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.commands;

import java.nio.file.Path;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;



public class Autonomo extends Command {
 

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
 public PathPlannerPath path1, path2, path3;
 
  private final Pose2d mStarPose = new Pose2d(9.5745 ,0.0, Rotation2d.fromDegrees(0));
  public boolean mFlag = false;
  
 

  public Autonomo() {
    // Use addRequirements() here to declare subsystem dependencies.
 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    try {
      path1 = PathPlannerPath.fromPathFile("Test");
      path2 = PathPlannerPath.fromPathFile("Test");
    path3 = PathPlannerPath.fromPathFile("Test");
         } catch (Exception e) {
             e.printStackTrace();
         }
 
 
  mFlag=true;

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return mFlag;
  }
}
*/