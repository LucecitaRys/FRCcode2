// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomos.Options;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomos.ExAuto;


public class AUTOBASE implements ExAuto{
    private final Pose2d mStartingPose; 
     private final Command mAutoCommand; 
    PathPlannerPath p1, p2, p3;
    PathPlannerTrajectory mp = new PathPlannerTrajectory(p1, null, null, null);

    public AUTOBASE () {
 try { p1 = PathPlannerPath.fromPathFile("Centr");
            p2 = PathPlannerPath.fromPathFile("Izquierdo");
            p3 = PathPlannerPath.fromPathFile("Derecha");
         } catch (Exception e) {
             e.printStackTrace();
         }
         mStartingPose = new Pose2d(9.5745 ,0.0, Rotation2d.fromDegrees(0)); 
     
        mAutoCommand  =  new SequentialCommandGroup(
                     AutoBuilder.followPath(p1),
                     AutoBuilder.followPath(p3)
          );
    }
    @Override 
    public Command getAutoCommand() {
        return mAutoCommand; 
    }
    @Override
    public Pose2d getStartingPose2d() {
        return mStartingPose; 
    }
}
