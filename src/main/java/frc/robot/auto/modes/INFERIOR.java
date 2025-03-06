// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
package frc.robot.auto.modes;

import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auto.IAuto;
import frc.robot.commands.Autonomos;
import frc.robot.commands.ModeAlgae;
import frc.robot.subsystems.Brazo;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Brazo.brazoposes;
import frc.robot.subsystems.ElevatorSub.ElePoses;
import frc.robot.subsystems.Shooter.intake_states;




public class INFERIOR implements IAuto {
    private final Pose2d mStartingPose; 
    private final Command mAutoCommand; 
    PathPlannerPath path1, path2, path3;
    
    public INFERIOR () {
        try {
            PathPlannerPath path1 = PathPlannerPath.fromPathFile("Centr");
            PathPlannerPath path2 = PathPlannerPath.fromPathFile("Izquierdo");
            PathPlannerPath path3 = PathPlannerPath.fromPathFile("Derecha");
        } catch (Exception e) {
            e.printStackTrace();
        }
        mStartingPose = new Pose2d(9.5745 ,0.0, Rotation2d.fromDegrees(0)); 
    
        mAutoCommand = 
         new SequentialCommandGroup(
                    AutoBuilder.followPath(path2),
                    AutoBuilder.followPath(path1),
                    new Autonomos(brazoposes.nivel4, intake_states.throwCoral, ElePoses.nivel4),
                    AutoBuilder.followPath(path2)
                 
                );
    }
    @Override 
    public Command getAutoCommand () {
        return mAutoCommand; 
    }
    @Override
    public Pose2d getStartingPose () {
        return mStartingPose; 
    }

}
*/