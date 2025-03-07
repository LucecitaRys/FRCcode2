// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.auto.modes;



import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.IAuto;
import frc.robot.commands.Autonomos;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSub.ElePoses;
import frc.robot.subsystems.Shooter.intake_states;




public class INFERIOR implements IAuto {
    private final Pose2d mStartingPose; 
    private final Command mAutoCommand; 
 PathPlannerPath path1, path2, path3;
 CommandSwerveDrivetrain mDrivetrain;
    public INFERIOR () {
        try {
     path1 = PathPlannerPath.fromPathFile("Centr");
     path2 = PathPlannerPath.fromPathFile("Izquierdo");
   path3 = PathPlannerPath.fromPathFile("Derecha");
        } catch (Exception e) {
            e.printStackTrace();
        }
        mStartingPose = new Pose2d(9.5745 ,0.0, Rotation2d.fromDegrees(0)); 
    
        mAutoCommand = 
         new SequentialCommandGroup(
                    AutoBuilder.followPath(path2),
                    AutoBuilder.followPath(path1),
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
