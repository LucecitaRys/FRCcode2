// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.Intake;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Autonomos extends Command {
   private final ElevatorSub mElevador = ElevatorSub.getInstance(); 
  private final Intake mIntake = Intake.getInstance();
 
 

 
 private ElevatorSub.ElePoses MyposeEl= null;
 private boolean mFlag;

  
  public Autonomos(ElevatorSub.ElePoses poseEle) {
  

    MyposeEl = poseEle;
    addRequirements(mIntake);
addRequirements(mElevador);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

   
    mElevador.ElPos = MyposeEl;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
      if ( mElevador.posel == mElevador.getPosEle()) {

        mIntake.ConstanVel(0);

        if (mIntake.Current() >= 0.32) {
          mIntake.ConstanVel(0);
          mFlag = true;
        }
      }

    




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mFlag;
  }
}
