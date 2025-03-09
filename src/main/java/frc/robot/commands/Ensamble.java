// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlBoard;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Ensamble extends Command {
  /** Creates a new Ensamble. */
  private ElevatorSub mElevator = ElevatorSub.getInstance();
  private Intake mIntake = Intake.getInstance();
  double refpos=0;
  double PosMu = 0;

  public Ensamble() {
  
    addRequirements(mElevator);
    addRequirements(mIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntake.ConstanVel(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     mElevator.GETMANUALPOS(ControlBoard.getRightY_ope());
     mIntake.MANUALPOSE(ControlBoard.getLeftY_ope());
     if(ControlBoard.COLLECT()){
      refpos=0.0;
      PosMu=0.0;
      mIntake.ConstanVel(-0.5);
      if (mIntake.Current()>=20){
        mIntake.ConstanVel(0);
      }
     }
     if(ControlBoard.Nivel1()){
      refpos=0.0;
      PosMu=0.0;
      mIntake.ConstanVel(0.5);
      if (mIntake.Current()<=20){
        mIntake.ConstanVel(0);
      }
     }
     if(ControlBoard.Nivel2()){
      refpos=0.0;
      PosMu=0.0;
      mIntake.ConstanVel(0.5);
      if (mIntake.Current()<=20){
        mIntake.ConstanVel(0);
      }
     }
     if(ControlBoard.Nivel3()){
      refpos=0.0;
      PosMu=0.0;
      mIntake.ConstanVel(0.5);
      if (mIntake.Current()<=20){
        mIntake.ConstanVel(0);
      }
     }


     mElevator.setPosElevator(refpos);
     mIntake.SetPosM(PosMu);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
