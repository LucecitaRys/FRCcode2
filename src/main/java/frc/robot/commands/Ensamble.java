// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlBoard;
import frc.robot.subsystems.Algaes;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Ensamble extends Command {

  private ElevatorSub mElevator = ElevatorSub.getInstance();
  private Intake mIntake = Intake.getInstance();
  //private Algaes mAlgaes = Algaes.getInstance();
  double refpos=0;
  double PosMu = 0;
  double posA= 0;
boolean salida;

  public Ensamble( ) {
  
    addRequirements(mElevator);
    addRequirements(mIntake);
   // addRequirements(mAlgaes);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
     mElevator.GETMANUALPOS(ControlBoard.getRightY_ope());
     mIntake.MANUALPOSE(ControlBoard.getLeftY_ope());
    // if(ControlBoard.coll()){
     // mIntake.ConstanVel(0.1);
    // }
     if(ControlBoard.COLLECT()){ //a
      refpos=0.0;
      PosMu=4.15;
     }
     if(ControlBoard.Nivel1()){ //b
      refpos=-3;
      PosMu= 2.4;// 0.9
      
     }
     if(ControlBoard.Nivel2()){ // x
      refpos=-17.5;
      PosMu= 2.1;
      
     }
     if(ControlBoard.Nivel3()){ // y //1
      refpos=0.0;
      PosMu=1.9;
     }
     //if (ControlBoard.algae1()){
      //posA=-1;
      
     //}
    /* else{
      posA=0;
     }
     mAlgaes.setVelocityAlgae(ControlBoard.velAl());
*/ 
if(ControlBoard.Continue()){
  mIntake.ConstanVel(-2.5);
}
if(ControlBoard.Stop()){
  mIntake.ConstanVel(0.0); 
}
if (ControlBoard.ButtonCollect()) {
  mIntake.SetPosM(0);
}
if (ControlBoard.Buttonthrow()) {
  mIntake.SetPosM(4.15);
}

    mElevator.setPosElevator(refpos);
    mIntake.SetPosM(PosMu);
    //mAlgaes.PosMAl(posA);
  mIntake.ConstanVel(ControlBoard.ButtonControVelIntake()*0.25+ 0.05);
 
  }
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished( ) {
    return salida;
  }
}
