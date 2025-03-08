// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlBoard;

import frc.robot.subsystems.ElevatorSub;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PRUEBAS extends Command {
  /** Creates a new PRUEBAS. */
  private final ElevatorSub mElevador = ElevatorSub.getInstance(); 

  double refpos=0;

  public PRUEBAS() {
    // Use addRequirements() here to declare subsystem dependencies.
  
addRequirements(mElevador);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    mElevador.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
//joystic izquierdo brazo 
     mElevador.GETMANUALPOS(ControlBoard.getRightY_ope());// joystic derecho elevador
 
  // Coral operador
  if (ControlBoard.ButtonMuL()>0){

}
  

  if(ControlBoard.buttonA()){
    mElevador.reset();
  }
 if(ControlBoard.ButtonCOLLECT()){

 }  
 if(ControlBoard.ButtonCOLLECT()){
  
 }  
  
  
  else{
   
  }
  if (ControlBoard.buttonx()) {
    refpos = 0.5;
  }
  if (ControlBoard.buttony()) {
    refpos = 0.0;
  }
  mElevador.setPosElevator(refpos);

  // negativo muñeca trigger right
  SmartDashboard.putNumber("POSE ELEVATOR", mElevador.getPosEle());


  if(ControlBoard.buttonA()){
  }  
}
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
