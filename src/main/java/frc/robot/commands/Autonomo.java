// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Autonomo extends Command {
  /** Creates a new Autonomo.*/ 
  private Shooter mShooter = Shooter.getInstance();
  
  public boolean x;
  private double time;
  public Autonomo() {
   
  addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Double.NaN;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time= Timer.getFPGATimestamp();
    
  if(time> 3.5 ){
   
    
    mShooter.setConstantVel(0.15);}
   else if(time< 4){
      mShooter.setConstantVel(0);
      x= true;
    }
    
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return x;
  }
}
