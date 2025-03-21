// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Throw extends Command {
  /** Creates a new throw. */
  private final Intake mIntake;
  private final ElevatorSub mElevator;
  double pos = 0;
  private double mStartime;
  boolean Flag = false;
  double vel;
  
  //private double mTime;
  public Throw(Intake mIntake, ElevatorSub mElevator) {
    this.mIntake = mIntake;
    this.mElevator = mElevator;
addRequirements(mIntake);
addRequirements(mElevator);


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mStartime = Timer.getFPGATimestamp(); 
    SmartDashboard.putBoolean("True", true);
      pos=2.6; 
  if (mStartime >= 0.5) {
    mIntake.ConstanVel(-0.25);
    mIntake.SetPosM(pos);     
  }
  if(mStartime >= 1){
    Flag = true;
  }
}
  // Called once the command ends or is interrupted.
@Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("True", false);
}

  // Returns true when the command should end.
@Override
public boolean isFinished() {
  return true;
}
}
