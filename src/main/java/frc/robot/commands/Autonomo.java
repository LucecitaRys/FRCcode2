// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Autonomo extends Command {
  /** Creates a new Autonomo.*/ 
  private Shooter mShooter = Shooter.getInstance();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
      private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
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
    
  if(time< 3.5 ){
    drivetrain.applyRequest(() ->
            drive.withVelocityX(0) // Drive forward with negative Y (forward)
               .withVelocityY(5) // Drive left with negative X (left) -
               .withRotationalRate(0.0)
               );
    
    mShooter.setConstantVel(0.0);
    SmartDashboard.putBoolean("Estado 1", true);
    SmartDashboard.putNumber("Time", time);
  }
   
   else if(time>=3.5 && time < 4){
    SmartDashboard.putBoolean("Estado 2", true);
      mShooter.setConstantVel(0.15);
      x= true;
      SmartDashboard.putNumber("Time", time);
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
