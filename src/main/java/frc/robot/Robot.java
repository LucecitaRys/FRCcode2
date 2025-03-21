// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Ensamble;
import frc.robot.subsystems.Algaes;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.Intake;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
   UsbCamera usbCamera = CameraServer.startAutomaticCapture();

  private final RobotContainer m_robotContainer;
  //private Algaes mAlgaes = Algaes.getInstance();
private Intake mIntake = Intake.getInstance();
private final Ensamble ensamble = new Ensamble();

private ElevatorSub mElevatorSub = ElevatorSub.getInstance();
  public Robot() {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
     
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    mIntake.setDefaultCommand(ensamble);
     mElevatorSub.setDefaultCommand(ensamble);
    // mAlgaes.setDefaultCommand(ensamble);
    
  }

  @Override
  public void teleopPeriodic() {
  
  }

  @Override
  public void teleopExit() {
   
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
