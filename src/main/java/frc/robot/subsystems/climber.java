// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

 package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climber extends SubsystemBase {
  private static climber mClimber;
  private final TalonFX Climberm;
  private TalonFXConfiguration motorConfClimber = new TalonFXConfiguration();

  
   

  public climber() {

   // encoder = new Encoder(null, null);
Climberm= new TalonFX(Constants.MotorConstants.id_cl);


// Configuración de Slot (PID)
  Slot0Configs slot0Configs = new Slot0Configs(); 
slot0Configs.kP = 0;
slot0Configs.kI = 0;
slot0Configs.kD = 0;
slot0Configs.kS = 0;
slot0Configs.kV = 0;

// Configuración de Límites de Corriente
CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
currentLimitsConfigs.SupplyCurrentLowerLimit = 40;
currentLimitsConfigs.SupplyCurrentLowerTime = 1;
currentLimitsConfigs.SupplyCurrentLimit = 70;
currentLimitsConfigs.SupplyCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimit = 80;

FeedbackConfigs feedbackConfigs = new FeedbackConfigs(); 
  feedbackConfigs.FeedbackRemoteSensorID = 0;
  feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; 
  feedbackConfigs.FeedbackRotorOffset = 0;
  feedbackConfigs.RotorToSensorRatio = 0;
  feedbackConfigs.SensorToMechanismRatio = 0;

  // Configuración de Motion Magic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 15000; 
    motionMagicConfigs.MotionMagicAcceleration = 6000;    
    motionMagicConfigs.MotionMagicJerk = 0;   

    // Configuración de Límites de Posición
    SoftwareLimitSwitchConfigs softLimitSwitchConfigs = new SoftwareLimitSwitchConfigs(); 
    softLimitSwitchConfigs.ForwardSoftLimitThreshold = 10000; 
    softLimitSwitchConfigs.ReverseSoftLimitThreshold = -10000;
    //softLimitSwitchConfigs.ForwardSoftLimitEnable = true;
    //softLimitSwitchConfigs.ReverseSoftLimitEnable = true;
      
    // Configuración de Rampas
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.5;

    // Asignación de las subconfiguraciones a la configuración general
    motorConfClimber.Feedback = feedbackConfigs;
    motorConfClimber.Slot0 = slot0Configs;
    motorConfClimber.MotionMagic = motionMagicConfigs;
    motorConfClimber.CurrentLimits = currentLimitsConfigs;
    motorConfClimber.SoftwareLimitSwitch = softLimitSwitchConfigs;
    motorConfClimber.ClosedLoopRamps = closedLoopRampsConfigs;



  Climberm.getConfigurator().apply(motorConfClimber); 



    }
// metodo
public void setClimber(double PowerClim){
  Climberm.set(PowerClim);
} 

    public static climber getInstance (){
      if (mClimber== null){
        mClimber= new climber();
      }
      return mClimber;
    }

      }
