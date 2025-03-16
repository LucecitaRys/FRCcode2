// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private static Intake mIntakeSub;

  private final TalonFX Muneca = new TalonFX(Constants.MotorConstants.id_Mm);
  private final TalonFX Intake = new TalonFX(Constants.MotorConstants.id_mi);
  private TalonFXConfiguration MotorConfigM = new TalonFXConfiguration();
  private final TalonFXConfiguration MotorConfigI = new TalonFXConfiguration();
  private final PositionVoltage posvol = new PositionVoltage (0);

  public Intake() {
Muneca.setNeutralMode(NeutralModeValue.Brake);

 Slot0Configs slot0Configs = MotorConfigM.Slot0; 
slot0Configs.kP = 1.8;
slot0Configs.kI = 0;
slot0Configs.kD = 0;
slot0Configs.kS = 4;
slot0Configs.kG = 0;
slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

// Configuración de Límites de Corriente
CurrentLimitsConfigs currentLimitsConfigs = MotorConfigM.CurrentLimits;
currentLimitsConfigs.SupplyCurrentLowerLimit = 70;
currentLimitsConfigs.SupplyCurrentLowerTime = 1;
currentLimitsConfigs.SupplyCurrentLimit = 120;
currentLimitsConfigs.SupplyCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimit = 120;

CurrentLimitsConfigs CurrentLimitsM = MotorConfigM.CurrentLimits;
    CurrentLimitsM.SupplyCurrentLowerLimit = 40;
    CurrentLimitsM.SupplyCurrentLowerTime = 1;
    CurrentLimitsM.SupplyCurrentLimit = 70;
    CurrentLimitsM.SupplyCurrentLimitEnable = true;
    CurrentLimitsM.StatorCurrentLimitEnable = true;
    CurrentLimitsM.StatorCurrentLimit = 100;
Muneca.getConfigurator().apply(MotorConfigM); 


CurrentLimitsConfigs CurrentLimitsI = MotorConfigI.CurrentLimits;
    CurrentLimitsI.SupplyCurrentLowerLimit = 40;
    CurrentLimitsI.SupplyCurrentLowerTime = 1;
    CurrentLimitsI.SupplyCurrentLimit = 70;
    CurrentLimitsI.SupplyCurrentLimitEnable = true;
    CurrentLimitsI.StatorCurrentLimitEnable = true;
    CurrentLimitsI.StatorCurrentLimit = 100;
Intake.getConfigurator().apply(MotorConfigI);

Muneca.setPosition(0);

  }
  public void MANUALPOSE(double elePower ) {
    Muneca.set(elePower * 0.5);
      }
public void SetPosM(double pos) {
      Muneca.setControl(posvol.withPosition(pos));
     }
    public void ConstanVel(double vel){
      Intake.set(vel);
    }
    public static Intake getInstance (){
      if (mIntakeSub== null){
        mIntakeSub = new Intake();
      }
      return mIntakeSub;
    }
    public double Current(){
      //double Currentlim = Intake.getSupplyCurrent().getValueAsDouble();
      double CurrentLimit = Intake.getTorqueCurrent().getValueAsDouble();
      return CurrentLimit;
   }
  @Override
  public void periodic() {
      SmartDashboard.putNumber("Muneca", Muneca.getPosition().getValueAsDouble());
  SmartDashboard.putNumber("Curent", Current());
    }
}
