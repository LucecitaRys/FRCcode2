// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algaes extends SubsystemBase {
  /** Creates a new Algaes. */
  public TalonFX PosAlgae = new TalonFX(31);
  public TalonFX velAl= new TalonFX(32);
  public TalonFXConfiguration motorConfP = new TalonFXConfiguration();
public TalonFXConfiguration motorConfV = new TalonFXConfiguration();
  private final PositionVoltage posvolt = new PositionVoltage (0);
  private static Algaes mAlgaes;
  public Algaes() {
    CurrentLimitsConfigs currentLimitsConfig =motorConfP.CurrentLimits;
currentLimitsConfig.SupplyCurrentLowerLimit = 70;
currentLimitsConfig.SupplyCurrentLowerTime = 1;
currentLimitsConfig.SupplyCurrentLimit = 120;
currentLimitsConfig.SupplyCurrentLimitEnable = true;
currentLimitsConfig.StatorCurrentLimitEnable = true;
currentLimitsConfig.StatorCurrentLimit = 120;

 Slot0Configs slot0Configs = motorConfP.Slot0; 
slot0Configs.kP = 8;
slot0Configs.kI = 0;
slot0Configs.kD = 0;
slot0Configs.kS = 7;
slot0Configs.kG = 0;

CurrentLimitsConfigs currentLimitsConfigs =motorConfV.CurrentLimits;
currentLimitsConfigs.SupplyCurrentLowerLimit = 70;
currentLimitsConfigs.SupplyCurrentLowerTime = 1;
currentLimitsConfigs.SupplyCurrentLimit = 120;
currentLimitsConfigs.SupplyCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimit = 120;
PosAlgae.getConfigurator().apply(motorConfP);
PosAlgae.setPosition(0);

  }


  public void PosMAl(double position) {
    PosAlgae.setControl(posvolt.withPosition(position));
   }
   public void setVelocityAlgae(double velo){
    velAl.set(velo);
   }
   
   public static Algaes getInstance (){
    if (mAlgaes== null){
      mAlgaes= new Algaes();
    }
    return mAlgaes;
  }

  @Override
  public void periodic() {
     SmartDashboard.putNumber("PositionAlgae", PosAlgae.getPosition().getValueAsDouble());
  }
}
