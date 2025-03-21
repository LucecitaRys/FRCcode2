package frc.robot.subsystems;


import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ElevatorSub extends SubsystemBase {
    private static ElevatorSub mElevatorSub;
    private final TalonFX ElevatorR;
private final TalonFX ElevatorL;
    private final PositionVoltage posvol = new PositionVoltage (0);
double volts;
public double posel;
public TalonFXConfiguration motorConf = new TalonFXConfiguration();
public TalonFXConfiguration motorConfr = new TalonFXConfiguration();



  public ElevatorSub() {


ElevatorR= new TalonFX(Constants.MotorConstants.id_er);
ElevatorL = new TalonFX(Constants.MotorConstants.id_el);
ElevatorL.setNeutralMode(NeutralModeValue.Brake);
ElevatorR.setNeutralMode(NeutralModeValue.Brake);


  Slot0Configs slot0Configs = motorConf.Slot0; 
slot0Configs.kP = 1.3;
slot0Configs.kI = 0;
slot0Configs.kD = 0;
slot0Configs.kS = 5;
slot0Configs.kG = 0;
slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

// Configuración de Límites de Corriente
CurrentLimitsConfigs currentLimitsConfigs =motorConf.CurrentLimits;
currentLimitsConfigs.SupplyCurrentLowerLimit = 70;
currentLimitsConfigs.SupplyCurrentLowerTime = 1;
currentLimitsConfigs.SupplyCurrentLimit = 120;
currentLimitsConfigs.SupplyCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimit = 120;

CurrentLimitsConfigs currentLimitsConfig =motorConfr.CurrentLimits;
currentLimitsConfig.SupplyCurrentLowerLimit = 70;
currentLimitsConfig.SupplyCurrentLowerTime = 1;
currentLimitsConfig.SupplyCurrentLimit = 120;
currentLimitsConfig.SupplyCurrentLimitEnable = true;
currentLimitsConfig.StatorCurrentLimitEnable = true;
currentLimitsConfig.StatorCurrentLimit = 120;



 
ElevatorR.setControl(new Follower(ElevatorL.getDeviceID(), true));



  ElevatorR.getConfigurator().apply(motorConfr); 
  ElevatorL.getConfigurator().apply(motorConf); 
  ElevatorR.setPosition(0);
  ElevatorL.setPosition(0);
  

    }
// metodo
public void GETMANUALPOS(double elePower ) {
  ElevatorL.set(elePower);
  
    }

public void setPosElevator(double pos) {

ElevatorL.setControl(posvol.withPosition(pos));

  }

  public  double getPosEle(){
    return ElevatorL.getPosition().getValueAsDouble();
  }

  

    public static ElevatorSub getInstance (){
      if (mElevatorSub== null){
        mElevatorSub= new ElevatorSub();
      }
      return mElevatorSub;
    }
    public void reset(){
      ElevatorR.resetSignalFrequencies();
    }


  
   
      
 @Override
 public void periodic() {
  
     SmartDashboard.putNumber("positionER", ElevatorR.getPosition().getValueAsDouble());
     SmartDashboard.putNumber("positionEL", ElevatorL.getPosition().getValueAsDouble());
 } 

}