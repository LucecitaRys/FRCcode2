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
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
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
public TalonFXConfiguration motorConfL = new TalonFXConfiguration();

  public enum ElePoses {
    none,
    collect,
    nivel1,
    nivel2,
    nivel3,
    nivel4,
    throwAlagae,
    intakeVC;
  }
  public ElePoses ElPos = ElePoses.none;   

  public ElevatorSub() {


ElevatorR= new TalonFX(Constants.MotorConstants.id_er);
ElevatorL = new TalonFX(Constants.MotorConstants.id_el);



  Slot0Configs slot0Configs = motorConf.Slot0; 
slot0Configs.kP = 8;
slot0Configs.kI = 0;
slot0Configs.kD = 0;
slot0Configs.kS = 5;
slot0Configs.kG = 0;
slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

// Configuración de Límites de Corriente
CurrentLimitsConfigs currentLimitsConfigs =motorConf.CurrentLimits;
currentLimitsConfigs.SupplyCurrentLowerLimit = 70;
currentLimitsConfigs.SupplyCurrentLowerTime = 1;
currentLimitsConfigs.SupplyCurrentLimit = 120;
currentLimitsConfigs.SupplyCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimit = 120;


/*Slot0Configs slot0ConfigsL = motorConfL.Slot0; 
slot0ConfigsL.kP = 8;
slot0ConfigsL.kI = 0;
slot0ConfigsL.kD = 0;
slot0ConfigsL.kS = 5;
slot0ConfigsL.kG = 0;
slot0ConfigsL.GravityType = GravityTypeValue.Arm_Cosine;
slot0ConfigsL.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

// Configuración de Límites de Corriente
CurrentLimitsConfigs currentLimitsConfigsL =motorConfL.CurrentLimits;
currentLimitsConfigsL.SupplyCurrentLowerLimit = 70;
currentLimitsConfigsL.SupplyCurrentLowerTime = 1;
currentLimitsConfigsL.SupplyCurrentLimit = 120;
currentLimitsConfigsL.SupplyCurrentLimitEnable = true;
currentLimitsConfigsL.StatorCurrentLimitEnable = true;
currentLimitsConfigsL.StatorCurrentLimit = 120;
*/

 
ElevatorL.setControl(new Follower(ElevatorR.getDeviceID(), true));



  ElevatorR.getConfigurator().apply(motorConf); 
  ElevatorL.getConfigurator().apply(motorConf); 
  ElevatorR.setPosition(0);
  ElevatorL.setPosition(0);
  

    }
// metodo
public void GETMANUALPOS(double elePower ) {
  ElevatorR.set(elePower * 0.5);
    }

public void setPosElevator(double pos) {
ElevatorR.setControl(posvol.withPosition(pos));
ElevatorL.setControl(posvol.withPosition(pos));
  }

  public  double getPosEle(){
    return ElevatorR.getPosition().getValueAsDouble();
  }

  public void setElevatorPoses (ElePoses elevadoStates) {
      if (ElPos != elevadoStates) {ElPos = elevadoStates; }
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


  
    public void setposEl(){
      switch (ElPos) {
        case none:
          setPosElevator(0);
          posel= 0;

          break;
          case collect:
          setPosElevator(0);          
          posel= 0;
          break; 
          case nivel1:
          setPosElevator(0);
          posel= 0;

          break;
          case nivel2:
          setPosElevator(0);
          posel= 0;
          
          break;
          case nivel3:
          setPosElevator(0);
          posel= 0;

          break;
          case nivel4:
          setPosElevator(0);
          posel= 0;

          break;
          case throwAlagae:
          setPosElevator(0);
          posel= 0;

          break;
        default:
          break;
        }
      }
      
 @Override
 public void periodic() {
  
     SmartDashboard.putNumber("position", ElevatorR.getPosition().getValueAsDouble());
 } 

}