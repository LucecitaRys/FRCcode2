package frc.robot.subsystems;


import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ElevatorSub extends SubsystemBase {
    private static ElevatorSub mElevatorSub;
    private final TalonFX ElevatorR;
      public Encoder Encoderele= new Encoder(1, 0);
   

  // private Encoder encoder;

    public double posel;
   // private final TalonFX ElevatorL;
  public TalonFXConfiguration motorConf = new TalonFXConfiguration();

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

   // encoder = new Encoder(null, null);
ElevatorR= new TalonFX(Constants.MotorConstants.id_er);

//motorConf.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
//motorConf.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 18;
// Configuración de Slot (PID)
  Slot0Configs slot0Configs = new Slot0Configs(); 
slot0Configs.kP = 0;
slot0Configs.kI = 0;
slot0Configs.kD = 0;
slot0Configs.kS = 0;
slot0Configs.kV = 0;

// Configuración de Límites de Corriente
CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
currentLimitsConfigs.SupplyCurrentLowerLimit = 70;
currentLimitsConfigs.SupplyCurrentLowerTime = 1;
currentLimitsConfigs.SupplyCurrentLimit = 120;
currentLimitsConfigs.SupplyCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimit = 120;

FeedbackConfigs feedbackConfigs = new FeedbackConfigs(); 
  feedbackConfigs.FeedbackRemoteSensorID = 0;
  feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; 
  feedbackConfigs.FeedbackRotorOffset = 2;
  feedbackConfigs.RotorToSensorRatio = 0;
  feedbackConfigs.SensorToMechanismRatio = 0;

  // Configuración de Motion Magic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 15000; 
    motionMagicConfigs.MotionMagicAcceleration = 6000;    
    motionMagicConfigs.MotionMagicJerk = 0;   
 
    // Configuración de Límites de Posición
    
    //softLimitSwitchConfigs.ReverseSoftLimitEnable = true;
     // HardwareLimitSwitchConfigs hardwareLimitSwitch = new HardwareLimitSwitchConfigs();
     // hardwareLimitSwitch.ForwardLimitAutosetPositionValue=10;
     // hardwareLimitSwitch.ForwardLimitEnable = true;
    // Configuración de Rampas
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.5;

    // Asignación de las subconfiguraciones a la configuración general
    motorConf.Feedback = feedbackConfigs;
    motorConf.Slot0 = slot0Configs;
    motorConf.MotionMagic = motionMagicConfigs;
    motorConf.CurrentLimits = currentLimitsConfigs;
    motorConf.ClosedLoopRamps = closedLoopRampsConfigs;
    //motorConf.HardwareLimitSwitch = hardwareLimitSwitch;



  ElevatorR.getConfigurator().apply(motorConf); 
  //ElevatorL.getConfigurator().apply(motorConf); 

Encoderele.setDistancePerPulse(1);


Encoderele.setReverseDirection(false);

// Configures an encoder to average its period measurement over 5 samples
// Can be between 1 and 127 samples
Encoderele.setSamplesToAverage(5);
SmartDashboard.putNumber("EncoderElevator", Encoderele.getDistance());
    }
// metodo
public void GETPOSESELEVATORPower(double elePower ) {
  ElevatorR.set(elePower * 0.95);
  
    }

public void setPosElevator(double pos ) {
ElevatorR.setPosition(pos);
SmartDashboard.putNumber("Encoder", Encoderele.getDistance());
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
//public double GestPosEle(){
// return encoder.getDistance();
//}

  
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
      public void reset(){
        Encoderele.reset();
       }
 @Override
 public void periodic() {
     SmartDashboard.putNumber("EncoderElevator", Encoderele.getDistance());
     SmartDashboard.putNumber("position", ElevatorR.getPosition().getValueAsDouble());

 } 

}