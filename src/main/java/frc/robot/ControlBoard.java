// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.OpenOption;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class ControlBoard {
     public static final XboxController driver = new XboxController(0); 
    public static final XboxController operator = new XboxController(1); 

public static boolean COLLECT(){
    return operator.getAButton();
}
public static boolean Nivel1(){
    return operator.getBButton();
}
public static boolean Nivel2(){
    return operator.getXButton();
}  
public static boolean Nivel3(){
    return operator.getYButton();
} 
    public static double getLeftY_ope (){
        return MathUtil.applyDeadband(-operator.getLeftY(), 0.2); 
    }
    public static double getRightY_ope (){
        return MathUtil.applyDeadband(-operator.getRightY()*0.4, 0.2); 
    }
    public static double getLeftXop(){
        return MathUtil.applyDeadband(-operator.getLeftX(),0.2);
    }
    public static boolean buttonA (){
        return operator.getAButton(); 
    }
    public static boolean buttonB(){
        return operator.getBButton(); 
    }
  
    public static boolean buttonx (){
        return operator.getXButton(); 
    }
    public static boolean buttony(){
        return operator.getYButton(); 
    }
    public static Boolean button5(){
        return operator.getRightBumperButtonPressed();
    }
    public static Boolean ButtonCOLLECT(){
        return operator.getLeftBumperButton();
    }
    public static Boolean Buttonthrow(){
        return operator.getRightBumperButton();
    }
    public static Boolean ButtonCORAL(){
        return operator.getBackButtonPressed();
    }
    public static Boolean ButtonALGAE(){
        return operator.getStartButtonPressed();
    }
    public static Double ButtonMuL(){
        return operator.getRightTriggerAxis()-operator.getLeftTriggerAxis();
    }
    
    public static Double ButtonClimPosR(){
        return driver.getRightTriggerAxis()-driver.getLeftTriggerAxis();
    }
    
    public static boolean ButtonPosEle1(){
        return operator.getStartButton();
    }
    public static boolean ButtonReset(){
        return operator.getBackButton();
    }
}
