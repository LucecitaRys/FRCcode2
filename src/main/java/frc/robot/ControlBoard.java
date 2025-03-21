// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.OpenOption;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class ControlBoard {
    // public static final XboxController drive = new XboxController(0); 
    //public static final XboxController operator = new XboxController(1); 
    public static final PS4Controller operator= new PS4Controller(1);
    
public static boolean COLLECT(){
     return operator.getCrossButtonReleased();
}
public static boolean Nivel1(){
    return operator.getCircleButtonReleased();
}
public static boolean Nivel2(){
    return operator.getTriangleButtonReleased();
}
public static boolean Nivel3(){
    return operator.getSquareButtonReleased();
}

public static Boolean Continue(){
    return operator.getOptionsButton();
}
public static boolean Stop(){
    return operator.getShareButtonReleased();
}

/*public static boolean algae1(){
    return drive.getAButton();
}
public static boolean algae2(){
    return drive.getYButtonReleased();
}
public static boolean coll(){
    return operator.getR1ButtonReleased();
} z*/
/*public static boolean Nivel1(){
    return operator.getBButtonReleased();
}
public static boolean Nivel2(){
    return operator.getYButtonReleased();
}  
public static boolean Nivel3(){
    return operator.getXButtonReleased();
} 
public static Boolean Continue(){
    return operator.getStartButtonReleased();
}
public static boolean Stop(){
    return operator.getBackButtonReleased();
}*/
    public static double getLeftY_ope (){
        return MathUtil.applyDeadband(-operator.getLeftY(), 0.2); 
    }
    public static double getRightY_ope (){
        return MathUtil.applyDeadband(-operator.getRightY()*0.4, 0.2); 
    }
    public static double getLeftXop(){
        return MathUtil.applyDeadband(-operator.getLeftX(),0.2);
    }
   
    public static Boolean Buttonthrow(){
        return operator.getR1Button();
    }
  public static Boolean ButtonCollect(){
     return operator.getL1Button();
  }
    
    public static Double ButtonControVelIntake(){

        return operator.getR2Axis()-operator.getL2Axis();
   }

 //  public static Double velAl(){

  //  return drive.getRightTriggerAxis()-drive.getLeftTriggerAxis();
//}

    
    
}
