// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Constants {
    
    public static double kLooperDt = 0.02; 

    public static class Field {
        public static final double length = 17.55; 
        public static final double width = 8.05; 
    }



    

  // public static TrajectoryConfig createTrajConfig (double maxVel, double maxAccel) {
     //   TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
    //    config.setStartVelocity(0); 
    //    config.setEndVelocity(0); 
    //    return new TrajectoryConfig(maxVel, maxAccel);
  //  }
 
  
  
    public static final class MotorConstants {
        public static final int id_ms= 31; 
         public static final int id_mi= 17; 
         public static final int id_er = 22;
         public static final int id_mb =21;
         public static final int id_cl=47;
      }

      public static final class  Swerve {
      public static final Translation2d flModuleOffset = new Translation2d(0.546 / 2.0, 0.546 / 2.0);
    public static final Translation2d frModuleOffset = new Translation2d(0.546 / 2.0, -0.546 / 2.0);
    public static final Translation2d blModuleOffset = new Translation2d(-0.546 / 2.0, 0.546 / 2.0);
    public static final Translation2d brModuleOffset = new Translation2d(-0.546 / 2.0, -0.546 / 2.0);
   
        public static final double maxModuleSpeed = 4.5; // M/S

    public static final PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants rotationConstants = new PIDConstants(5.0, 0.0, 0.0);
      }
}
