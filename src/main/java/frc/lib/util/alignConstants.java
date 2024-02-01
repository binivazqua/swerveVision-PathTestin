/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

 package frc.lib.util;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;


 public class alignConstants {
     
     public double driveOffset;
     public double strafeOffset;
     public double rotationOffset;
     public PIDController drivePID, strafePID, rotationPID;

 
     public alignConstants(
        double driveValue,
        double strafeValue,
        double rotationOffset,
        PIDController drivePID,
        PIDController strafePID,
        PIDController rotationPID     
     ){
         this.driveOffset = driveValue;
         this.strafeOffset = strafeValue;
         this.rotationOffset = rotationOffset;
         this.drivePID = drivePID;
         this.strafePID = strafePID;
         this.rotationPID = rotationPID;    
 
     }
 }
 