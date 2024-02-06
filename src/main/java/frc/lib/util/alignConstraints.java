/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

 package frc.lib.util;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;


 public class alignConstraints {
     
     public double driveOffset;
     public double strafeOffset;
     public double rotationOffset;
     public ProfiledPIDController drivePID, strafePID, rotationPID;

 
     public alignConstraints(
        double driveValue,
        double strafeValue,
        double rotationOffset,
        ProfiledPIDController drivePID,
        ProfiledPIDController strafePID,
        ProfiledPIDController rotationPID     
     ){
         this.driveOffset = driveValue;
         this.strafeOffset = strafeValue;
         this.rotationOffset = rotationOffset;
         this.drivePID = drivePID;
         this.strafePID = strafePID;
         this.rotationPID = rotationPID;    
 
     }
 }
 