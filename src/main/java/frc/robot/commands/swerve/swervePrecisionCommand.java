/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swerve.swerveSusbsystem;

public class swervePrecisionCommand extends Command {

    private final swerveSusbsystem swerveSubsystem;
    private final Supplier<Double> driveFunction, strafeFunction, giroFunction;
    private final Supplier<Boolean> isFieldOriented;
    private final SlewRateLimiter xLimiter, yLimiter, giroLimiter;
    private final  Boolean joystickDrive;


    /**
     * Driving command
     * 
     * @param swerveSubsystem Swerve subsystem in use
     * @param driveVelocityFunction Supplier for the drive function
     * @param strafeVelocityFunction Supplier for the strafe function
     * @param rotationVelocityFunction Supplier for the rotation function
     * @param fieldOrientedFunction Boolean for whether the drive will be field oriented
     */
    public swervePrecisionCommand(swerveSusbsystem subsistemaSwerve,
            Supplier<Double> velAvanceFuncion, Supplier<Double> velStrafeFuncion, Supplier<Double> velGiroFuncion,
            Supplier<Boolean> fieldOrientedFunction, Boolean joystickDrive
            ) {
       
        /**
         * Subsystem
         */
        this.swerveSubsystem = subsistemaSwerve;
        

        this.driveFunction = velAvanceFuncion;
        this.strafeFunction = velStrafeFuncion;
        this.giroFunction = velGiroFuncion;
        
        this.isFieldOriented = fieldOrientedFunction;
        
        this.joystickDrive = joystickDrive;
        
       /**
        * Limiters for acceleration and a better moving of the robot
        * 
        */
        this.xLimiter = new SlewRateLimiter(DriveConstants.kDriveAccelerationLimiter);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kDriveAccelerationLimiter);
        this.giroLimiter = new SlewRateLimiter(DriveConstants.kRotationAccelerationLimiter);
        
       
        addRequirements(subsistemaSwerve);
    }

    @Override
    public void initialize() {
    
        
    }

    @Override
    public void execute() {

        // 1. Get real-time joystick inputs
 if(joystickDrive == true){
        double driveVel;
        double strafeVel;
        double rotationVel; 


    driveVel = driveFunction.get();
    strafeVel = strafeFunction.get();
    rotationVel = giroFunction.get();  


        // 2. Apply deadband
        driveVel = Math.abs(driveVel) > OIConstants.kDeadband ? driveVel : 0.0;
        strafeVel = Math.abs(strafeVel) > OIConstants.kDeadband ? strafeVel : 0.00;
        rotationVel = Math.abs(rotationVel) > OIConstants.kDeadband ? rotationVel : 0.00;

         // 3. Make the driving smoother
         // CHANGE VELOCITY @RAMOS//
        driveVel = xLimiter.calculate(driveVel) * 5;
        strafeVel = yLimiter.calculate(strafeVel) * 5;
        rotationVel = giroLimiter.calculate(rotationVel)
                * 7;

                SmartDashboard.putNumber("drive velocity", driveVel);
                SmartDashboard.putNumber("strafe velocity", strafeVel);
                SmartDashboard.putNumber("rotation velocity", rotationVel);

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveVel, strafeVel, rotationVel, swerveSubsystem.getRotation2d());
        
             //Relative to robot
        

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    } else {
        return;
    }
    } 

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
