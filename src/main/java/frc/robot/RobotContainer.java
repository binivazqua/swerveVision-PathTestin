/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

package frc.robot;


import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.lib.util.alignConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PS4OIConstants;
import frc.robot.Constants.limelightConstants;
import frc.robot.commands.ArmVelocityCommand;
import frc.robot.commands.PhotonLLCommand;
import frc.robot.commands.PivoteoCommand;
import frc.robot.commands.swerveDriveComando;
import frc.robot.commands.autos.autos;
import frc.robot.commands.limelight.autoAlign;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLightObject;
import frc.robot.subsystems.PhotonLL;
import frc.robot.subsystems.swerveSusbsystem;
import frc.robot.subsystems.Mecanismos.Pivoteo;

public class RobotContainer {

    //private final subsistemaSwerve swerveSubsystem = new subsistemaSwerve();
    private swerveSusbsystem swerveSubsystem;
    private LimeLightObject limelight;
    private PhotonLL photoncamera;
    private Pivoteo arm;

    public static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    public static Joystick placerJoystick = new Joystick(OIConstants.kPlacerControllerPort);
    
    
  

    public RobotContainer(){

        swerveSubsystem = swerveSusbsystem.getInstance();
        limelight  = LimeLightObject.getInstance();
        photoncamera = PhotonLL.getInstance();
        arm = Pivoteo.getInstance();

        // "save" a command in order to call it within an event marker.

        /* 
        swerveSubsystem.setDefaultCommand(new swerveDriveComando(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                true
                ));

        /* +++++ NO ME FUNEN ++++
        */
        swerveSubsystem.setDefaultCommand(new swerveDriveComando(
                    swerveSubsystem,
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverYAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverXAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverRotAxis),
                    () -> driverJoytick.getRawButton(PS4OIConstants.kDriverFieldOrientedButtonIdx),
                    true
                    ));
    

              //limelight.setDefaultCommand(new limelighCommand(swerveSubsystem, limelight, false));
              photoncamera.setDefaultCommand(new PhotonLLCommand());

              //pivoteo deffault command:
              arm.setDefaultCommand(new ArmVelocityCommand(0));

                
              
        configureButtonBindings();

         
         
    }

    
    private void configureButtonBindings() {

        //APRIL TAG:
       //new JoystickButton(driverJoytick, 5).whileTrue(new autoAlign());

       //PS4:
       //new JoystickButton(driverJoytick, 2).whileTrue(new autoAlign(limelightConstants.noteOffsets.offsets));
       //new JoystickButton(driverJoytick, 1).whileTrue(new autoAlign(limelightConstants.aprilTag.offsets));


       //PID PIVOTEO PRUEBA:
       //new JoystickButton(placerJoystick, 3).whileTrue(new PivoteoCommand(0.2)); // x
       //new JoystickButton(placerJoystick, 1).whileTrue(new PivoteoCommand(0.25)); // a
       //new JoystickButton(placerJoystick, 2).whileTrue(new PivoteoCommand(0.05)); // b

       // SHOOTING POSITIONS:
       // --> SUBWOOFER <----
       new JoystickButton(placerJoystick, 3).whileTrue(new PivoteoCommand(0.061)); // 35°

       // --> ROBOT STARTING ZONE <---
       new JoystickButton(placerJoystick, 1).whileTrue(new PivoteoCommand(0.0820)); // x


       // --> 50° <---
       new JoystickButton(placerJoystick,2).whileTrue(new PivoteoCommand(0.122)); // 44°






        
       //REFLECTIVE TAPE:
        //new JoystickButton(driverJoytick, 4).whileTrue(new autoAlign(swerveSubsystem, limelight, false));

    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
       //return autos.autoForward();
       //return autos.test_papaya();
       return null;

        // 5. Add some init and wrap-up, and return everything
    }


}
