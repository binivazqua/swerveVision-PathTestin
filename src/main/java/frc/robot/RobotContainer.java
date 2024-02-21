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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.lib.util.alignConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PS4OIConstants;
import frc.robot.Constants.limelightConstants;
import frc.robot.commands.ArmVelocityCommand;
import frc.robot.commands.PhotonLLCommand;
import frc.robot.commands.PivoteoCommand;
import frc.robot.commands.swerveDriveComando;
import frc.robot.commands.Mecanismos.IntakeButtonCmd;
import frc.robot.commands.Mecanismos.ShooterButtonCmd;
import frc.robot.commands.autos.autos;
import frc.robot.commands.limelight.autoAlign;
import frc.robot.subsystems.LimeLightObject;
import frc.robot.subsystems.PhotonLL;
import frc.robot.subsystems.swerveSusbsystem;
import frc.robot.subsystems.Mecanismos.IntakeSubsystem;
import frc.robot.subsystems.Mecanismos.Pivoteo;
import frc.robot.subsystems.Mecanismos.ShooterSubsystem;

public class RobotContainer {

    //private final subsistemaSwerve swerveSubsystem = new subsistemaSwerve();
    private swerveSusbsystem swerveSubsystem;
    private LimeLightObject limelight;
    private PhotonLL photoncamera;
    private Pivoteo arm;
    private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;


    public static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    public static Joystick placerJoystick = new Joystick(OIConstants.kPlacerControllerPort);
    
    
  

    public RobotContainer(){

        swerveSubsystem = swerveSusbsystem.getInstance();
        limelight  = LimeLightObject.getInstance();
        photoncamera = PhotonLL.getInstance();
        arm = Pivoteo.getInstance();
        m_intakeSubsystem = IntakeSubsystem.getInstance();
        m_shooterSubsystem = ShooterSubsystem.getInstance();

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

        // Intake:
        m_intakeSubsystem.setDefaultCommand(
            new IntakeButtonCmd(0)//
        );

        // shooter:
        m_shooterSubsystem.setDefaultCommand(
            new ShooterButtonCmd(0)//
        );

        

                
              
        configureButtonBindings();

         
         
    }

    public static Command shooter_intake_conDelay() {
        return new SequentialCommandGroup(
            new ShooterButtonCmd(0.75).withTimeout(1.7),
            new ParallelCommandGroup(
                new ShooterButtonCmd(0.75),
                new IntakeButtonCmd(-0.5)).withTimeout(2));
    
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
       //new JoystickButton(placerrJoystick, 5).whileTrue(new PivoteoCommand(0.061)); // 35°
       new JoystickButton(driverJoytick, 5).whileTrue(new PivoteoCommand(0.061)); // 35°

       // --> ROBOT STARTING ZONE <---
       //new JoystickButton(placerJoystick, 1).whileTrue(new PivoteoCommand(0.0820)); // x


       // --> 50° <---
       //new JoystickButton(placerJoystick,10).whileTrue(new PivoteoCommand(0.122)); // 44°
       new JoystickButton(placerJoystick,10).whileTrue(new PivoteoCommand(0.075)); // 44°


    /* 
       m_operatorController.R1().whileTrue(new ShooterButtonCmd(-0.75));
       m_operatorController.cross().whileTrue(new IntakeButtonCmd(0.5));
       m_operatorController.square().whileTrue(new IntakeButtonCmd(-0.5));
       m_operatorController.R2().whileTrue(shooter_intake_conDelay());
      */ 
    

        // COMMENTED OTHER COMMANDS:
        new JoystickButton(placerJoystick, 5).whileTrue(new ShooterButtonCmd(-0.75));
        new JoystickButton(placerJoystick, 6).whileTrue(shooter_intake_conDelay());
        new JoystickButton(placerJoystick, 2).whileTrue(new IntakeButtonCmd(0.5));
        new JoystickButton(placerJoystick, 1).whileTrue(new IntakeButtonCmd(-0.5));


        // DRIVER - MECHANISMS //

        // JoystickButton(driverJoytick, 5).whileTrue(new ShooterButtonCmd(-0.75));
        new JoystickButton(driverJoytick, 6).whileTrue(shooter_intake_conDelay());
        new JoystickButton(driverJoytick, 2).whileTrue(new IntakeButtonCmd(0.5));
        new JoystickButton(driverJoytick, 3).whileTrue(new IntakeButtonCmd(-0.5));




        
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
