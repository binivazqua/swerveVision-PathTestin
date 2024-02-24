/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PS4OIConstants;
import frc.robot.Constants.limelightConstants.aprilTag;
import frc.robot.commands.Mecanismos.ArmVelocityCommand;
import frc.robot.commands.Mecanismos.ClimbCommand;
import frc.robot.commands.Mecanismos.IntakeButtonCmd;
import frc.robot.commands.Mecanismos.PhotonLLCommand;
import frc.robot.commands.Mecanismos.PivoteoCommand;
import frc.robot.commands.Mecanismos.ShooterButtonCmd;
import frc.robot.commands.hybrid.autos;
import frc.robot.commands.hybrid.subroutines;
import frc.robot.commands.swerve.autoAlign;
import frc.robot.commands.swerve.swerveDriveComando;
import frc.robot.subsystems.Mecanismos.ClimberSubsystem;
import frc.robot.subsystems.Mecanismos.IntakeSubsystem;
import frc.robot.subsystems.Mecanismos.Pivoteo;
import frc.robot.subsystems.Mecanismos.ShooterSubsystem;
import frc.robot.subsystems.swerve.swerveSusbsystem;
import frc.robot.subsystems.vision.PhotonLL;

public class RobotContainer {

    private swerveSusbsystem swerveSubsystem;
    private PhotonLL photoncamera;
    private Pivoteo arm;
    private ClimberSubsystem climber;
    private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;


    public static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    public static Joystick placerJoystick = new Joystick(OIConstants.kPlacerControllerPort);
    
    public RobotContainer(){

        
        
        swerveSubsystem = swerveSusbsystem.getInstance();
        photoncamera = PhotonLL.getInstance();
        arm = Pivoteo.getInstance();
        climber = ClimberSubsystem.getInstance();
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

        */
        swerveSubsystem.setDefaultCommand(new swerveDriveComando(
                    swerveSubsystem,
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverYAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverXAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverRotAxis),
                    () -> driverJoytick.getRawButton(PS4OIConstants.kDriverFieldOrientedButtonIdx),
                    true
                    ));

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

        climber.setDefaultCommand(
            new ClimbCommand(false)
        );

        

                
              
        configureButtonBindings();

         
         
    }

   

    
    private void configureButtonBindings() {

        //APRIL TAG:
       //new JoystickButton(driverJoytick, 5).whileTrue(new autoAlign());

       //PS4:
       //new JoystickButton(driverJoytick, 2).whileTrue(new autoAlign(limelightConstants.noteOffsets.offsets));
       new JoystickButton(driverJoytick, 1).whileTrue(new autoAlign(aprilTag.constraints));


       //PID PIVOTEO PRUEBA:
       //new JoystickButton(placerJoystick, 3).whileTrue(new PivoteoCommand(0.2)); // x
       //new JoystickButton(placerJoystick, 1).whileTrue(new PivoteoCommand(0.25)); // a
       //new JoystickButton(placerJoystick, 2).whileTrue(new PivoteoCommand(0.05)); // b

       // SHOOTING POSITIONS:
       // --> SUBWOOFER <----
       //new JoystickButton(placerJoystick, 10).whileTrue(new PivoteoCommand(0.061)); // 35°
        

       // --> ROBOT STARTING ZONE <---
       //new JoystickButton(placerJoystick, 1).whileTrue(new PivoteoCommand(0.0820)); // x


       // --> 50° <---
       //new JoystickButton(placerJoystick,10).whileTrue(new PivoteoCommand(0.122)); // 44°
       new JoystickButton(placerJoystick,9).whileTrue(new PivoteoCommand(0.075)); // 44°

       new JoystickButton(placerJoystick,10).whileTrue(new PivoteoCommand(0.40)); // 44°


        // BAJAR BRAZO
        /* 
        new JoystickButton(placerJoystick,1).whileTrue(new SequentialCommandGroup(
            new PivoteoCommand(0.26),
            new PivoteoCommand(0.061)
        )); // 44°
        */

        
    /* 
       m_operatorController.R1().whileTrue(new ShooterButtonCmd(-0.75));
       m_operatorController.cross().whileTrue(new IntakeButtonCmd(0.5));
       m_operatorController.square().whileTrue(new IntakeButtonCmd(-0.5));
       m_operatorController.R2().whileTrue(shooter_intake_conDelay());
      */ 
    

        // COMMENTED OTHER COMMANDS:
         
        new JoystickButton(placerJoystick, 5).whileTrue(new ShooterButtonCmd(-0.75));
        new JoystickButton(placerJoystick, 6).whileTrue(subroutines.shootWithDelay());
        new JoystickButton(placerJoystick, 2).whileTrue(new IntakeButtonCmd(0.5));
        
        //recoger
        new JoystickButton(placerJoystick, 1).whileTrue(new IntakeButtonCmd(-0.5, true));


        // DRIVER - MECHANISMS //

        // JoystickButton(driverJoytick, 5).whileTrue(new ShooterButtonCmd(-0.75));
        /* 
        new JoystickButton(driverJoytick, 6).whileTrue(subroutines.shootWithDelay());
        new JoystickButton(driverJoytick, 2).whileTrue(new IntakeButtonCmd(0.5));
        new JoystickButton(driverJoytick, 3).whileTrue(new IntakeButtonCmd(-0.5));
        */

        // CLIMBER COMMAND:
        new POVButton(placerJoystick, 90).whileTrue(new ClimbCommand(true));




        
    }

    public Command getAutonomousCommand() {
       
       return autos.test_papaya();

    }


}
