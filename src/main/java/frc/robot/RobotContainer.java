  /**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PS4OIConstants;
import frc.robot.Constants.limelightConstants.aprilTag;
import frc.robot.Constants.limelightConstants.trapAprilTag;
import frc.robot.commands.Mecanismos.ArmVelocityCommand;
import frc.robot.commands.Mecanismos.ClimbCommand;
import frc.robot.commands.Mecanismos.IntakeButtonCmd;
import frc.robot.commands.Mecanismos.PhotonLLCommand;
import frc.robot.commands.Mecanismos.PivoteoByInterpolation;
import frc.robot.commands.Mecanismos.PivoteoCommand;
import frc.robot.commands.Mecanismos.ShooterButtonCmd;
import frc.robot.commands.Mecanismos.setPivotVelocity;
import frc.robot.commands.hybrid.autos;
import frc.robot.commands.hybrid.subroutines;
import frc.robot.commands.swerve.autoAlign;
import frc.robot.commands.swerve.swerveDriveComando;
import frc.robot.commands.swerve.swervePrecisionCommand;
import frc.robot.subsystems.Mecanismos.ClimberSubsystem;
import frc.robot.subsystems.Mecanismos.IntakeSubsystem;
import frc.robot.subsystems.Mecanismos.Pivoteo;
import frc.robot.subsystems.Mecanismos.ShooterSubsystem;
import frc.robot.subsystems.swerve.swerveSusbsystem;
import frc.robot.subsystems.vision.PhotonLL;

public class RobotContainer {

    private swerveSusbsystem swerve;
    private PhotonLL photoncamera;
    private Pivoteo arm;
    private ClimberSubsystem climber;
    private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;


    public static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    public static Joystick placerJoystick = new Joystick(OIConstants.kPlacerControllerPort);
    
    public RobotContainer(){

        
        
        swerve = swerveSusbsystem.getInstance();
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
        swerve.setDefaultCommand(new swerveDriveComando(
                    swerve,
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverYAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverXAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverRotAxis),
                    () -> driverJoytick.getRawButton(PS4OIConstants.kDriverFieldOrientedButtonIdx),
                    true
                    ));


              //pivoteo deffault command:
       // arm.setDefaultCommand(new ArmVelocityCommand(0));
        // Intake:
        m_intakeSubsystem.setDefaultCommand(
            new IntakeButtonCmd(0)//
        );

        // shooter:
        m_shooterSubsystem.setDefaultCommand(
            new ShooterButtonCmd(0)//
        );

        /* comented to set deffault as "under the chain"
        climber.setDefaultCommand(
            new ClimbCommand(false)
        );
        */

        photoncamera.setDefaultCommand(new PhotonLLCommand());

                
              
        configureButtonBindings();

         
         
    }

   

    
    private void configureButtonBindings() {

       
         
       // ================================================== DRIVER JOYSTICKS  ========================================//

        // LOWER VELOCITY:
       new JoystickButton(driverJoytick, Constants.PS4OIConstants.topLeft).whileFalse(
        new swervePrecisionCommand(swerve, 
        () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverYAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverXAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverRotAxis),
                    () -> driverJoytick.getRawButton(PS4OIConstants.kDriverFieldOrientedButtonIdx),
                    true)
       );

        // INCREASE VELOCITY:
       new JoystickButton(driverJoytick, Constants.PS4OIConstants.topRight).whileFalse(
        new swerveDriveComando(swerve, 
        () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverYAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverXAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverRotAxis),
                    () -> driverJoytick.getRawButton(PS4OIConstants.kDriverFieldOrientedButtonIdx),
                    true)
       );

        // RESEAT HEADING:
        new JoystickButton(driverJoytick, Constants.PS4OIConstants.triangle).whileFalse(
            new InstantCommand(
                () -> swerve.resetHeading()
            )
        );

      /* */  arm.setDefaultCommand(
            new ArmVelocityCommand(0)
        );
     
        // ALIGN TO APRILTAG:
        new JoystickButton(driverJoytick, Constants.PS4OIConstants.square).whileTrue(new autoAlign(trapAprilTag.constraints));


        // CLIMBER:
        //new JoystickButton(driverJoytick, 7).whileTrue(new ClimbCommand(true)); // trigger left
        //new JoystickButton(driverJoytick, 8).whileTrue(new ClimbCommand(false)); // trigger right

        // RESET HEADING #2
        new JoystickButton(driverJoytick, 4).whileFalse(
            new InstantCommand(
                () -> swerve.resetHeading()
            )
        );

        // SHOOTEAR DE LEJOS
        //new JoystickButton(driverJoytick, Constants.PS4OIConstants.PSButton).whileTrue(subroutines.shootWithDelayLejos());

        // SET POSITION BY INTERPOLATION:
        //new JoystickButton(driverJoytick, Constants.PS4OIConstants.circle).whileTrue(new PivoteoByInterpolation());

       // ================================================== PLACER JOYSTICKS  =============================================//



       // =========================================== ÁNGULOS PIVOTEO ====================================================== //
        
        // BAJAR BRAZO
        /* 
        new JoystickButton(placerJoystick, Constants.PS4OIConstants.square).whileTrue(new SequentialCommandGroup(
            new PivoteoCommand(0.26),
            new PivoteoCommand(0.061)
        )); // 44°+
      
        new POVButton(placerJoystick, 90).whileTrue(
            new InstantCommand(
                () -> arm.setAbsoluteEncoderZero(0)
            )
        );

    
        /*new JoystickButton(placerJoystick, PS4OIConstants.triangle).whileTrue(
            new InstantCommand(
            () -> arm.resetEncoder()
            )
        );*/
        


        new JoystickButton(placerJoystick, PS4OIConstants.joystickIzq).whileTrue(new PivoteoCommand(0.17));

         
        //new JoystickButton(placerJoystick, PS4OIConstants.joystickDer).whileTrue(new PivoteoCommand(0.21));


        // --> APUNTAR <--
       // new JoystickButton(placerJoystick, PS4OIConstants.joystickIzq).whileTrue(new ArmVelocityCommand(0.15));

        //new JoystickButton(placerJoystick, PS4OIConstants.joystickIzq).whileTrue(new PivoteoCommand(0.2));

        // --> TEST 1 <-- SHOOT DE LEJOS
        new JoystickButton(placerJoystick, PS4OIConstants.PSButton).whileTrue(new PivoteoCommand(0.2));
        //new JoystickButton(placerJoystick, PS4OIConstants.joystickIzq).whileTrue(subroutines.apuntarLejos());



        new JoystickButton(placerJoystick, PS4OIConstants.square).whileTrue(subroutines.lowArm());
  //////////////////////////////// SUBIDA CHOLA
  /* 
        // --> FIJAR <--
        new JoystickButton(placerJoystick, PS4OIConstants.topLeft).whileTrue(new ArmVelocityCommand(0.075));

        // --> BAJAR <--
        new JoystickButton(placerJoystick, PS4OIConstants.square).whileTrue(new ArmVelocityCommand(-0.1));

        // --> ESCALAR <--
        new JoystickButton(placerJoystick, PS4OIConstants.joystickDer).whileTrue(new ArmVelocityCommand(0.3));
*/

        // ÁNGULO SHOOT:
       //new JoystickButton(placerJoystick,Constants.PS4OIConstants.joystickIzq).whileTrue(new PivoteoCommand( 0.075));// 30° // 0.19 //0.075

        //PID PIVOTING
       
       /*new JoystickButton(placerJoystick, PS4OIConstants.pad).whileTrue(new PivoteoCommand(0.23));

            //DESCOMENTAR PARA MATCH
         //86 PULGADAS A 60 GRADOS /// SI SIRVE /// TIRA DESDE LA NOTA DE ENMEDIO
         new JoystickButton(placerJoystick, Constants.PS4OIConstants.PSButton).whileTrue(new PivoteoCommand(0.19)); 

         // ÁNGULO TRAP:
         new JoystickButton(placerJoystick,Constants.PS4OIConstants.topLeft).whileTrue(new PivoteoCommand(0.154)); // 40° a 32.5in del trap encoder: 0.25 //0.122
 
         // UNDER THE CHAIN:
        new JoystickButton(placerJoystick,Constants.PS4OIConstants.triggerRight).whileTrue(new PivoteoCommand(0.43)); 
         
        // ÁNGULO CLIMBER:
         new JoystickButton(placerJoystick,Constants.PS4OIConstants.triggerRight).whileTrue(new PivoteoCommand(0.51)); // 90 grados (empujón)
        */
        
       // =============================== NOTE SCORING ======================


        // shoot sin delay:
        new JoystickButton(placerJoystick, PS4OIConstants.topLeft).whileTrue(new ShooterButtonCmd(0.75)); // 
        

        // SHOOT WITH DELAY
        //new JoystickButton(placerJoystick, Constants.PS4OIConstants.topRight).whileTrue(subroutines.shootWithDelay());

        // recoger
        // --> BACKUP <--

        /* ---> YA TIENE TOPE MECÁNICO <---
        new JoystickButton(placerJoystick, Constants.PS4OIConstants.cross).whileTrue(
            new ParallelCommandGroup(
                new IntakeButtonCmd(-0.5),
                new ArmVelocityCommand(0.075)
            )
        );
        */

        // ********true
        // --> PUSH NOTE <--

        new JoystickButton(placerJoystick, PS4OIConstants.topRight).whileTrue(
            new IntakeButtonCmd(-0.5)
        );

        // --
        new JoystickButton(placerJoystick, PS4OIConstants.cross).whileTrue(
            new IntakeButtonCmd(-0.4, true)
        );
        // ********true
        
        //comentado sig linea 09/03/24 8Mam
        //new JoystickButton(driverJoytick, PS4OIConstants.cross).whileTrue(new PivoteoCommand(0.25));
        //new JoystickButton(placerJoystick, PS4OIConstants.cross).whileTrue(new IntakeButtonCmd(-0.5));
        
        // escupir
        new JoystickButton(placerJoystick, Constants.PS4OIConstants.circle).whileTrue(new IntakeButtonCmd(0.6));
        
        
        // PRUEBA MOTOR:
        

        //new JoystickButton(placerJoystick, PS4OIConstants.square).whileTrue(new ShooterButtonCmd(-0.6));

       
       //pruebas amp
       //new JoystickButton(placerJoystick, PS4OIConstants.triggerLeftBtn).whileTrue(new IntakeButtonCmd(0.5));
        //new JoystickButton(placerJoystick, PS4OIConstants.shareBtn).whileTrue(new IntakeButtonCmd(0.6));
        new JoystickButton(placerJoystick, PS4OIConstants.optionsBtn).whileTrue(new IntakeButtonCmd(0.7));
        
        

        new JoystickButton(driverJoytick, 4).whileFalse(
            new InstantCommand(
                () -> swerve.resetHeading()
            )
        );

        // climber:+

        //new POVButton(placerJoystick, 90).whileTrue(new ClimbCommand(true));


        //================================== APRIL TAG: ===================================================================//
       //new JoystickButton(driverJoytick, 5).whileTrue(new autoAlign());
       //new JoystickButton(driverJoytick, 2).whileTrue(new autoAlign(limelightConstants.noteOffsets.offsets));

       // =============================  SHOOTING POSITIONS TBC =========================================================== //
       // --> SUBWOOFER <----
       //new JoystickButton(placerJoystick, 10).whileTrue(new PivoteoCommand(0.061)); // 35°

       // --> ROBOT STARTING ZONE <---
       //new JoystickButton(placerJoystick, 1).whileTrue(new PivoteoCommand(0.0820)); // x
        new JoystickButton(placerJoystick, PS4OIConstants.shareBtn).whileTrue(new PivoteoCommand(0.49)); // changed from 0.53 to 0.49
        new JoystickButton(placerJoystick, PS4OIConstants.joystickDer).whileTrue(new PivoteoCommand(0.6));

        new JoystickButton(placerJoystick, PS4OIConstants.triggerRight).whileTrue(new IntakeButtonCmd(0.5));
        // ======================================== X-BOX PLACER JOYSTICKS =================================================//
        // MECHANISMS
        //new JoystickButton(placerJoystick,9).whileTrue(new PivoteoCommand(0.075)); // 44°

       //new JoystickButton(placerJoystick,10).whileTrue(new PivoteoCommand(0.40)); // 

       // =================================== ÁNGULOS INCONCLUSOS (PROBAR) =================================================//

        // NO SE SABE
        //new JoystickButton(placerJoystick,Constants.PS4OIConstants.topLeft).whileTrue(new PivoteoCommand(0.061)); //0.35
        //new JoystickButton(placerJoystick,Constants.PS4OIConstants.topLeft).whileTrue(new PivoteoCommand(0.095)); // 35°
        // TIRAR DE LEJOS:
        //new JoystickButton(placerJoystick, Constants.PS4OIConstants.PSButton).whileTrue(new PivoteoCommand(0.1375)); 
        
        // ============================================ Tiro al AMP (Boton de PS) ==========================================//

        //new JoystickButton(placerJoystick, Constants.PS4OIConstants.PSButton).whileTrue(subroutines.shootToAmp()); 

        
    }

    public Command getAutonomousCommand() {
       
       //return subroutines.lowArmAndShoot();
       //return null;
      //return AutoBuilder.buildAuto("1and2OnCenter");
      
      //return autos.threeNoteCenterV2();
        //return AutoBuilder.buildAuto("1and2OnCenterHermosillo");
        return autos.OneTwoThreeCenterShooting();
        //return AutoBuilder.buildAuto("angleTest");

    }


}
