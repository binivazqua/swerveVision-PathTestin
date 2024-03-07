/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */
package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class autos {

    public autos(){
        SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();

        chooser.addOption(null, null);

        
    }    
   // private static PathPlannerPath test = PathPlannerPath.fromPathFile("platanop1");
    
    /*public static Command autoForward(){
        return Commands.sequence(TrajectoryReader.readTrajectory(goForward, true));
    }


    public static Command alignAuto(){

        return Commands.sequence(
            TrajectoryReader.readTrajectory(holonomic, true),
            new autoAlign(swerve, limelight, true));
    }

    public static Command holonomic(){
        return Commands.sequence(TrajectoryReader.readTrajectory(holonomic, true),
        new autoAlign(swerve, limelight, true));
    }
    public static Command autoDefault(){
        return Commands.sequence(
            TrajectoryReader.readTrajectory(defaultAuto, true)
        );
    }
*/

    // =============================== AUTOS FINALES ======================================== //
    public static Command fourNoteCenter() {
        return AutoBuilder.buildAuto("123OnCenterShootOnWingNoCurve");
    }

    public static Command fourNoteCenterReturning() {
        return AutoBuilder.buildAuto("123OnCenter");
    }

    public static Command threeNoteCenter() {
        return AutoBuilder.buildAuto("1and2OnCenter");
    }

    public static Command leaveAndTakeOut() {
        return AutoBuilder.buildAuto("overture");
    }

    public static Command TwoNoteCenter() {
        return AutoBuilder.buildAuto("centerShootSimple");
    }

    
    // ============================== AUTOS DE PRUEBA =================
    public static Command lengthTest() {
        return AutoBuilder.buildAuto("oneMeter");
    }

    public static Command timeTest() {
        return AutoBuilder.buildAuto("timingPrueba");
    }   

  

    public static Command OneAnd2OnLeft(){
        return AutoBuilder.buildAuto("1and2OnLeft");
    }
    



    
}