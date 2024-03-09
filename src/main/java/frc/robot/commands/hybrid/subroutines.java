package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.NamedCommands;  

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.limelightConstants.aprilTag;
import frc.robot.commands.Mecanismos.IntakeButtonCmd;
import frc.robot.commands.Mecanismos.PivoteoCommand;
import frc.robot.commands.Mecanismos.ShooterButtonCmd;
import frc.robot.commands.swerve.autoAlign;
import frc.robot.subsystems.Mecanismos.Pivoteo;

public class subroutines {

    Pivoteo arm;

    public subroutines(){
        
        arm = Pivoteo.getInstance();

    }

     public static Command shootWithDelay() {
        return new SequentialCommandGroup(
            //cambio de .8 a .6  09/03 8:25
            new ShooterButtonCmd(0.6).withTimeout(1.3), // DELAY CHANGED FROM 1s to 0.8s // speed changed from 0.9 to 0.8 (04/03)
            new ParallelCommandGroup(
                new ShooterButtonCmd(0.8),
                new IntakeButtonCmd(-0.5))
                ).withTimeout(2); // 2 seg
    
    }

    
    public static Command shootWithDelayLejos() {
        return new SequentialCommandGroup(
            new ShooterButtonCmd(0.85).withTimeout(1.3), // DELAY CHANGED FROM 1s to 0.8s
            new ParallelCommandGroup(
                new ShooterButtonCmd(0.85),
                new IntakeButtonCmd(-0.5))
                ).withTimeout(2);
    
    }

    public static Command shootToAmp() {
        return new ParallelCommandGroup(
                new ShooterButtonCmd(0.225),
                new IntakeButtonCmd(-0.25)
                ).withTimeout(1.2);
    
    }

    public static Command lowArm() {
        return new SequentialCommandGroup(
            new PivoteoCommand(0.26),
            new PivoteoCommand(0.15)
        ).withTimeout(5);     
    }

    public static Command lowArmAndShoot() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
            new PivoteoCommand(0.23).withTimeout(1.3),
            new ShooterButtonCmd(0.9).withTimeout(1.3) // DELAY CHANGED FROM 1s to 0.8s
            ),
            new ParallelCommandGroup(
                new PivoteoCommand(0.14),
                shootWithDelay()
            )
        ).withTimeout(3);     
    }

   

   
    /*public static Command alignAndShit(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new autoAlign(aprilTag.constraints),
                new 
            )

        )
    }*/

    
}
