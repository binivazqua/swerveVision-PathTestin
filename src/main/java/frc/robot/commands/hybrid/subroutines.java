package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.NamedCommands;  

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.limelightConstants.aprilTag;
import frc.robot.commands.Mecanismos.IntakeButtonCmd;
import frc.robot.commands.Mecanismos.ShooterButtonCmd;
import frc.robot.commands.swerve.autoAlign;

public class subroutines {

    public subroutines(){
        NamedCommands.registerCommand("ShootWithDelay", shootWithDelay());
    }

     public static Command shootWithDelay() {
        return new SequentialCommandGroup(
            new ShooterButtonCmd(0.75).withTimeout(1),
            new ParallelCommandGroup(
                new ShooterButtonCmd(0.75),
                new IntakeButtonCmd(-0.5)).withTimeout(2));
    
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
