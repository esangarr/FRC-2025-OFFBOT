package frc.robot.MecaCommands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Climber.ClimberSub;

public class ClimberCommands {
    public static Command angleClimber(ClimberSub climber, double speed){

        return Commands.run(()-> {climber.AngulateClimber(speed);},climber).finallyDo(()->{climber.StopAng();});
    }
}
