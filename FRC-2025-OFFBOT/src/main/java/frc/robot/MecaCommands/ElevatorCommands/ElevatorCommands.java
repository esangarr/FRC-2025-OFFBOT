package frc.robot.MecaCommands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Elevator.ElevatorSub;

public class ElevatorCommands {

    public static Command MoveMotors (ElevatorSub Elev, double speed){

        return Commands.run(()->{ Elev.runMot(speed);},Elev).finallyDo(()->{Elev.StopMotors();});

    }




    
}
