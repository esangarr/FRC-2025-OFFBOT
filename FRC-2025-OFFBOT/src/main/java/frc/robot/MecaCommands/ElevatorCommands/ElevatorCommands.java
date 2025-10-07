package frc.robot.MecaCommands.ElevatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Elevator.ElevatorSub;

public class ElevatorCommands {

    public static Command runManual(ElevatorSub Elev, DoubleSupplier joystick){

        return Commands.run(()->{ Elev.runMot(joystick.getAsDouble());},Elev).finallyDo(()->{Elev.StopMotors();});

    }




    
}
