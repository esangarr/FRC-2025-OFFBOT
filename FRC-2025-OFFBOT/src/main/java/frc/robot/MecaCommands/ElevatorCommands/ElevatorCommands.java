package frc.robot.MecaCommands.ElevatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Elevator.ElevatorSub;
import frc.robot.Mechanisms.Elevator.ElevatorSub.RequestType;

public class ElevatorCommands {

    public static Command runManual(ElevatorSub Elev, DoubleSupplier joystick){

        return Commands.run(()->{ Elev.runMot(joystick.getAsDouble());},Elev).finallyDo(()->{Elev.StopMotors();});

    }

    public static Command setVol(ElevatorSub Elev, double voltage){
        return Commands.run(()->{ Elev.setVoltage(voltage);}, Elev).finallyDo(()->{Elev.StopMotors();});
    }

    public static Command setPos(ElevatorSub Elev, double setpoint){

        return Commands.run(()->{ Elev.setPosition(-setpoint, RequestType.kUP);},Elev).finallyDo(()->{Elev.StopMotors();});

    }




    
}
