package frc.robot.MecaCommands.ElevatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Elevator.ElevatorSub;
import frc.robot.Mechanisms.Elevator.ElevatorSub.RequestType;
import frc.robot.Mechanisms.Outake.OutakeSub;

public class ElevatorCommands {

    public static Command runManual(ElevatorSub Elev, DoubleSupplier joystick){

        return Commands.run(()->{ Elev.runMot(joystick.getAsDouble());},Elev).finallyDo(()->{Elev.StopMotors();});

    }

    public static Command setVol(ElevatorSub Elev, double voltage){
        return Commands.run(()->{ Elev.setVoltage(voltage);}, Elev).finallyDo(()->{Elev.StopMotors();});
    }

    public static Command setPosUp(ElevatorSub Elev, OutakeSub outake, double setpoint, double targetOutake){

        return Commands.run(()->{ 
            Elev.setPosition(-setpoint, RequestType.kUP);
            outake.setPositionUp(targetOutake);
            
        },Elev, outake).finallyDo(()->{Elev.StopMotors();});

    }

    public static Command setPosDown(ElevatorSub Elev, OutakeSub outake, double setpoint, double targetOutake){

        return Commands.run(()->{ 
            Elev.setPosition(-setpoint, RequestType.kDown);
            outake.setPositionUp(targetOutake);
        },Elev, outake).finallyDo(()->{Elev.StopMotors();});

    }




    
}
