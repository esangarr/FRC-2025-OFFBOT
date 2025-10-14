package frc.robot.MecaCommands.ElevatorCommands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Elevator.ElevatorSub;
import frc.robot.Mechanisms.Elevator.ElevatorSub.RequestType;
import frc.robot.Mechanisms.Indexer.IndexerSub;
import frc.robot.Mechanisms.Outake.OutakeSub;
import frc.robot.Mechanisms.Outake.OutakeSub.OutakeRequestType;


public class ElevatorCommands {

    public static Command runManual(ElevatorSub Elev, DoubleSupplier joystick){

        return Commands.run(()->{ Elev.runMot(joystick.getAsDouble());},Elev).finallyDo(()->{Elev.StopMotors();});

    }

    public static Command setVol(ElevatorSub Elev, double voltage){
        return Commands.run(()->{ Elev.setVoltage(voltage);}, Elev).finallyDo(()->{Elev.StopMotors();});
    }
    public static Command scoreCoral(
        ElevatorSub Elev, 
        OutakeSub outake,
        IndexerSub index,
        double setpoint,
        double targetOutake){

        return Commands.run(()->{ 
            Elev.setPosition(-Elev.metersToRot(63), RequestType.kDown);
            outake.runWheelsOutake(-0.6);
            outake.setPosition(1, OutakeRequestType.KDown);//--------------------------------

        },Elev, outake, index).until(()-> !index.hasPiece())

        .andThen(
            Commands.sequence(
            Commands.run(()-> {
                outake.stopwheelsOutake(); 
                Elev.setPosition(-setpoint, RequestType.kUP);}, Elev, outake)).until(()-> Elev.atGoal()),
            Commands.run(()-> {outake.setPosition(targetOutake, OutakeRequestType.kUp);} )
        
        .andThen(Commands.run(()-> {
            outake.setPosition(targetOutake, OutakeRequestType.kUp);
        }, Elev, outake, index)).
        
        finallyDo(()->{
            Elev.StopMotors();
        }));

    }

    public static Command GetAlgae(
        ElevatorSub Elev, 
        OutakeSub outake,
        double setpoint,
        double targetOutake, 
        double speed){

        return Commands.run(()->{ 

            Elev.setPosition(-setpoint, RequestType.kUP);

        },Elev, outake).until(()-> Elev.atGoal())

            
            .andThen(Commands.parallel(
                Commands.run(()-> {outake.setPosition(targetOutake, OutakeRequestType.kUp);
                }),
                Commands.sequence(
                    Commands.run(() -> outake.runWheelsOutake(speed), outake).withTimeout(0.1),
                    Commands.run(() -> outake.stopwheelsOutake(), outake).withTimeout(0.1)).repeatedly()));

        

    }


    public static Command setPosDown(ElevatorSub Elev, OutakeSub outake, double setpoint, double targetOutake){

        return Commands.run(()->{ 
            Elev.setPosition(-setpoint, RequestType.kDown);
            outake.setPosition(targetOutake, OutakeRequestType.KDown);
        },Elev, outake).finallyDo(()->{
            outake.stopALL();
            Elev.StopMotors();
            });

    }





    
}
