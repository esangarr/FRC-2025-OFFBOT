package frc.robot.MecaCommands.ElevatorCommands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Elevator.ElevatorSub;
import frc.robot.Mechanisms.Elevator.ElevatorSub.RequestType;
import frc.robot.Mechanisms.Outake.OutakeSub;
import frc.robot.Mechanisms.Outake.OutakeSub.OutakeRequestType;


public class ElevatorCommands {

    public static Command runManual(ElevatorSub Elev, DoubleSupplier joystick){

        return Commands.run(()->{ Elev.runMot(joystick.getAsDouble());},Elev).finallyDo(()->{Elev.StopMotors();});

    }

    public static Command resetElevator(ElevatorSub Elev){

        return Commands.run(()->{ Elev.resetElev();},Elev);

    }

    public static Command setVol(ElevatorSub Elev, double voltage){
        return Commands.run(()->{ Elev.setVoltage(voltage);}, Elev).finallyDo(()->{Elev.StopMotors();});
    }

    public static Command takeCoral(
        ElevatorSub Elev, 
        OutakeSub outake, 
        double setpoint, 
        double targetOutake){
        return Commands.run(()->{ 
            Elev.setPosition(-Elev.metersToRot(63), RequestType.kDown);
            outake.runWheelsOutake(-0.6);
            outake.setPosition(outake.DegreesToRotations(2), OutakeRequestType.KDown);}, Elev).withTimeout(0.4)
            
            .andThen(Commands.run(()-> {
                outake.runWheelsOutake(0);     
                Elev.setPosition(-setpoint, RequestType.kDown);
                outake.setPosition(targetOutake, OutakeRequestType.KDown);
            }, outake, Elev));
                
            }

    public static Command scoreCoral2(
        ElevatorSub Elev, 
        OutakeSub outake,
        double setpoint,
        double targetOutake){

        return Commands.sequence(
            Commands.run(()-> {
                outake.stopwheelsOutake(); 
                Elev.setPosition(-setpoint, RequestType.kUP);}, Elev, outake).until(()-> Elev.atGoal()),

            Commands.run(()-> {outake.setPosition(targetOutake, OutakeRequestType.kUp);} )
        ).
        
        finallyDo(()->{ Elev.setPosition(-setpoint, RequestType.kUP);
            outake.setPosition(targetOutake, OutakeRequestType.kUp);});
        


    }

    public static Command GetAlgae(
        ElevatorSub Elev, 
        OutakeSub outake,
        double setpoint,
        double targetOutake, 
        double speed, 
        RequestType typeElev,
        OutakeRequestType typeOut
        ){

        return Commands.run(()->{ 

            Elev.setPosition(-setpoint, typeElev);

        },Elev, outake).until(()-> Elev.atGoal())

            
            .andThen(Commands.parallel(
                Commands.run(()-> {outake.setPosition(targetOutake, typeOut);
                }),
                Commands.sequence(
                    Commands.run(() -> outake.runWheelsOutake(speed), outake).withTimeout(0.12),
                    Commands.run(() -> outake.stopwheelsOutake(), outake).withTimeout(0.1)).repeatedly()))
                    .finallyDo(()-> { outake.stopArm();});

        

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
