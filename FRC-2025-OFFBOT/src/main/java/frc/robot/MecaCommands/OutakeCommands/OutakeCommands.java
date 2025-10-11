package frc.robot.MecaCommands.OutakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Outake.OutakeSub;

public class OutakeCommands {

    public static Command armManual(OutakeSub outake, DoubleSupplier joystick){

        return Commands.run(()-> {outake.runArm(joystick.getAsDouble());},outake).finallyDo(()->{outake.stopArm();});
    }

    public static Command setAngleUp(OutakeSub outake, double angle){

        return Commands.run(()-> {
            outake.setPositionUp(angle);
        },outake)
        .finallyDo(()->{
            outake.stopArm();
        });
    }

    public static Command setAngleDown(OutakeSub outake, double angle){

        return Commands.run(()-> {
            outake.setPositionDown(angle);
        },outake).finallyDo(()->{
            outake.stopArm();
        });
    }

    public static Command resetEncoder(OutakeSub outake){
        return Commands.run(()-> {
            outake.resetEconder();
        }, outake );
    }

    public static Command moveWheels(OutakeSub outake, double speed){
        return Commands.run(()-> {
            outake.runWheelsOutake(speed);
        }, outake ).finallyDo(()-> {outake.stopwheelsOutake();});
    }

    public static Command shootDunk(OutakeSub outake, double speed){
        return Commands.run(()-> {
            outake.setPositionDown(outake.currentSetpoint() - 30);
            outake.runWheelsOutake(speed);
        }, outake ).finallyDo(()-> {outake.stopALL();});    
    }



    public static Command AlgaeWheels(OutakeSub outake, double speed){
        return Commands.sequence(
            Commands.run(() -> outake.runWheelsOutake(speed), outake).withTimeout(0.1),
            Commands.run(() -> outake.stopwheelsOutake(), outake).withTimeout(0.1)
        ).repeatedly();

        
    }



    
}
