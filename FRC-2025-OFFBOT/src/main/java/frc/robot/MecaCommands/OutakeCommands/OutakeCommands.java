package frc.robot.MecaCommands.OutakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Outake.OutakeSub;
import frc.robot.Mechanisms.Outake.OutakeSub.OutakeRequestType;

public class OutakeCommands {

    public static Command armManual(OutakeSub outake, DoubleSupplier joystick){

        return Commands.run(()-> {outake.runArm(joystick.getAsDouble());},outake).finallyDo(()->{outake.stopArm();});
    }

    public static Command setVoltage(OutakeSub outake, double voltage, double speed){
        return Commands.parallel(
            Commands.run(()-> {outake.setVoltage(voltage);}),
            Commands.sequence(
                Commands.run(() -> outake.runWheelsOutake(speed), outake).withTimeout(0.1),
                Commands.run(() -> outake.stopwheelsOutake(), outake).withTimeout(0.1)).repeatedly());
    }

    public static Command setAngleUp(OutakeSub outake, double angle){

        return Commands.run(()-> {
            outake.setPosition(angle, OutakeRequestType.kUp);
        },outake);
    }

    public static Command setAngleDown(OutakeSub outake, double angle){

        return Commands.run(()-> {
            outake.setPosition(angle, OutakeRequestType.KDown);
        },outake).finallyDo(()->{
            outake.stopArm();
        });
    }


    public static Command moveWheels(OutakeSub outake, double speed){
        return Commands.run(()-> {
            outake.runWheelsOutake(speed);
        }, outake ).finallyDo(()-> {outake.stopwheelsOutake();});
    }

    public static Command shootDunk(OutakeSub outake, double speed){
        double setDunk = outake.getSetpoint() - 0.85;
        return Commands.run(()-> {
            outake.setPosition(outake.DegreesToRotations(setDunk), OutakeRequestType.KDown);
            outake.runWheelsOutake(speed);
        }, outake ).finallyDo(()-> {outake.stopALL();});    
    }



    public static Command AlgaeWheels(OutakeSub outake, double speed){
        return Commands.sequence(
            Commands.run(() -> outake.runWheelsOutake(speed), outake).withTimeout(0.1),
            Commands.run(() -> outake.stopwheelsOutake(), outake).withTimeout(0.1)
        ).repeatedly();

        
    }

    public static Command resetArmEncoder(OutakeSub outake){

        return Commands.runOnce(()-> {outake.reset();}, outake);
    }



    
}
