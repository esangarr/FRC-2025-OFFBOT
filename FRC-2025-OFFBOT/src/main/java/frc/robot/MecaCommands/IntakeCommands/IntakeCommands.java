package frc.robot.MecaCommands.IntakeCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Elevator.ElevatorSub;
import frc.robot.Mechanisms.Indexer.IndexerSub;
import frc.robot.Mechanisms.Intake.IntakeSub;
import frc.robot.Mechanisms.Outake.OutakeSub;

public class IntakeCommands {

    public static Command setAngleUp(IntakeSub intake, double angle){

        return Commands.run(()-> {intake.setPositionUp(angle);},intake);
    }

    public static Command setVoltageCommand(IntakeSub intake, double voltage){

        return Commands.run(()-> {intake.setVoltage(voltage);},intake).finallyDo(()->{intake.stopAng();});
    }



    public static Command setAngleDown(IntakeSub intake, double angle){

        return Commands.run(()-> {intake.setPositionDown(angle);},intake).finallyDo(()->{intake.stopAll();});
    }

    public static Command runIntakeManual(IntakeSub intake, double joystick){
        return Commands.run(()->{
            intake.runIntake(joystick);
        }, intake).finallyDo(()->{intake.stopAng();});
    }

    public static Command runIntakeWheels(IntakeSub intake, double speed){
        return Commands.run(()-> {
            intake.runWheelsIntake(speed);
        },intake).finallyDo(()->{intake.stopWheelsIntake();});
    }

    public static Command clearPiece(
        IntakeSub intake, 
        IndexerSub index, 
        ElevatorSub elevator,
        OutakeSub outake,
        double intakeSpeed,
        double rightSpeed,
        double leftSpeed, 
        Debouncer timer, 
        double angleUp, 
        double angleDown
        ){ 

            return Commands.run(() -> {
                intake.runWheelsIntake(intakeSpeed);
                index.runWheels(rightSpeed, leftSpeed);
            }, intake, index)

            .until(() -> timer.calculate(index.getCurrentRight() > 30 || index.getCurrentLeft() > 30))
            
            .andThen(Commands.run(() -> {

                if (index.getCurrentRight() >= 25){
                    intake.runWheelsIntake(-0.3);
                    index.runWheels(-rightSpeed, leftSpeed);}

                if (index.getCurrentLeft() >= 25){
                    intake.runWheelsIntake(-0.3);
                    index.runWheels(rightSpeed, -leftSpeed);}

                }, intake, index).withTimeout(0.5).

                andThen(Commands.run(() -> {
                    intake.runWheelsIntake(intakeSpeed);
                    index.runWheels(rightSpeed, leftSpeed);
                }, intake, index)))
                

                .finallyDo(()->{
                    index.stop();
                    intake.stopWheelsIntake();

                   ;});
                    
        }

       

    public static Command outPiece(
        IntakeSub intake, 
        IndexerSub index, 
        double intakeSpeed,
        double angleUp,
        double indexSpeed){
            return Commands.run(()->{
                intake.setPositionUp(angleUp);
                intake.runWheelsIntake(-intakeSpeed);
                index.runWheels(-indexSpeed, -indexSpeed);
            }, intake, index).until(()-> intake.atGoal())
                .finallyDo(()->{index.stop(); intake.stopAll(); index.stop();});
        }

}
