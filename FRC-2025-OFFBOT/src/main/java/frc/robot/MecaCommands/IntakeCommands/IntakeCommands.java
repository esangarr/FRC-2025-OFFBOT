package frc.robot.MecaCommands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Elevator.ElevatorSub;
import frc.robot.Mechanisms.Elevator.ElevatorSub.RequestType;
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

    public static Command runIntakeManual(IntakeSub intake, DoubleSupplier joystick){
        return Commands.run(()->{
            intake.runIntake(joystick.getAsDouble());
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

            .until(() -> timer.calculate(index.getCurrentRight() > 26 || index.getCurrentLeft() > 26))
            
            .andThen(Commands.run(() -> {

                if (index.getCurrentRight() >= 26){
                    intake.runWheelsIntake(-intakeSpeed/2);
                    index.runWheels(-rightSpeed, leftSpeed);}

                if (index.getCurrentLeft() >= 26){
                    intake.runWheelsIntake(-intakeSpeed/2);
                    index.runWheels(rightSpeed, -leftSpeed);}

                }, intake, index).withTimeout(0.33).

                andThen(Commands.run(() -> {
                    intake.runWheelsIntake(intakeSpeed);
                    index.runWheels(rightSpeed, leftSpeed);
                }, intake, index)))
                

                .finallyDo(()->{
                    index.stop();
                    intake.stopWheelsIntake();

                   ;});
                    
        }

        public static Command clearPieceComplete(
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
    
                .until(() -> timer.calculate(index.getCurrentRight() > 24 || index.getCurrentLeft() > 14))
                
                .andThen(Commands.run(() -> {
    
    
                    if (index.getCurrentRight() >= 24){
                        intake.runWheelsIntake(-intakeSpeed/2);
                        index.runWheels(-rightSpeed, leftSpeed);}
    
                    if (index.getCurrentLeft() >= 24){
                        intake.runWheelsIntake(-intakeSpeed/2);
                        index.runWheels(rightSpeed, -leftSpeed);}
    
                    }, intake, index).withTimeout(0.35).
    
                    andThen(Commands.run(() -> {
   
                        intake.runWheelsIntake(intakeSpeed);
                        index.runWheels(rightSpeed, leftSpeed);
                    }, intake, index)))
                    
                    .until(()-> index.hasPiece())
                    
                    .andThen(Commands.run(()-> {
  
                        index.stop();
                        intake.stopWheelsIntake();
                        intake.setPositionUp(angleUp);
    
                    })
    
                    .finallyDo(()->{
      
                        elevator.setPosition(-elevator.metersToRot(80), RequestType.kUP);
                        intake.stopAll(); 
                        index.stop();
                        outake.stopALL();
                       ;}));
                        
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
