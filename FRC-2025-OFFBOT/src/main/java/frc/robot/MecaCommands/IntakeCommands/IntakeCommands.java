package frc.robot.MecaCommands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Indexer.IndexerSub;
//import frc.robot.Mechanisms.Indexer.IndexerSub;
import frc.robot.Mechanisms.Intake.IntakeSub;

public class IntakeCommands {

    public static Command setAngleUp(IntakeSub intake, double angle){

        return Commands.run(()-> {intake.setPositionUp(angle);},intake).finallyDo(()->{intake.stopAng();});
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
        double intakeSpeed,
        double rightSpeed,
        double leftSpeed
        ){ 

            return Commands.run(() -> {
                intake.runWheelsIntake(intakeSpeed);
                index.runWheels(rightSpeed, leftSpeed);
            }, intake, index)

            .until(() -> index.getVoltageRight() <= 7.6 || index.getVoltageLeft() <= 7.6)
            
            .andThen(Commands.run(() -> {

                    intake.runWheelsIntake(-intakeSpeed/2);
                    index.runWheels(-rightSpeed, -leftSpeed);

                }, intake, index).withTimeout(1).
                
                andThen(
                Commands.run(() -> {
                    intake.runWheelsIntake(intakeSpeed);
                    index.runWheels(rightSpeed, leftSpeed);
                }, intake, index)));
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
            }, intake, index).finallyDo(()->{index.stop();});
        }
    
    public static Command intakePiece(
        IntakeSub intake,
        IndexerSub index,   
        double wheelsIntake,
        double rightSpeed,
        double leftSpeed){

            return Commands.run(()->{
                intake.runWheelsIntake(wheelsIntake);
                index.runWheels(rightSpeed, leftSpeed);

          },intake, index).finallyDo(()-> {

            intake.stopAll();
            index.stop();});
    }

    public static Command eatPieceNoBeam(
        IntakeSub intake,
        IndexerSub index,
        double angle,
        double wheelsIntake,
        double rightSpeed,
        double leftSpeed){

            return Commands.run(()->{
                intake.setPositionDown(angle);
                intake.runWheelsIntake(wheelsIntake);
                index.runWheels(rightSpeed, leftSpeed);

          },intake, index).finallyDo(()-> {

            intake.stopAll();
            index.stop();});
    }

    public static Command eatPiece(
        IntakeSub intake,
        IndexerSub index,
        double eatAngle,
        double retractAngle,
        double wheelsIntake,
        double rightSpeed,
        double leftSpeed ){

            return Commands.run(()->{
                intake.setPositionDown(eatAngle);

              if (index.hasPiece() == false){
                    intake.runWheelsIntake(wheelsIntake);
                  index.runWheels(rightSpeed, leftSpeed);
                }else{
                    intake.stopWheelsIntake();
                    index.stop();
                    intake.setPositionUp(retractAngle);
                }

            }, intake, index).finallyDo(()->{intake.stopAll(); index.stop();});
    }





}
