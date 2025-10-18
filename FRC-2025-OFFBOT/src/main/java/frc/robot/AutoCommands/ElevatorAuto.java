package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Mechanisms.Elevator.ElevatorSub;
import frc.robot.Mechanisms.Elevator.ElevatorSub.RequestType;
import frc.robot.Mechanisms.Outake.OutakeSub;
import frc.robot.Mechanisms.Outake.OutakeSub.OutakeRequestType;

public class ElevatorAuto extends Command{

    private ElevatorSub elevator;
    private OutakeSub outake;
    private double setpoint;
    private double targetOutake;

   

    public ElevatorAuto (ElevatorSub elevator, OutakeSub outake, double setpoint, double targetOutake){

        this.elevator = elevator;
        this.outake = outake;
        this.setpoint = setpoint;
        this.targetOutake = targetOutake;


    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){

        outake.stopwheelsOutake();
        
        elevator.setPosition(-setpoint, RequestType.kUP);
        outake.setPosition(targetOutake, OutakeRequestType.kUp);



    }


    @Override
    public void end(boolean interrupted) {
       
    }

    @Override
    public boolean isFinished(){     
        return elevator.atGoal() && outake.atGoal();
    }
    
}
