package frc.robot.MecaCommands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Mechanisms.Elevator.ElevatorSub;
import frc.robot.Mechanisms.Indexer.IndexerSub;
import frc.robot.Mechanisms.Outake.OutakeSub;
import lib.ForgePlus.REV.REVBlinkin.REVBlinkin.PatternType;

public class ScoreCoralCommand extends Command {

    private ElevatorSub elevator;
    private OutakeSub outake;
    private IndexerSub index;
    private double setpoint;
    private double targetOutake;

    public ScoreCoralCommand (ElevatorSub elevator, OutakeSub outake, IndexerSub index, double setpoint, double targetOutake){
        this.elevator = elevator;
        this.outake = outake;
        this.index = index;

        this.setpoint = setpoint;
        this.targetOutake = targetOutake;

    }
    
       @Override
    public void initialize () {
     
    }

    @Override
    public void execute (){

        
        
    }

    @Override
    public void end (boolean interrupted) {}
      

    @Override
    public boolean isFinished(){
        return false;
    }

}
