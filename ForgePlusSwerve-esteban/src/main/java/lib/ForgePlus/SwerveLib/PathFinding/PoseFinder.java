package lib.ForgePlus.SwerveLib.PathFinding;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lib.ForgePlus.Equals.TwoDimensionalSpace;
import lib.ForgePlus.SwerveLib.Odometer.FieldObject;
import lib.ForgePlus.SwerveLib.Odometer.OdometerUtils;

/**
 * The {@code PoseFinder} class manages swerve drive pathfinding to a target pose using PathPlanner.
 * It integrates with WPILib’s command-based framework and provides conditional, parallel, and deadline commands.
 */
public class PoseFinder implements Sendable{

    private Subsystem swerveSub;
    private Pose2d goal;
    private Supplier<Pose2d> swervePose;
    private Consumer<Pose2d> poseReseter;
    private Consumer<ChassisSpeeds> runVelocity;
    private FieldObject field;
    private PathConstraints constraints;
    private boolean currentState = false;
    private double errorTolerance = 0;

    /**
     * Constructs a {@code PoseFinder} instance for autonomous navigation.
     *
     * @param field The field object for flipping poses based on alliance color.
     * @param finderConstraints The default path constraints.
     * @param setVelocity A consumer to set the robot's velocity.
     * @param getPose A supplier to retrieve the current robot pose.
     * @param setPose A consumer to reset the robot's pose.
     * @param errorTolerance The allowable position tolerance for reaching the goal.
     * @param holonomicDriveTrain The swerve subsystem, required for command-based operations.
     */
    public PoseFinder(FieldObject field, PathConstraints finderConstraints, Consumer<ChassisSpeeds> setVelocity, Supplier<Pose2d> getPose, Consumer<Pose2d> setPose, double errorTolerance, Subsystem holonomicDriveTrain){
        this.goal = new Pose2d();
        this.swerveSub = holonomicDriveTrain;
        this.swervePose = getPose;
        this.poseReseter = setPose;
        this.runVelocity = setVelocity;
        this.field = field;
        this.constraints = finderConstraints;
        this.currentState = false;
        this.errorTolerance = errorTolerance;

        PathfindingCommand.warmupCommand().schedule();

    }

    @Override
    public void initSendable(SendableBuilder builder){

        builder.setSmartDashboardType("PoseFinder");

        builder.addDoubleArrayProperty("Objective Pose",
        ()-> new double[]{goal.getX(), goal.getY(), goal.getRotation().getDegrees()}, null);

        builder.addBooleanProperty("PoseReached", ()-> atGoal(), null);

        builder.addDoubleProperty("PercentageTolerance", ()-> errorTolerance, null);

        builder.addBooleanProperty("IsSwervePathFinding", ()-> isPathFinding(), null);
        
    }

    /**
     * @return The position tolerance for determining if the robot has reached its goal.
     */
    public double getPositionTolerance(){
        return errorTolerance;
    }

    /**
     * Sets the position tolerance for determining goal completion.
     *
     * @param newTolerance The new error tolerance.
     */
    public void setPathFindingTolerance(double newTolerance){
        this.errorTolerance = newTolerance;
    }


    /**
     * @return The current robot pose.
     */
    public Pose2d currentHolonomicPose(){
        return swervePose.get();
    }

    /**
     * @return True if the alliance is blue, false if red.
     */
    public boolean isBlueAlliance(){
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }

    /**
     * @return True if the robot is within the error tolerance of the goal.
     */
    public boolean atGoal(){
        return new TwoDimensionalSpace(goal.getTranslation(), errorTolerance).atSpace(swervePose.get());
    }


     /**
     * Loads a new pose as the robot’s current position.
     *
     * @param pose The pose to set.
     */
    public void loadPose(Pose2d pose){
        poseReseter.accept(pose);
    }

    /**
     * @param pose The pose to load.
     * @return A command that sets the robot’s pose once.
     */
    public Command loadPoseCommand(Pose2d pose){
        return Commands.runOnce(()-> loadPose(pose), swerveSub);
    }

    /**
     * Stops the robot by setting its velocity to zero.
     */
    public void stopHolonomic(){
        runVelocity.accept(new ChassisSpeeds());
    }

     /**
     * @return True if the robot is currently pathfinding, false otherwise.
     */
    public boolean isPathFinding(){
        return currentState;
    }

    private Command toRawPose(Pose2d pose, PathConstraints pathVel){

        Command pathFind = AutoBuilder.pathfindToPose(pose, pathVel).beforeStarting(()-> {

        this.goal = pose;
        this.currentState = true;
        }, swerveSub).onlyWhile(()-> !atGoal()).finallyDo(()-> {
            this.currentState = false;
            stopHolonomic();
        });

        pathFind.addRequirements(swerveSub);

        return pathFind;


    }

    /**
     * Generates a command to follow a path defined in a PathPlanner file.
     *
     * @param pathName The name of the path file.
     * @return A command that follows the specified path.
     */
    public Command toPathPlannerPath(String pathName){
        try{

            Command pathFind = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(pathName), constraints).
            beforeStarting(()-> {
                this.currentState = true;
            }, swerveSub).onlyWhile(()-> !atGoal()).finallyDo(()-> {
                this.currentState = false;
                stopHolonomic();
            });

            return pathFind;
        } catch (Exception e){
            System.out.println("Path not found: " + pathName);
            return Commands.none();
        }
    }

    /**
     * Generates a command to move to the specified pose.
     *
     * @param objective The target pose.
     * @return A ConditionalCommand that adjusts for alliance color.
     */
    public ConditionalCommand toPoseCommand(Pose2d objective){
        return new ConditionalCommand(
            toRawPose(objective, constraints),
            toRawPose(OdometerUtils.flipPose(objective, field), constraints),
            ()-> isBlueAlliance());

    }

    /**
     * Generates a command to move to the specified pose with new constraints.
     *
     * @param objective The target pose.
     * @param newConstraints The path constraints.
     * @return A ConditionalCommand that adjusts for alliance color.
     */
    public ConditionalCommand toPoseCommand(Pose2d objective, PathConstraints newConstraints){
        return new ConditionalCommand(
            toRawPose(objective, newConstraints),
            toRawPose(OdometerUtils.flipPose(objective, field), newConstraints),
            ()-> isBlueAlliance());
    }

    /**
     * Moves to a pose in parallel with another command.
     *
     * @param objective The target pose.
     * @param other The other command to run in parallel.
     * @return A ParallelCommandGroup.
     */
    public ParallelCommandGroup toPoseParallelCommand(Pose2d objective, Command other){
        return new ParallelCommandGroup(toPoseCommand(objective), other);
    }

    /**
     * Moves to a pose in parallel with another command.
     *
     * @param objective The target pose.
     * @param other The other command to run in parallel.
     * @param newConstraints The path constraints.
     * @return A ParallelCommandGroup.
     */
    public ParallelCommandGroup toPoseParallelCommand(Pose2d objective, Command other, PathConstraints newConstraints){
        return new ParallelCommandGroup(toPoseCommand(objective, newConstraints), other);
    }

    /**
     * Moves to a pose in deadline with another command.
     *
     * @param objective The target pose.
     * @param other The other command to run.
     * @return A DeadlineCommandGroup.
     */
    public ParallelDeadlineGroup toPoseDeadlineCommand(Pose2d objective, Command other){
        return new ParallelDeadlineGroup(toPoseCommand(objective), other);
    }

    /**
     * Moves to a pose in deadline with another command.
     *
     * @param objective The target pose.
     * @param other The other command to run.
     * @param newConstraints The path constraints.
     * @return A DeadlineCommandGroup.
     */
    public ParallelDeadlineGroup toPoseDeadlineCommand(Pose2d objective, Command other, PathConstraints newConstraints){
        return new ParallelDeadlineGroup(toPoseCommand(objective, newConstraints), other);
    }

    /**
     * Creates a command that conditionally moves to one of two poses.
     *
     * @param pose The first pose option.
     * @param otherPose The second pose option.
     * @param condition A BooleanSupplier determining which pose to choose.
     * @return A ConditionalCommand that moves to the selected pose.
     */
    public ConditionalCommand toIFPoseCommand(Pose2d pose, Pose2d otherPose, BooleanSupplier condition){
        return new ConditionalCommand(toPoseCommand(pose), toPoseCommand(otherPose), condition);
    }

    /**
     * Creates a command that conditionally moves to one of two poses with a specified path constraint.
     *
     * @param pose The first pose option.
     * @param otherPose The second pose option.
     * @param condition A BooleanSupplier determining which pose to choose.
     * @param newConstraints The path constraints to apply.
     * @return A ConditionalCommand that moves to the selected pose with constraints.
     */
    public ConditionalCommand toIFPoseCommand(Pose2d pose, Pose2d otherPose, BooleanSupplier condition, PathConstraints newConstraints){
        return new ConditionalCommand(toPoseCommand(pose, newConstraints), toPoseCommand(otherPose, newConstraints), condition);
    }

    /**
     * Creates a command that conditionally moves to one of two poses with different path constraints for each.
     *
     * @param pose The first pose option.
     * @param otherPose The second pose option.
     * @param condition A BooleanSupplier determining which pose to choose.
     * @param firstPoseConstraints The path constraints for the first pose.
     * @param secondPoseConstraints The path constraints for the second pose.
     * @return A ConditionalCommand that moves to the selected pose with respective constraints.
     */
    public ConditionalCommand toIFPoseCommand(Pose2d pose, Pose2d otherPose, BooleanSupplier condition, PathConstraints firstPoseConstraints, PathConstraints secondPoseConstraints){
        return new ConditionalCommand(toPoseCommand(pose, firstPoseConstraints), toPoseCommand(otherPose, secondPoseConstraints), condition);
    }

    /**
     * Creates a parallel command that moves to one of two poses while running another command simultaneously.
     *
     * @param pose The first pose option.
     * @param otherPose The second pose option.
     * @param condition A BooleanSupplier determining which pose to choose.
     * @param other The command to run in parallel.
     * @return A ParallelCommandGroup executing pose movement and the other command together.
     */
    public ParallelCommandGroup toIFPoseParallelCommand(Pose2d pose, Pose2d otherPose, BooleanSupplier condition, Command other){
        return new ParallelCommandGroup(toIFPoseCommand(pose, otherPose, condition), other);
    }

    /**
     * Creates a parallel command that moves to one of two poses with specified path constraints while running another command.
     *
     * @param pose The first pose option.
     * @param otherPose The second pose option.
     * @param condition A BooleanSupplier determining which pose to choose.
     * @param other The command to run in parallel.
     * @param newConstraints The path constraints to apply.
     * @return A ParallelCommandGroup executing pose movement with constraints and the other command together.
     */
    public ParallelCommandGroup toIFPoseParallelCommand(Pose2d pose, Pose2d otherPose, BooleanSupplier condition, Command other, PathConstraints newConstraints){
        return new ParallelCommandGroup(toIFPoseCommand(pose, otherPose, condition, newConstraints), other);
    }

    /**
     * Creates a parallel command that moves to one of two poses with different path constraints for each while running another command.
     *
     * @param pose The first pose option.
     * @param otherPose The second pose option.
     * @param condition A BooleanSupplier determining which pose to choose.
     * @param other The command to run in parallel.
     * @param firstPoseConstraints The path constraints for the first pose.
     * @param secondPoseConstraints The path constraints for the second pose.
     * @return A ParallelCommandGroup executing pose movement with respective constraints and the other command together.
     */
    public ParallelCommandGroup toIFPoseParallelCommand(Pose2d pose, Pose2d otherPose, BooleanSupplier condition, Command other, PathConstraints firstPoseConstraints, PathConstraints secondPoseConstraints){
        return new ParallelCommandGroup(toIFPoseCommand(pose, otherPose, condition, firstPoseConstraints, secondPoseConstraints), other);
    }

    /**
     * Creates a deadline command that moves to one of two poses while running another command, which terminates when the pose movement completes.
     *
     * @param pose The first pose option.
     * @param otherPose The second pose option.
     * @param condition A BooleanSupplier determining which pose to choose.
     * @param other The command that runs alongside but stops when the pose movement completes.
     * @return A ParallelDeadlineGroup where pose movement dictates when execution stops.
     */
    public ParallelDeadlineGroup toIFPoseDeadlineCommand(Pose2d pose, Pose2d otherPose, BooleanSupplier condition, Command other){
        return new ParallelDeadlineGroup(toIFPoseCommand(pose, otherPose, condition), other);
    }

    /**
     * Creates a deadline command that moves to one of two poses with specified path constraints while running another command.
     *
     * @param pose The first pose option.
     * @param otherPose The second pose option.
     * @param condition A BooleanSupplier determining which pose to choose.
     * @param other The command that runs alongside but stops when the pose movement completes.
     * @param newConstraints The path constraints to apply.
     * @return A ParallelDeadlineGroup where pose movement with constraints dictates when execution stops.
     */
    public ParallelDeadlineGroup toIFPoseDeadlineCommand(Pose2d pose, Pose2d otherPose, BooleanSupplier condition, Command other, PathConstraints newConstraints){
        return new ParallelDeadlineGroup(toIFPoseCommand(pose, otherPose, condition, newConstraints), other);
    }

    /**
     * Creates a deadline command that moves to one of two poses with different path constraints for each while running another command.
     *
     * @param pose The first pose option.
     * @param otherPose The second pose option.
     * @param condition A BooleanSupplier determining which pose to choose.
     * @param other The command that runs alongside but stops when the pose movement completes.
     * @param firstPoseConstraints The path constraints for the first pose.
     * @param secondPoseConstraints The path constraints for the second pose.
     * @return A ParallelDeadlineGroup where pose movement with respective constraints dictates when execution stops.
     */
    public ParallelDeadlineGroup toIFPoseDeadlineCommand(Pose2d pose, Pose2d otherPose, BooleanSupplier condition, Command other, PathConstraints firstPoseConstraints, PathConstraints secondPoseConstraints){
        return new ParallelDeadlineGroup(toIFPoseCommand(pose, otherPose, condition, firstPoseConstraints, secondPoseConstraints), other);
    }

    /**
     * Moves to the nearest pose in the list.
     *
     * @param poses The list of possible target poses.
     * @return A ConditionalCommand to move to the nearest pose.
     */
    public ConditionalCommand toNearestPoseCommand(List<Pose2d> poses){
        return toPoseCommand(swervePose.get().nearest(poses));
    }

    /**
     * Moves to the nearest pose in the list with specified constraints.
     *
     * @param poses The list of possible target poses.
     * @param newConstraints The path constraints.
     * @return A ConditionalCommand to move to the nearest pose.
     */
    public ConditionalCommand toNearestPoseCommand(List<Pose2d> poses, PathConstraints newConstraints){
        return toPoseCommand(swervePose.get().nearest(poses), newConstraints);
    }

    /**
     * Creates a parallel command that moves to the nearest pose while running another command simultaneously.
     *
     * @param poses The list of possible target poses.
     * @param other The command to run in parallel.
     * @return A ParallelCommandGroup executing both pose movement and the other command together.
     */
    public ParallelCommandGroup toNearestPoseParallelCommand(List<Pose2d> poses, Command other){
        return new ParallelCommandGroup(toNearestPoseCommand(poses), other);
    };

    /**
     * Creates a parallel command that moves to the nearest pose with specified constraints while running another command.
     *
     * @param poses The list of possible target poses.
     * @param other The command to run in parallel.
     * @param newConstraints The path constraints to apply.
     * @return A ParallelCommandGroup executing both pose movement with constraints and the other command together.
     */
    public ParallelCommandGroup toNearestPoseParallelCommand(List<Pose2d> poses, Command other, PathConstraints newConstraints){
        return new ParallelCommandGroup(toNearestPoseCommand(poses, newConstraints), other);
    };

    /**
     * Creates a deadline command that moves to the nearest pose while running another command,
     * stopping when the pose movement completes.
     *
     * @param poses The list of possible target poses.
     * @param other The command that runs alongside but stops when the pose movement completes.
     * @return A ParallelDeadlineGroup where pose movement dictates when execution stops.
     */
    public ParallelDeadlineGroup toNearestPoseDeadlineCommand(List<Pose2d> poses, Command other){
        return new ParallelDeadlineGroup(toNearestPoseCommand(poses), other);
    }

    /**
     * Creates a deadline command that moves to the nearest pose with specified constraints while running another command.
     * The command stops when the pose movement completes.
     *
     * @param poses The list of possible target poses.
     * @param other The command that runs alongside but stops when the pose movement completes.
     * @param newConstraints The path constraints to apply.
     * @return A ParallelDeadlineGroup where pose movement with constraints dictates when execution stops.
     */
    public ParallelDeadlineGroup toNearestPoseDeadlineCommand(List<Pose2d> poses, Command other, PathConstraints newConstraints){
        return new ParallelDeadlineGroup(toNearestPoseCommand(poses, newConstraints), other);
    }

}
