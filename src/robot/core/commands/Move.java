/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package robot.core.commands;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import robot.core.Robot;

/**
 *
 * @author Developer
 */
public class Move extends Command {
    private double  forward  = 0.0;
    private double  turn     = 0.0;
    private double  time     = 0.0;
    private boolean finished = false;
    public Move(double forward, double turn, double time) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.chassis);
        this.forward = forward;
        this.turn    = turn;
        this.time    = time;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.chassis.move(forward, turn);
        Timer.delay(time);
        Robot.chassis.move(0.0, 0.0);
        finished = true;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.chassis.move(0.0, 0.0);
    }
}
