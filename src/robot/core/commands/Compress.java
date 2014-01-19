/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package robot.core.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.core.Robot;
/**
 *
 * @author Westmont Robotics
 */
public class Compress extends Command {
    
    public Compress() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.pneumatics);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.pneumatics.startCompressorLoop();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.pneumatics.stopCompressorLoop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
