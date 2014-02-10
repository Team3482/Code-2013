/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package robot.core.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.core.Robot;

public class Stop extends Command {
    
    public Stop() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.chassis.stop();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end(); 
   }
}
