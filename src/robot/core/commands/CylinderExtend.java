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
public class CylinderExtend extends Command {
    
    public CylinderExtend() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if(Robot.oi.toggleCylinder.get()) {
            Robot.pneumatics.extend();
            System.out.println("Cylinder extended");
        } else {
            Robot.pneumatics.retract();
            System.out.println("Cylinder retracted");
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.pneumatics.retract();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
