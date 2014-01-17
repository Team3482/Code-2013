/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package robot.core.commands;
import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.core.Robot;

/**
 *
 * @author Developer
 */
public class ArmDown extends CommandGroup {
    
    public ArmDown() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.chassis);
        addParallel(new Drive());
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.chassis.moveArm(-0.25);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !Robot.oi.armDownButton.get();
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.chassis.moveArm(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
