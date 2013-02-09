// RobotBuilder Version: 0.0.2
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
package robot.core.subsystems;
import robot.core.RobotMap;
import robot.core.commands.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
/**
 * @author Westmont Robotics
 */
public class Chassis extends Subsystem {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SpeedController leftFront = RobotMap.chassisLeftFront;
    SpeedController leftRear = RobotMap.chassisLeftRear;
    SpeedController rightFront = RobotMap.chassisRightFront;
    SpeedController rightRear = RobotMap.chassisRightRear;
    RobotDrive robotDrive = RobotMap.chassisRobotDrive;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        setDefaultCommand(new Drive());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
	
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public void driveWithJoystick(Joystick s){
        robotDrive.arcadeDrive(s);
    }
    public void stop(){
        robotDrive.stopMotor();
    }
    public void aim() {
        // TODO: aiming
    }
}
