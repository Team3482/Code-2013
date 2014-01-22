// RobotBuilder Version: 0.0.2
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in th future.


package robot.core.subsystems;

import robot.core.RobotMap;
import robot.core.commands.*;
import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
import robot.core.Robot;


/**
 *
 */
public class Chassis extends Subsystem {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SpeedController leftFront = RobotMap.chassisLeftFront;
    SpeedController leftRear = RobotMap.chassisLeftRear;
    SpeedController rightFront = RobotMap.chassisRightFront;
    SpeedController rightRear = RobotMap.chassisRightRear;
    SpeedController armController = RobotMap.shooterCam;
    RobotDrive robotDrive = RobotMap.chassisRobotDrive;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        setDefaultCommand(new Drive());
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
	
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public void invertMotors() {
        robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
        robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
    }
    public void driveWithJoystick(Joystick s) {
        double deadZone = .2;
        double xAxis = s.getAxis(Joystick.AxisType.kX);
        double yAxis = s.getAxis(Joystick.AxisType.kY);
        //System.out.print("Raw input: \n ");
        //System.out.println(xAxis + ", " + yAxis);
        
        // X sensitivity set by slider, Y sensitivity set by knob
        double slider = SmartDashboard.getNumber("Slider 1");
        xAxis *= (slider/100);
        double knob = s.getAxis(Joystick.AxisType.kZ);
        knob = 1 - (knob/2);    // Format input from Z Axis
        yAxis *= knob;

        // If the X or Y axes are in the deadzone, flip them to zero.
        if (xAxis < deadZone && xAxis > -deadZone) {
            xAxis = 0;
        }
        if (yAxis < deadZone && yAxis > -deadZone) {
            yAxis = 0;
        }
        //System.out.print("Dead zone corrected: \n");
        //System.out.println(xAxis + ", " + yAxis);
        robotDrive.arcadeDrive(yAxis, xAxis);
    }
    public void driveWithXboxController(Joystick s) {
        double leftY = s.getRawAxis(2);
        double rightX = s.getRawAxis(4);
        double sensitivity = SmartDashboard.getNumber("Slider 1");
        sensitivity /= 100;
        double deadZone = 0.1;
        
        if(leftY < deadZone && leftY > -deadZone) {
            leftY = 0;
        }
        if(rightX < deadZone && rightX > -deadZone) {
            rightX = 0;
        }
        leftY *= sensitivity;
        robotDrive.arcadeDrive(leftY, rightX);
        //boolean squaredInputs = true;
        //robotDrive.arcadeDrive(leftY, rightX, squaredInputs);
    }
    public void move(double moveValue, double rotateValue) {
        robotDrive.arcadeDrive(moveValue, rotateValue);
    }
    public void stop() {
        robotDrive.stopMotor();
    }
    public void setSafety(boolean n) {
        robotDrive.setSafetyEnabled(n);
    }
    public void moveArm(double yolo) {
        armController.set(yolo);
    }
}
