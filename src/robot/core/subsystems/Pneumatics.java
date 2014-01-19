/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package robot.core.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.core.RobotMap;
import robot.core.commands.Compress;

/**
 *
 * @author Westmont Robotics
 */
public class Pneumatics extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    Compressor compressor = RobotMap.compressor;
    DoubleSolenoid testCylinder = RobotMap.cylinder;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new Compress());
    }
    public void startCompressorLoop() {
        compressor.start();
    }
    public void stopCompressorLoop() {
        compressor.stop();
    }
    public void extend() {
        testCylinder.set(DoubleSolenoid.Value.kForward);
    }
    public void retract() {
        testCylinder.set(DoubleSolenoid.Value.kReverse);
    }
    public void turnOff() {
        testCylinder.set(DoubleSolenoid.Value.kOff);
    }
}
