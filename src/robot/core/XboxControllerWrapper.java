/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package robot.core;

import ch.aplu.xboxcontroller.XboxController;
import ch.aplu.xboxcontroller.XboxControllerAdapter;
import ch.aplu.xboxcontroller.XboxControllerListener;

/**
 *
 * @author
 * Westmont
 * Robotics
 */

public class XboxControllerWrapper {
	private final String xboxControllerDLL = "C:/User/Westmont Robotics/Downloads/XboxController/dll/xboxcontroller.dll";
	private final int playerNum = 1;
	// check controller state period in ms
	private final int controllerPoll = 50;
	// period of the timer that polls the message queue in ms
	private final int queuePoll = 50;
	XboxController c = new XboxController(xboxControllerDLL, playerNum, controllerPoll, queuePoll);

	boolean backButton, AButton, BButton, XButton, YButton, dPadPressed;
	boolean leftBumper, leftThumb, rightBumper, rightThumb, startButton;
	public XboxControllerWrapper(XboxController controller) {
		c = controller;
		c.addXboxControllerListener(new XboxControllerAdapter() {
			public void back(boolean value) {
			}
			public void buttonA(boolean value) {
			}
			public void buttonB(boolean value) {
			}
			public void buttonX(boolean value) {
			}
			public void buttonY(boolean value) {
			}
			public void dpad(int direction, boolean pressed) {
			}
			public void leftShoulder(boolean pressed) {
			}
			public void leftThumb(boolean pressed) {
			}
			public void leftThumbDirection(double direction) {
			}
			public void leftThumbMagnitude(double magnitude) {
			}
			public void leftTrigger(double value) {
			}
			public void rightShoulder(boolean pressed) {
			}
			public void rightThumb(boolean pressed) {
			}
			public void rightThumbDirection(double direction) {
			}
			public void rightThumbMagnitude(double magnitude) {
			}
			public void rightTrigger(double value) {
			}
			public void start(boolean pressed) {
			}
		});
	}
	public void close() {
		c.release();
	}
}