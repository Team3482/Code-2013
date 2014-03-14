/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package robot.core.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.*;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author
 * Westmont
 * Robotics
 * 
 */
public class Camera extends Subsystem {
	/* The camera subsytem allows for processing of an image to detect both the vertical
	 * and horizontal vision targets (when present). Image processing here is not real time,
	 * instead a single image is captured and analyzed, and we assume that only one pair of
	 * vision targets is visible. The image is captured at the start of autonomous and if both
	 * targets are visible (the goal is "hot"), the robot scores the ball. Otherwise, if only
	 * the vertical target is detected, the robot waits five seconds for the hot targets to 
	 * change and then scores the ball. If neither target is visible, then the robot defaults
	 * to scoring the ball.
	 * */

	// Initialize variables used in this subsystem
	AxisCamera cam;
	ColorImage colorImg;
	BinaryImage filtered;
	BinaryImage sizeFiltered;
	BinaryImage convexHull;
	boolean foundHorizontalTarget = false;
	boolean foundVerticalTarget = false;

	// These are constants for our image filters #vintage #hipster #instagram #yolo
	final int HUE_LOW = 93;
	final int HUE_HIGH = 169;
	final int SATURATION_LOW = 195;
	final int SATURATION_HIGH = 255;
	final int VALUE_LOW = 180;
	final int VALUE_HIGH = 255;
	final String IP_ADDRESS = "10.34.82.11";
	final int BRIGHTNESS = 40;
	final int COLOR_LEVEL = 70;
	final int COMPRESSION = 20;

	// These constants are used for target detection and thresholding
	final double VERTICAL_ASPECT_RATIO = 0.125;  // aspect ratio of the vertical target
	final double HORIZONTAL_ASPECT_RATIO = 5.875;  // aspect ratio fo the horizontal target
	final double MIN_RECTANGULARITY = 80.0;  // this is a percentage, spectifies a minimum cutoff
	final double ASPECT_RATIO_BOUND = 20.0;  /* the aspect ratio is a percent error, this variable
	                                          * bounds it at +/- its value */

	/* These are the settings that we use with the camera, to ensure that camera
	 * settings do not change and interfere with image processing */
	final AxisCamera.ExposureT EXPOSURE = AxisCamera.ExposureT.flickerfree60;
	final AxisCamera.ExposurePriorityT EXPOSURE_PRIORITY = AxisCamera.ExposurePriorityT.imageQuality;
	final int MAX_FPS = 20;
	final AxisCamera.ResolutionT RESOLUTION = AxisCamera.ResolutionT.k640x480;
	final AxisCamera.RotationT ROTATION = AxisCamera.RotationT.k0;
	final AxisCamera.WhiteBalanceT WHITE_BALANCE = AxisCamera.WhiteBalanceT.fixedIndoor;

	public Camera() {
		/* Call initCamera() here so that (ideally) the robot is ready to go before autonomous
		 * mode is instantiated */
		try {
			initCamera();
		} catch (Exception e) {
			System.out.println(e.getMessage());
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
	}
	
	public void initCamera() throws AxisCameraException {
		/* Call this function before trying to process images, this only
		 * has to be called once, and only once. It takes too much time
		 * to run, so consider running this before the match begins */

		long startTime = System.currentTimeMillis();  // this is for timing purposes
		cam = AxisCamera.getInstance(IP_ADDRESS);  // grab the camera at the default IP
		while (!cam.freshImage()) {  // wait for the camera to finish starting up
			Timer.delay(0.1);
		}

		/* Set up the camera settings to make sure they're consistent.
		 * This takes a *lot* of time (about six to ten seconds) */
		cam.writeBrightness(BRIGHTNESS);
		cam.writeColorLevel(COLOR_LEVEL);
		cam.writeCompression(COMPRESSION);
		cam.writeExposureControl(EXPOSURE);
		cam.writeExposurePriority(EXPOSURE_PRIORITY);
		cam.writeMaxFPS(MAX_FPS);
		cam.writeResolution(RESOLUTION);
		cam.writeRotation(ROTATION);
		cam.writeWhiteBalance(WHITE_BALANCE);
		
		// Output function execution time
		long endTime = System.currentTimeMillis();
		System.out.println("Initialization Time: " + (endTime - startTime));
	}

	public void processImage() throws AxisCameraException, NIVisionException {
		/* This grabs a single image, processes it, and stores the particle scores in a local array */

		// For timing function execution
		long startTime = System.currentTimeMillis();
		
		// Get an image and immediately write it for debugging purposes
		colorImg = cam.getImage();
		colorImg.write("/tmp/axis_camera_rgb.png");
		
		// Filter the image -> returns a binary image + write
		filtered = colorImg.thresholdHSV(HUE_LOW, HUE_HIGH, SATURATION_LOW, SATURATION_HIGH, VALUE_LOW, VALUE_HIGH);
		filtered.write("/tmp/filtered_image.png");
		
		/* Filter blobs by size, save it to a different image variable otherwise it will cause a memory leak.
		 * For example, the line
		 *	   filtered = filtered.removeSmallObjects(...) 
		 *   would cause a leak because the image data originally stored in filtered is not freed.
		 *   Do not do this, save your image data in a separate variable and make sure to free all image
		 * variables later. */     
		sizeFiltered = filtered.removeSmallObjects(true, 3);
		sizeFiltered.write("/tmp/size_filtered.png");
		
		// Convex hull helps fill in holes in [articles
		convexHull = sizeFiltered.convexHull(false);
		convexHull.write("/tmp/convex_hull.png");
	
		// Print the execution time
		long endTime = System.currentTimeMillis();
		System.out.println("Image Write(s) Time: " + (endTime - startTime));

		// Calculate particle scores and time it
		startTime = System.currentTimeMillis();
		calculateScores(filtered);
		endTime = System.currentTimeMillis();
		System.out.println("Score Processing Time: " + (endTime - startTime));

		// Free image variables
		colorImg.free();
		filtered.free();
		sizeFiltered.free();
		convexHull.free();
	}

	public boolean foundVertical() {
		/* Returns the corresponding boolean variable set in calculateScores() */

		return foundVerticalTarget;
	}

	public boolean foundHorizontal() {
		return foundHorizontalTarget;
	}

	/**************************
	 *    HELPER FUNCTIONS    *
	 **************************/
	private void calculateScores(BinaryImage img) throws NIVisionException {
		/* Calculate the rectangularity score as a percentage, and the vertical/horizontal
		 * aspect ratios */

		// getOrdered...() returns particles ordered from greatest area to least
		ParticleAnalysisReport[] reports = img.getOrderedParticleAnalysisReports();
		double[][] scoredBlobs = new double[reports.length][3];
		for (int i = 0; i < reports.length; i++) {
			/* Store rectangularity + veritcal/horizontal aspect ratio
			 * The resulting array looks like this:
			 *     [[rectangularity, vertical AR score, horizontal AR score], ...] */
			scoredBlobs[i][0] = scoreRectangularity(reports[i]);
			scoredBlobs[i][1] = scoreAspectRatioVertical(reports[i]);
			scoredBlobs[i][2] = scoreAspectRatioHorizontal(reports[i]);

			/* If any scored blobs are detected as horizontal or vertical at any point, 
			 * the corresponding variables will stay true (they're initialized false) */
			foundHorizontalTarget = foundHorizontalTarget || isHorizontal(scoredBlobs[i]);
			foundVerticalTarget = foundVerticalTarget || isVertical(scoredBlobs[i]);
		}
	}

	private double scoreRectangularity(ParticleAnalysisReport blob) {
		/* Returns rectangularity score as a value from 0 to 100, with 100 being the best */

		return blob.particleArea / (blob.boundingRectWidth * blob.boundingRectHeight) * 100;
	}

	private double scoreAspectRatioVertical(ParticleAnalysisReport blob) {
		/* Returns vertical aspect ratio score as a percent error, the closer to zero the better */

		double blobAspectRatio = (double) blob.boundingRectWidth / blob.boundingRectHeight;
		return Math.abs(VERTICAL_ASPECT_RATIO - blobAspectRatio) / VERTICAL_ASPECT_RATIO * 100;
	}
	
	private double scoreAspectRatioHorizontal(ParticleAnalysisReport blob) {
		/* Returns horizontal aspect ratio score as a percent error, the closer to zero the better */

		double blobAspectRatio = (double) blob.boundingRectWidth / blob.boundingRectHeight;
		return Math.abs(HORIZONTAL_ASPECT_RATIO - blobAspectRatio) / HORIZONTAL_ASPECT_RATIO * 100;
	}
	
	private boolean isVertical(double[] scoredBlob) {
		/* Determines whether a scored particle falls within the threshold, and prints debugging information */

		if (scoredBlob[0] > MIN_RECTANGULARITY && scoredBlob[1] < ASPECT_RATIO_BOUND) {
			System.out.println("Vertical Target Matched");
			System.out.println("Rectangularity, Aspect Ratio");
			System.out.println(scoredBlob[0] + ", " + scoredBlob[1]);
			return true;
		} else {
			System.out.println("Rectangularity, Aspect Ratio");
			System.out.println(scoredBlob[0] + ", " + scoredBlob[1]);
		}
		return false;
	}
	
	private boolean isHorizontal(double[] scoredBlob) {
		if (scoredBlob[0] > MIN_RECTANGULARITY && scoredBlob[2] < ASPECT_RATIO_BOUND) {
			System.out.println("Horizontal Target Matched");
			System.out.println("Rectangularity, Aspect Ratio");
			System.out.println(scoredBlob[0] + ", " + scoredBlob[2]);
			return true;
		} else {
			System.out.println("Rectangularity, Aspect Ratio");
			System.out.println(scoredBlob[0] + ", " + scoredBlob[2]);
		}
		return false;
	}

}
