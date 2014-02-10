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
 * @author Westmont Robotics
 */
public class Camera extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    AxisCamera cam;
    ColorImage colorImg;
    BinaryImage filtered;
    final int HUE_LOW = 115;
    final int HUE_HIGH = 140;
    final int SATURATION_LOW = 215;
    final int SATURATION_HIGH = 255;
    final int VALUE_LOW = 215;
    final int VALUE_HIGH = 255;
    double[][] scores;
    
    final String IP_ADDRESS = "10.34.82.11";
    final int BRIGHTNESS = 40;
    final int COLOR_LEVEL = 70;
    final int COMPRESSION = 20;
    final AxisCamera.ExposureT EXPOSURE = AxisCamera.ExposureT.flickerfree60;
    final AxisCamera.ExposurePriorityT EXPOSURE_PRIORITY = AxisCamera.ExposurePriorityT.imageQuality;
    final int MAX_FPS = 20;
    final AxisCamera.ResolutionT RESOLUTION = AxisCamera.ResolutionT.k640x480;
    final AxisCamera.RotationT ROTATION = AxisCamera.RotationT.k0;
    final AxisCamera.WhiteBalanceT WHITE_BALANCE = AxisCamera.WhiteBalanceT.fixedIndoor;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    // Call this function before trying to process images
    public void initCamera () throws AxisCameraException {
        long startTime = System.currentTimeMillis();
        cam = AxisCamera.getInstance(IP_ADDRESS);
        while(!cam.freshImage()) {
            Timer.delay(0.5);
        }
        /*cam.writeBrightness(BRIGHTNESS);
        cam.writeColorLevel(COLOR_LEVEL);
        cam.writeCompression(COMPRESSION);
        cam.writeExposureControl(EXPOSURE);
        cam.writeExposurePriority(EXPOSURE_PRIORITY);
        cam.writeMaxFPS(MAX_FPS);
        cam.writeResolution(RESOLUTION);
        cam.writeRotation(ROTATION);
        cam.writeWhiteBalance(WHITE_BALANCE);*/
        long endTime = System.currentTimeMillis();
        System.out.println("Initialization Time: " + (endTime - startTime));
    }
    public void processImage() throws AxisCameraException, NIVisionException {
        long startTime = System.currentTimeMillis();
        colorImg = cam.getImage();
        colorImg.write("/tmp/axis_camera_rgb.png");
        filtered = colorImg.thresholdHSV(HUE_LOW, HUE_HIGH, SATURATION_LOW, SATURATION_HIGH, VALUE_LOW, VALUE_HIGH);
        filtered.write("/tmp/filtered_image.png");
        filtered = filtered.removeSmallObjects(true, 3);
        filtered.write("/tmp/size_filtered.png");
        filtered = filtered.convexHull(false);
        filtered.write("/tmp/convex_hull.png");
        long endTime = System.currentTimeMillis();
        System.out.println("Image Write(s) Time: " + (endTime - startTime));
        
        startTime = System.currentTimeMillis();
        scores = calculateScores(filtered);
        endTime = System.currentTimeMillis();
        System.out.println("Score Processing Time: " + (endTime - startTime));
        
        // free image memory
        colorImg.free();
        filtered.free();
    }
    private double[][] calculateScores(BinaryImage img) throws AxisCameraException, NIVisionException {
        // blobs in reports[] are ordered by area from greatest to least
        ParticleAnalysisReport[] reports = img.getOrderedParticleAnalysisReports();
        double[][] scoredBlobs = new double[reports.length][2];
        for(int i = 0; i < reports.length; i++) {
            scoredBlobs[i][0] = scoreRectangularity(reports[i]);
            scoredBlobs[i][1] = scoreAspectRatioVertical(reports[i]);
        }
        return scoredBlobs;
    }
    private double scoreRectangularity(ParticleAnalysisReport blob) {
        // returns rectangularity score as a value from 0 to 100, with 100 being the best
        return (blob.particleArea / (blob.boundingRectWidth*blob.boundingRectHeight) * 100);
        
    }
    private double scoreAspectRatioVertical(ParticleAnalysisReport blob) {
        // returns aspect ratio score as a percent error, the closer to zero the better
        final double VERTICAL_ASPECT_RATIO = 0.125;
        double blobAspectRatio = (double) blob.boundingRectWidth / blob.boundingRectHeight;
        System.out.println("Aspect Ratio: " + blobAspectRatio);
        return (Math.abs(VERTICAL_ASPECT_RATIO - blobAspectRatio) / VERTICAL_ASPECT_RATIO * 100);
    }
    public boolean isVertical() {
        System.out.println("blob scores length: " + scores.length);
        for(int i = 0; i < scores.length; i++) {
            if(scores[i][0] > 80 && scores[i][1] < 30) {
                System.out.println("Vertical Target Matched");
                System.out.println("Rectangularity, Aspect Ratio");
                System.out.println(scores[i][0] + ", " + scores[i][1]);
                return true;
            } else {
                System.out.println("Rectangularity, Aspect Ratio");
                System.out.println(scores[i][0] + ", " + scores[i][1]);
            }
        }
        return false;
    }
}
