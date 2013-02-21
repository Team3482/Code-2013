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
import com.sun.squawk.util.Arrays;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCamera.*;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.Timer;
import robot.core.commands.*;
import edu.wpi.first.wpilibj.image.*;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.sun.squawk.util.*;
/**
 * The Camera subsystem provides vision processing functionality
 * for the Axis Camera
 */
// TODO: free images
public class Camera extends Subsystem {
    // Camera setting constants
    static final int BRIGHTNESS  = 50;
    static final int COLOR_LEVEL = 50;
    static final int COMPRESSION = 30;
    static final int MAX_FPS     = 24;
    static final ExposurePriorityT EXPOSURE_PRIORITY = ExposurePriorityT.none;
    static final ResolutionT       RESOLUTION        = ResolutionT.k320x240;
    static final RotationT         ROTATION          = RotationT.k0;
    static final WhiteBalanceT     WHITE_BALANCE     = WhiteBalanceT.fixedIndoor;
    static final String IPAdress = "10.34.82.11";
    
    // TODO: find constants for *our* ring light
    // Pixel filtering constants for the green ring light in the sample images (HSV)
    static final int HUE_LOW         = 96;
    static final int HUE_HIGH        = 114;
    static final int SATURATION_LOW  = 148;
    static final int SATURATION_HIGH = 255;
    static final int VALUE_LOW       = 84;
    static final int VALUE_HIGH      = 162;
    
    BinaryImage threshold, convexHull, filtered;
    static CriteriaCollection cc = new CriteriaCollection();
    
    // Area ranges
    static final float   AREA_LOW      = 500;
    static final float   AREA_HIGH     = 65535;
    static final boolean OUTSIDE_RANGE = false;
    
    // Max/Min edge scores
    final int XMAXSIZE = 24;
    final int XMINSIZE = 24;
    final int YMAXSIZE = 24;
    final int YMINSIZE = 48;
    static final double xMax[] = { 1,  1,   1,   1,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,   1,   1,   1,   1};
    static final double xMin[] = {.4, .6,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1, 0.6,   0};
    static final double yMax[] = { 1,  1,   1,   1,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,  .5,   1,   1,   1,   1};
    static final double yMin[] = {.4, .6, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .6, 0};

    static final double COMPOSITE_MIN = 75;

    public class Scores {
        double rectangularity;
        double aspectRatioMiddle;
        double aspectRatioOuter;
        double xEdge;
        double yEdge;
        double compositeScore;
        int particleNumber;
    }
    public class CompareScores implements Comparer {
        public int compare(Object o1, Object o2) {
            Scores s1 = (Scores)o1;
            Scores s2 = (Scores)o2;
            if(s1.compositeScore > s2.compositeScore) {
                return 1;
            } else if(s1.compositeScore < s2.compositeScore) {
                return -1;
            } else {
                return 0;
            }
        }
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    /* ========================================================
     * |  SCORING
     * ======================================================== 
     */
    public Scores[] score(ColorImage img) throws NIVisionException {
        // TODO: Add waits before looping again (preferably until a new image is available)
        // Adds particle area to the CriteriaCollection object for calculating rectangularity score
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, AREA_LOW, AREA_HIGH, OUTSIDE_RANGE);
        boolean connectivity8 = false;
        
        threshold = img.thresholdHSV(HUE_LOW, HUE_HIGH, SATURATION_LOW,
            SATURATION_HIGH, VALUE_LOW, VALUE_HIGH);
        convexHull = threshold.convexHull(connectivity8);
        filtered = convexHull.particleFilter(cc);
        
        Scores[] scores = new Scores[filtered.getNumberParticles()];
        for(int i = 0; i < scores.length; i++) {
            ParticleAnalysisReport report = filtered.getParticleAnalysisReport(i);
            
            scores[i].rectangularity    = scoreRectangularity(report);
            scores[i].aspectRatioMiddle = scoreAspectRatio(report, true); 
            scores[i].aspectRatioOuter  = scoreAspectRatio(report, false);
            scores[i].xEdge             = scoreXEdge(threshold, report);
            scores[i].yEdge             = scoreYEdge(threshold, report);
            scores[i].compositeScore    = compositeScore(scores[i]);
            scores[i].particleNumber    = i;
        }
        return scores;
    }
    
    private double scoreRectangularity(ParticleAnalysisReport report) {
        double boundingRectArea;
        
        if((boundingRectArea = report.boundingRectHeight*report.boundingRectWidth) != 0) {
            return 100*report.particleArea/boundingRectArea;
        } else {
            return 0;
        }
    }
    private double scoreAspectRatio(ParticleAnalysisReport report, boolean middle) {
        // TODO: Does this method work as it should?
        double aspectRatio = report.boundingRectWidth/report.boundingRectHeight;
        double idealAspectRatio = middle ? (62/29) : (62/20);
        return Math.max(0, 100 * (1 - Math.abs(1 - aspectRatio/idealAspectRatio)));
    }
    private double scoreXEdge(BinaryImage img, ParticleAnalysisReport report) throws NIVisionException {
        LinearAverages avgs;
        float[] colAvgs;
        int c = 0;
        
        NIVision.Rect rect = new NIVision.Rect(report.boundingRectTop, report.boundingRectLeft,
                report.boundingRectHeight, report.boundingRectWidth);
        avgs = NIVision.getLinearAverages(img.image,
                LinearAverages.LinearAveragesMode.IMAQ_COLUMN_AVERAGES, rect);
        colAvgs = avgs.getColumnAverages();
        
        for(int i = 0; i < colAvgs.length; i ++) {
            if(xMin[(i*(XMAXSIZE-1)/colAvgs.length)] <= colAvgs[i] &&
                    xMax[(i*(XMAXSIZE-1)/colAvgs.length)] >= colAvgs[i]) {
                c++;
            }
        }
        
        return 100 * c/(colAvgs.length);
    }
    private double scoreYEdge(BinaryImage img, ParticleAnalysisReport report) throws NIVisionException {
        LinearAverages avgs;
        float[] rowAvgs;
        int c = 0;
        
        NIVision.Rect rect = new NIVision.Rect(report.boundingRectTop, report.boundingRectLeft,
                report.boundingRectHeight, report.boundingRectWidth);
        avgs = NIVision.getLinearAverages(img.image,
                LinearAverages.LinearAveragesMode.IMAQ_ROW_AVERAGES, rect);
        rowAvgs = avgs.getRowAverages();
        
        for(int i = 0; i < rowAvgs.length; i++) {
            if(yMin[(i*(YMINSIZE-1)/rowAvgs.length)] <= rowAvgs[i] &&
                    yMax[(i*(YMAXSIZE-1)/rowAvgs.length)] >= rowAvgs[i]) {
                c++;
            }
        }
        
        return 100 * c/(rowAvgs.length);
    }
    private double compositeScore(Scores score) {
        double aspectRatio = score.aspectRatioMiddle > score.aspectRatioOuter ? score.aspectRatioMiddle : score.aspectRatioOuter;
        double compositeScore = (score.rectangularity + aspectRatio + score.xEdge + score.yEdge) / 4.0;
        return compositeScore;
    }
    
    /* ========================================================
     * |  TARGETING
     * ======================================================== 
     */
    /**
     * Finds the three (or less) highest scoring particles.
     * Uses the getCompositeScore() method of the Scores class to find the three
     * highest composite particle scores, and returns their indices. Returns the
     * indices because the MeasureParticle() method requires the index of the
     * particle.
     * @author Rodrigo Valle
     * @param score
     * @return Integer array of the indices of the three highest scoring particles
     */
    public int[] getTargets(Scores[] score) {
        // TODO: copy the score array? Currently sorts the score array passed to it.
        Arrays.sort(score, new CompareScores());
        int c = 0;
        for(int i = 0; i < 3; i++) {
            if(score[i].compositeScore >= COMPOSITE_MIN) {
                c++;
            } else {
                break;
            }
        }
        int[] topScores = new int[c];
        for(int i = 0; i < topScores.length; i++) {
            topScores[i] = score[i].particleNumber;
        }
        
        return topScores;
    }
    
    /**
     * Finds offset of the selected target from the center of the image.
     * Takes the filtered convex hull image.
     * @param img
     * @param topScores
     * @param target 0 => left target, 1 => middle target, 2 => right target
     * @return
     * 
     */
    public double calculateOffset(int[] topScores, int target) throws NIVisionException {
        // Check to make sure selected target is valid
        if(target < 0 || target >= topScores.length) {
            return -1000.0;  // invalid target
            // TODO: Perhaps print an error to the DriverStationLCD.
        }
        double[] positionsXCenter = new double[topScores.length];
        for(int i = 0; i < positionsXCenter.length; i++) {
            double firstPixelX   = NIVision.MeasureParticle(filtered.image, topScores[i], false, NIVision.MeasurementType.IMAQ_MT_FIRST_PIXEL_X);
            double particleWidth = NIVision.MeasureParticle(filtered.image, topScores[i], false, NIVision.MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH);
            positionsXCenter[i]  = firstPixelX + particleWidth/2.0;
        }
        // Sorts from least to greatest.
        Arrays.sort(positionsXCenter);
        // if left target is selected
        double imageCenter = positionsXCenter[0] - filtered.getWidth()/2.0;
        if(target == 0) {
            // positions to the right of the center are positive
            // positions to the left of the center are negative
            return positionsXCenter[0] - imageCenter;
        } else if(target == 1) {
            return positionsXCenter[1] - imageCenter;
        } else /*if(target == 2)*/ {
            return positionsXCenter[2] - imageCenter;
        }
    }
    
    public double calculateDistance() {
        // TODO: Implement calculateDistance() method.
        return 0.0;
    }
    
    // Retrieving images from the camera
    public boolean freshImage() {
        return AxisCamera.getInstance(IPAdress).freshImage();
    }
    public ColorImage getImage() throws AxisCameraException, NIVisionException {
        return AxisCamera.getInstance(IPAdress).getImage();
    }
    public void freeImages() throws NIVisionException {
        threshold.free();
        convexHull.free();
        filtered.free();
    }

    // Method to configure camera settings
    public void configureCamera() {
        AxisCamera c = AxisCamera.getInstance();
        c.writeBrightness(BRIGHTNESS);
        c.writeColorLevel(COLOR_LEVEL);
        c.writeCompression(COMPRESSION);
        c.writeExposurePriority(EXPOSURE_PRIORITY);
        c.writeMaxFPS(MAX_FPS);
        c.writeResolution(RESOLUTION);
        c.writeRotation(ROTATION);
        c.writeWhiteBalance(WHITE_BALANCE);

        c.writeExposureControl(ExposureT.automatic);
        Timer.delay(5);
        c.writeExposureControl(ExposureT.hold);
    }
}
