package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


//Here is a new and improved version of the prop detector
// using HSV masking so that we can filter certain colors and search the whole screen
// -Ambrose :D
public class propDetector extends OpenCvPipeline {
    enum propLocation {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    private int width; // width of the frame
    private String color; // color to search for
    propLocation location;

    /**
     *Detector for red/blue game props using HSV masking
     * @param width The width of the frame (check your camera)
     * @param color the color of the prop you want to detect ("RED"/"BLUE")
     */
    public propDetector(int width, String color) {
        this.width = width;
        this.color = color;
    }
    //constructor go brr

    @Override
    public Mat processFrame(Mat input) {

        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); //change to hsv for processing

        // if something is wrong, we assume none and should default to center
        if (mat.empty()) {
            location = propLocation.NONE;
            return input;
        }

        Scalar lowHSV;
        Scalar highHSV;

        if (color == "RED" || color == "red") { //red low and high HSV ranges
            lowHSV = new Scalar(0.5, 80, 80);
            highHSV = new Scalar(15, 255, 255);
        } else { //blue low and high HSV ranges
            lowHSV = new Scalar(210, 255, 255);
            highHSV = new Scalar(245, 80, 80);
        } //if someone puts in a value other than blue or red (for whatever reason), we default to blue

        Mat thresh = new Mat();

        Core.inRange(mat, lowHSV, highHSV, thresh); //filter the image through to extract only red/blue

        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);

        //fancy getting contours stuff
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // Iterate and check whether the bounding boxes are on left, center, or right.
        double left_x = 0.33 * width; //right now it's approx third regions of the screen - adjust as needed!
        double right_x = 0.66 * width;
        for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].x + (int)(boundRect[i].width / 2) > left_x && boundRect[i].x + (boundRect[i].width / 2) < right_x) { //left_x < boundRect[i].x + (boundRect[i].width / 2) < right_x
                location = propLocation.CENTER; //if the prop is in the center
            } else if (boundRect[i].x + (int)(boundRect[i].width / 2) < left_x) {
                location = propLocation.LEFT; //if the prop is in the left region
            } else if (boundRect[i].x + (int)(boundRect[i].width / 2) > right_x) {
                location = propLocation.RIGHT; //if the prop is in the right region
            } else {
                location = propLocation.NONE; //probably would never get this
            }
            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
        }

        return mat; // return the mat with rectangles drawn
    }

    public propLocation getLocation() {
        return this.location; //gimme that spike mark info
    }
}