package org.firstinspires.ftc.teamcode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;
import java.util.ArrayList;

public class colorDetect {
    public void processFrame(Mat input) {
        Scalar redLowerRange = new Scalar(136, 87, 111);
        Scalar redUpperRange = new Scalar(180, 255, 255);

        Scalar blueLowerRange = new Scalar(94, 80, 2);
        Scalar blueUpperRange = new Scalar(120, 255, 255);

        Mat redMat = new Mat();
        Mat blueMat = new Mat();
        Imgproc.cvtColor(input, redMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, blueMat, Imgproc.COLOR_RGB2HSV);

        Mat redThresh = new Mat();
        Mat blueThresh = new Mat();

        Core.inRange(redMat, redLowerRange, redUpperRange, redThresh);
        Core.inRange(blueMat, blueLowerRange, blueUpperRange, blueThresh);

        Mat redEdges = new Mat();
        Imgproc.Canny(redThresh, redEdges, 100, 300); //TUNE AS NECESSARY

        Mat blueEdges = new Mat();
        Imgproc.Canny(blueThresh, blueEdges, 100, 300); //TUNE AS NECESSARY

        List<MatOfPoint> redContours = new ArrayList<>();
        Mat redHierarchy = new Mat();
        Imgproc.findContours(redEdges, redContours, redHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        List<MatOfPoint> blueContours = new ArrayList<>();
        Mat blueHierarchy = new Mat();
        Imgproc.findContours(redEdges, redContours, redHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
    }
}