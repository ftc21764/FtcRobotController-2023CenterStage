package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class colorDetect extends OpenCvPipeline {
    Mat mat = new Mat();
//    Rect leftROI = new Rect(new Point(0, 0), new Point(183, 300));
//    Rect centerROI = new Rect(new Point(183, 0), new Point(366, 300));
//    Rect rightROI = new Rect(new Point(366, 0), new Point(550, 300));
//    Mat leftMat;
//    Mat centerMat;
//    Mat rightMat;
    Telemetry telemetry;

//    public colorDetect (HardwareMap hwMap, Telemetry t) {
//        telemetry = t;
//        int camMonViewId = = hwMap.appContext.getResources.getIdentifier
//    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);

        Scalar lowerBound = new Scalar(136, 87, 111); //red lower range
        Scalar upperBound = new Scalar(180, 255, 255); //red upper range

        Core.inRange(mat, lowerBound, upperBound, mat);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        for (MatOfPoint contour : contours) {
            MatOfPoint2f copy = new MatOfPoint2f(contour.toArray());
            Rect rect = Imgproc.boundingRect(copy);
            int blobX = rect.x;
            int blobY = rect.y;
            contour.release();
            copy.release();

        }

//        leftMat = mat.submat(leftROI);
//        centerMat = mat.submat(centerROI);
//        rightMat = mat.submat(rightROI);
//
//        double leftVal = Math.round(Core.mean(leftMat).val[2] / 255);
//        double centerVal = Math.round(Core.mean(centerMat).val[2] / 255);
//        double rightVal = Math.round(Core.mean(rightMat).val[2] / 255);
//
//        leftMat.release();
//        centerMat.release();
//        rightMat.release();
//        mat.release();
        return null;
    }
}
