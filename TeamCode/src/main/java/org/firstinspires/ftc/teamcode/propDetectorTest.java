package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


//test OpMode for the code I just wrote
//-Ambrose
@Autonomous(name="propDetector: Test OpMode")
public class propDetectorTest extends LinearOpMode {
    // Handle hardware stuff...

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    propDetector detector = new propDetector(width, "RED");
    OpenCvCamera camera;

    @Override
    public void runOpMode() {
        waitForStart();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        camera.setPipeline(detector);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        //maybe put this in a while(opModeInInit) loop?
        propDetector.propLocation location = detector.getLocation();
        if (location != propDetector.propLocation.NONE) {
            if (location == propDetector.propLocation.LEFT) {
                //go to the left
            } else if (location == propDetector.propLocation.RIGHT) {
                //go to the right
            } else { //must be center
                //go to the center
            }
        } else {
            // uh oh! default to center
        }

        // more autonomous stuff...
    }
}