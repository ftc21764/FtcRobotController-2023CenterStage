/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="CenterStage Autonomous 21764", group="Robot")
//@Disabled
public class CenterStageAutonomous extends LinearOpMode {

    protected boolean Overrideselection = false;
//    protected FirstVisionProcessor.Selected selectionOverride = FirstVisionProcessor.Selected.MIDDLE;

    /* Declare OpMode members. */
    protected DcMotor leftDriveF = null;
    protected DcMotor leftDriveB = null;
    protected DcMotor rightDriveF = null;
    protected DcMotor rightDriveB = null;
    protected IMU imu = null;      // Control/Expansion Hub IMU
    //protected SignalSleeveRecognizer    recognizer = null;
    //protected LinearSlide         linearSlide = null;
    protected Intake intake = null;
    protected SwingArm swingArm = null;
    protected ElapsedTime runtime = new ElapsedTime();
    protected DeliveryServoController deliveryBoard = null;

    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;


    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private double frontSpeed = 0;
    private double backSpeed = 0;
    private int leftTargetF = 0;
    private int leftTargetB = 0;
    private int rightTargetF = 0;
    private int rightTargetB = 0;

    boolean isMirrored = true;
    boolean notMirrored = false;
    boolean isRed;

    boolean isFar = true;
    boolean parkOnly = true;
    boolean trianglePark = false;
    boolean stalling = false; //cap

    int armHardStopID = 0;
    int armPickUpID = 1;
    int armCarryID = 2;
    int armDeliveryID = 3;
    int armDrivePosID = 4;
    int armHighestScoringID = 5;
    int driveStraightLoops = 0;

    double tbegin;

    int aprilTagDriveDistance = 0;
    int aprilTagIdNumber = 0;
    int aprilTagInitialDistance = 24;
    int aprilNearDistance = 5;
    int aprilMidDistance = 19;
    int aprilFarDistance = 27;
    int driveBackToSeeAprilTag = 0;
    double distanceToScoreFromTags = 0;
    double driveDistanceToScore = -16.0;
    double aprilTag_CenterGoal = 0; // zero or how far to the left or right we want to be
    double aprilTag_Threshold = 0.5; //was 0.25
    double aprilTag_AdjustedX = 0; // this will be our adjusted value off center goal of ftcPose.X
    double LFRBSpeed = 0;
    double RFLBSpeed = 0;

    double wallToTriangleDriveDistance = 80;
    double scoringDistanceToTrianglePark;

    boolean strafeCorrectionDone = false;

    //String allianceColor = "blue";

    private FirstVisionProcessor propDetector;

    private VisionPortal propVisionPortal;

    protected FirstVisionProcessor.Selected selectionOverride;

    private AprilTagProcessor tagProcessor;

    private VisionPortal tagsVisionPortal;

    boolean findTag = false;

    double strafeSpeed = 0.05;
    double directionToStrafe = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 28.0;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 18.0;    // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    double MMperIN = 25.4;
    int wheelDiaMM = 75;
    double wheelDiaIN = wheelDiaMM / MMperIN; //or input just inches as constant
    double wheelCircum = wheelDiaIN * Math.PI; //get circum (aka inches per wheel rev)
    int ultPlanHexEncoderTicks = 28; //ticks per motor rotation
    double threeToOne = 84 / 29; // real 3:1
    double fourToOne = 76 / 21; // real 4:1
    double drivetrainMotorGearRatio = threeToOne * fourToOne; //get gear ratio

    public double inchesPerTick() {
        return (wheelCircum / (drivetrainMotorGearRatio * ultPlanHexEncoderTicks)); //Inches per tick
        //return ((drivetrainMotorGearRatio * ultPlanHexEncoderTicks)/wheelCircum) * inches; //Ticks per inch
    }

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double DRIVE_SPEED = 0.45;     // Max driving speed for better distance accuracy.
    static final double SLOW_DRIVE_SPEED = 0.15;
    static final double FAST_DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.45;     // Max Turn speed to limit turn rate
    static final double SCORE_DRIVE_SPEED = 0.15;
    static final double SLOW_TURN_SPEED = 0.15;
    static final double FAST_TURN_SPEED = 0.6;
    static final double HEADING_THRESHOLD = 4.0;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable

    protected boolean isAutonomous = true;

    //this sets up for bulk reads!
    protected List<LynxModule> allHubs;

    protected void setupRobot() {
        // Initialize the drive system variables.
        leftDriveB = hardwareMap.get(DcMotor.class, "left_driveB");
        leftDriveF = hardwareMap.get(DcMotor.class, "left_driveF");
        rightDriveB = hardwareMap.get(DcMotor.class, "right_driveB");
        rightDriveF = hardwareMap.get(DcMotor.class, "right_driveF");
        //recognizer = new SignalSleeveRecognizer(hardwareMap, telemetry);
        //linearSlide = new LinearSlide(hardwareMap, telemetry, gamepad2);
        intake = new Intake(hardwareMap, telemetry, gamepad2);
        swingArm = new SwingArm(hardwareMap, telemetry, gamepad2, true);
        deliveryBoard = new DeliveryServoController();
        //swingArm = new SwingArm(hardwareMap, telemetry, gamepad2, isAutonomous);

        boolean isNear;
        boolean parkLeft;


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        21764:
        leftDriveB.setDirection(DcMotor.Direction.FORWARD);
        leftDriveF.setDirection(DcMotor.Direction.FORWARD);
        rightDriveB.setDirection(DcMotor.Direction.REVERSE);
        rightDriveF.setDirection(DcMotor.Direction.FORWARD);

//        11109:
//        leftDriveB.setDirection(DcMotor.Direction.REVERSE);
//        leftDriveF.setDirection(DcMotor.Direction.REVERSE);
//        rightDriveB.setDirection(DcMotor.Direction.FORWARD);
//        rightDriveF.setDirection(DcMotor.Direction.REVERSE);

        // TODO: Figure out if it's better to use a static variable for imu
        // and then avoid re-initializing it if you're in teleop mode and it already exists.
        // That way the current orientation is not reset when the opmode starts.

        // define initialization values for IMU, and then initialize it.

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        allHubs = hardwareMap.getAll(LynxModule.class);

        //sets up for bulk reads in manual mode! Read about it here: https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // By default the value is 250ms... we send data to Driver Station 4x per second.
        // We can see if our loop runs faster if we essentially disable telemetry by putting
        // a high number here.
        // Change this so that we keep telemetry on during init but disable it during run mode
        //telemetry.setMsTransmissionInterval(10000;)
    }

    /**
     * Checks on linear slide, four bar, and intake inside driving loops so that they can update themselves
     */


    protected void mechanismLoop() {
        intake.loop();
        swingArm.loop();
    }

    @Override
    public void runOpMode() {
        setupRobot();
        deliveryBoard.init(hardwareMap);
        // Wait for the game to start (Display Gyro value while waiting)
        propDetector = new FirstVisionProcessor();
        propVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), propDetector);
        while (opModeInInit()) {
            //telemetry.addData("", "Robot Heading = %4.0f", getRawHeading());
            telemetry.addData("bot heading:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("Identified", propDetector.getSelection());

            telemetry.addData("left front starting:", leftDriveF.getCurrentPosition());
            telemetry.addData("left back starting:", leftDriveB.getCurrentPosition());
            telemetry.addData("right front starting:", rightDriveF.getCurrentPosition());
            telemetry.addData("right back starting:", rightDriveB.getCurrentPosition());

            telemetry.addData("SwingArmPosition", swingArm.armMotor.getCurrentPosition());

            //telemetry.addData();

            telemetry.update();

            propDetector.getSelection();

            //parkOnly = true;

//            if (gamepad1.left_trigger > 0) {
//                isFar = false;
//            } else if (gamepad1.right_trigger > 0) {
//                isFar = true;
//            }

            //THE FOLLOWING OPTIONS ARE TOGGLEABLE
            //There's a 0.1 second delay between inputs to make controls work.

            //NOW TOGGLABLE!!!!!!111!!!1!
            if (gamepad1.right_trigger > 0) {
                if (isFar) {
                    isFar = false;
                    while (!gamepad1.atRest());
                } else {
                    isFar = true;
                    while (!gamepad1.atRest());
                }
            }

            if (gamepad1.right_bumper) {
                if (trianglePark) {
                    trianglePark = false;
                    while (!gamepad1.atRest());
                } else {
                    trianglePark = true;
                    while (!gamepad1.atRest());
                }
            }

//            if (gamepad1.x) {
//                isRed = false;
//            } else if (gamepad1.b) {
//                isRed = true;
//            }

            //haha togglable go brr
            if (gamepad1.b) {
                if (isRed) {
                    isRed = false;
                    while (!gamepad1.atRest());
                } else {
                    isRed = true;
                    while (!gamepad1.atRest());
                }
            }

        if (gamepad1.left_trigger > 0) {
            if (parkOnly) {
                parkOnly = false;
                while (!gamepad1.atRest());
            } else {
                parkOnly = true;
                while (!gamepad1.atRest());
            }
        }

        if (gamepad1.left_bumper) {
            if (stalling) {
                stalling = false;
                while (!gamepad1.atRest());
            } else {
                stalling = true;
                while (!gamepad1.atRest());
            }
        }

        //for testing (not legal in game):
//            if (gamepad1.left_bumper) {
//                selectionOverride = FirstVisionProcessor.Selected.LEFT;
//            }
//            else if (gamepad1.right_bumper) {
//                selectionOverride = FirstVisionProcessor.Selected.RIGHT;
//            }
//            else {
//                selectionOverride = FirstVisionProcessor.Selected.MIDDLE;
//            }
//            if (Overrideselection) {
//                telemetry.addData("OVERRIDE:", selectionOverride);
//            }


        String color;
        String side;
        String scoringPlan;
        String parkingPos;
        String stall;

        if (isRed) {
            color = "RED";
        } else {
            color = "BLUE";
        }

        if (isFar) {
            side = "FAR";
        } else {
            side = "CLOSE";
        }

        if (parkOnly) {
            scoringPlan = "IN PARKING";
        } else {
            scoringPlan = "ON BACKDROP";
        }

        if (trianglePark) {
            parkingPos = "TRIANGLE";
        } else {
            if (parkOnly) {
                parkingPos = "IN THE SQUARE HOLE";
            } else {
                parkingPos = "STAY AT BACKDROP";
            }
        }

        if (stalling) {
            stall = "YESSIR";
        } else {
            stall = "NO STALL?";
        }

        telemetry.addData("COLOR:", color);
        telemetry.addData("SIDE:", side);
        telemetry.addData("SCORING:", scoringPlan);
        telemetry.addData("PARKING:", parkingPos);
        telemetry.addData("WAITING FOR GOOFY AHH ALLIANCE PARTNER:", stall);
    }
        tbegin = (double) getRuntime();

        // Set the encoders for closed loop speed control, and reset the heading.
        leftDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();

        propVisionPortal.stopStreaming();

        runAutonomousProgram(isFar, parkOnly, trianglePark, stalling);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
        // Pause to display last telemetry message.
    }


    public void runAutonomousProgram(boolean isFar, boolean parkOnly, boolean trianglePark, boolean stalling) {

        swingArm.setPosition(armDrivePosID);

        telemetry.addData( "SwingArmPosition", swingArm.armMotor.getCurrentPosition());

//        while (true) {
//            while (!(gamepad1.a || gamepad1.b || gamepad1.y)) {
//            }
//            if (gamepad1.a) {
//                driveStraight(DRIVE_SPEED, -19, 0, false);
//            } else if (gamepad1.b) {
//                driveStraight(DRIVE_SPEED, -40, 0, false);
//            } else if (gamepad1.y) {
//                driveStraight(DRIVE_SPEED, -80, 0, false);
//            } else if (gamepad1.dpad_down) {
//                driveStraight(DRIVE_SPEED, -12, 0, false);
//            } else if (gamepad1.dpad_right) {
//                driveStraight(DRIVE_SPEED, -24, 0, false);
//            } else if (gamepad1.dpad_up) {
//                driveStraight(DRIVE_SPEED, -48, 0, false);
//            }
//        }
        propVisionPortal.close();

        tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .build();

        tagsVisionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        //move up to spike marks
        double distance = -18.0;
//        double distanceToTravel;
//        distanceToTravel = distance;

        FirstVisionProcessor.Selected selected = propDetector.selection;

        if(Overrideselection){
            selected = selectionOverride;
        }

        switch (selected){
            case LEFT:
                if(!isRed){
                aprilTagDriveDistance = aprilTagInitialDistance + aprilNearDistance;
                aprilTagIdNumber = 1;
                }
                else{
                aprilTagDriveDistance = aprilTagInitialDistance + aprilFarDistance;
                aprilTagIdNumber = 4;
                }
                break;
            case MIDDLE:
                aprilTagDriveDistance = aprilTagInitialDistance + aprilMidDistance;
                if(!isRed){aprilTagIdNumber = 2;}
                else{aprilTagIdNumber = 5;}
                break;
            case RIGHT:
                if(!isRed){
                aprilTagDriveDistance = aprilTagInitialDistance + aprilFarDistance;
                aprilTagIdNumber = 3;
                }
                else{
                aprilTagDriveDistance = aprilTagInitialDistance + aprilNearDistance;
                aprilTagIdNumber = 6;
                }
                break;
        }

        //push to corresponding spike mark
        switch (selected) {
            case LEFT:
                distance -= 7;
                driveStraight(DRIVE_SPEED, distance, 0.0, notMirrored);
                driveStraight(SLOW_DRIVE_SPEED, -2.0, 0.0, notMirrored);
                turnToHeading(TURN_SPEED, 45.0, notMirrored);
                driveStraight(DRIVE_SPEED, -14.0, 45.0, notMirrored);
                driveStraight(DRIVE_SPEED, 14.0, 45.0, notMirrored);
                turnToHeading(TURN_SPEED, 0.0, notMirrored);
                distance = (distance * -1) - 1;
                driveStraight(DRIVE_SPEED, distance, 0.0, notMirrored);
                break;
            case MIDDLE:
                distance -= 29;
                driveStraight(DRIVE_SPEED, distance, 0.0, notMirrored);
                //driveStraight(DRIVE_SPEED, (-distance + 10.0), 0.0, notMirrored);
                distance = (distance * -1) - 3.0;
                driveStraight(DRIVE_SPEED, distance, 0.0, notMirrored);
                break;
            case RIGHT:
                distance -= 7;
                driveStraight(DRIVE_SPEED, distance, 0.0, notMirrored);
                driveStraight(SLOW_DRIVE_SPEED, -2.0, 0.0, notMirrored);
                turnToHeading(TURN_SPEED, -45.0, notMirrored);
                driveStraight(DRIVE_SPEED, -14.0, -45.0, notMirrored);
                driveStraight(DRIVE_SPEED, 14.0, -45.0, notMirrored);
                turnToHeading(TURN_SPEED, 0.0, notMirrored);
                distance = (distance * -1) - 1;
                driveStraight(DRIVE_SPEED, distance, 0.0, notMirrored);
                break;
            case NONE:
                distance -= 29;
                driveStraight(DRIVE_SPEED, distance, 0.0, notMirrored);
                //driveStraight(DRIVE_SPEED, (-distance + 10.0), 0.0, notMirrored);
                distance = (distance * -1) - 3.0;
                driveStraight(DRIVE_SPEED, distance, 0.0, notMirrored);
                break;
        }
        /*
        if(trianglePark && !parkOnly){
        remainingDistanceToTriangePark = distanceToDriveToTrianglePark - AprilTagDriveDistance
            if(weScoredYellow){
            driveStraight(remainingDistanceToTriangePark}
            }

        */
        distance = -50;

        if (isFar) {
            distance -= 84;
        }
        if (parkOnly) {
            if (!trianglePark) {
                if (!stalling) {
                    distance -= 26; //used to be 12
                }
            }
        }
        if (stalling) {
            distance += 48;
        }

        if (parkOnly) { //since we're only parking, let's drive forward facing and score the yellow pixel.
            turnToHeading(TURN_SPEED, -90.0, isMirrored);
            distance *= -1;
            driveStraight(DRIVE_SPEED, distance, -90.0, isMirrored);
            if (stalling) {
                if (trianglePark) {
                    stall(tbegin, 10); //change these numbers later
                    driveStraight(FAST_DRIVE_SPEED, 48, -90.0, isMirrored); //remaining distance to get to line
                } else {
                    stall(tbegin, 8); //change these numbers later
                    driveStraight(DRIVE_SPEED, 64.0, -90.0, isMirrored); //remaining distance to park in square
                }
            }
            if (trianglePark) {
                turnToHeading(FAST_TURN_SPEED, 0.0, notMirrored);
                driveStraight(FAST_DRIVE_SPEED, -80.0, 0.0, notMirrored);
                turnToHeading(FAST_TURN_SPEED, -90.0, isMirrored);
                driveStraight(FAST_DRIVE_SPEED, 27.0, -90.0, isMirrored);
            }
            intake.intakeMotor.setPower(-1);
            driveStraight(SLOW_DRIVE_SPEED, -3.0, -90.0, isMirrored);
            intake.intakeMotor.setPower(0);
        }
        // if it's scoring on the backdrop with april tags:
        else if (!parkOnly) {
            turnToHeading(TURN_SPEED, 90.0, isMirrored);
            driveStraight(DRIVE_SPEED, distance, 90.0, isMirrored);
            if (stalling) {
                if (trianglePark) {
                    stall(tbegin, 10); //change these numbers later
                    driveStraight(DRIVE_SPEED, -26, 90.0, isMirrored); //remaining distance
                } else {
                    stall(tbegin, 8); //change these numbers later
                    driveStraight(DRIVE_SPEED, -64, 90.0, isMirrored); //remaining distance
                }
            }
            turnToHeading(TURN_SPEED, 180.0, notMirrored);
            swingArm.setPosition(armCarryID);
            driveStraight(DRIVE_SPEED, aprilTagDriveDistance, 180.0, notMirrored);
            turnToHeading(TURN_SPEED, 90.0, isMirrored);
            holdHeading(TURN_SPEED, 90.0, 1.0, isMirrored);
            //driveStraight(DRIVE_SPEED, driveBackToSeeAprilTag, 90.0, notMirrored);

            ElapsedTime strafeCorrectionTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            int correctionTime = 3;
            strafeCorrectionTimer.reset();
            AprilTagDetection tag = null;

            while (opModeIsActive() && strafeCorrectionTimer.time() <= correctionTime) {
                if (tagProcessor.getDetections().size() > 0) {

                    List<AprilTagDetection> currentDetections;
                    currentDetections = tagProcessor.getDetections();

                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.id == aprilTagIdNumber) {
                            tag = detection;
                        }
                    }
                    if (tag == null) {
                        tag = tagProcessor.getDetections().get(0);
                    }

                    telemetry.addLine("I CAN SEE! HELLO WORLD!!!");
                    telemetry.addData("LOOKING FOR", aprilTagIdNumber);
                    telemetry.addData("TAGS SEEN", tag.id);
                    telemetry.addLine(String.format("XYZ %6.2f %6.2f %6.2f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                    telemetry.addLine(String.format("RPY %6.2f %6.2f %6.2f", tag.ftcPose.roll, tag.ftcPose.pitch, tag.ftcPose.yaw));
                    telemetry.addData("STRAFE TIME ELAPSED: ", strafeCorrectionTimer.time());
                    telemetry.update();
                    if (tag.id == aprilTagIdNumber) {
                        aprilTag_AdjustedX = tag.ftcPose.x - aprilTag_CenterGoal;
                    }
                    RFLBSpeed = strafeSpeed * (aprilTag_AdjustedX / Math.abs(aprilTag_AdjustedX));
                    LFRBSpeed = -1 * RFLBSpeed;
                    while (Math.abs(aprilTag_AdjustedX) > aprilTag_Threshold && strafeCorrectionTimer.time() <= correctionTime && !strafeCorrectionDone) {
                        leftDriveF.setPower(LFRBSpeed);
                        rightDriveB.setPower(LFRBSpeed);
                        rightDriveF.setPower(RFLBSpeed);
                        leftDriveB.setPower(RFLBSpeed);
                        telemetry.addLine("CORRECTING!!!");
                        telemetry.addData("TARGET TAGS SEEN", tag.id);
                        telemetry.addLine(String.format("XYZ %6.2f %6.2f %6.2f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                        telemetry.addLine(String.format("RPY %6.2f %6.2f %6.2f", tag.ftcPose.roll, tag.ftcPose.pitch, tag.ftcPose.yaw));
                        telemetry.addData("STRAFE TIME ELAPSED: ", strafeCorrectionTimer.time());
                        telemetry.update();
                        if (tagProcessor.getDetections().size() > 0) {
                            currentDetections = tagProcessor.getDetections();

                            for (AprilTagDetection detection : currentDetections) {
                                if (detection.id == aprilTagIdNumber) {
                                    tag = detection;
                                }
                            }
                            if (tag == null) {
                                tag = tagProcessor.getDetections().get(0);
                            }
                            if (tag.id == aprilTagIdNumber) {
                                aprilTag_AdjustedX = tag.ftcPose.x - aprilTag_CenterGoal;
                            }
                            RFLBSpeed = strafeSpeed * (aprilTag_AdjustedX / Math.abs(aprilTag_AdjustedX));
                            LFRBSpeed = -1 * RFLBSpeed;
                        } else {
                            leftDriveF.setPower(0);
                            rightDriveB.setPower(0);
                            rightDriveF.setPower(0);
                            leftDriveB.setPower(0);
                            //probably want to do something else here too but this is only if you see nothing
                        }
                    }
                    strafeCorrectionDone = true;
                    leftDriveF.setPower(0);
                    rightDriveB.setPower(0);
                    rightDriveF.setPower(0);
                    leftDriveB.setPower(0);
                }
            }
            tagsVisionPortal.close();
//            TROUBLESHOOT AND IMPLEMENT LATER:

//            if (tagProcessor.getDetections().size() > 0) {
//                driveDistanceToScore = (double)(Math.sqrt(Math.pow(tag.ftcPose.y, 2) - 81) - 1.5); //pythagorean theorem based off of robot height and tag y value. -1.5 inches to get to black line.
//                driveDistanceToScore = -1 * (driveDistanceToScore - distanceToScoreFromTags);
//                driveDistanceToScore *= (5/3);
//                //driveDistanceToScore = -1 * (tag.ftcPose.z - distanceToScoreFromTags);
//            } else {

            //driveDistanceToScore = -16.0; //roughly
//            }
            leftDriveF.setPower(0);
            rightDriveB.setPower(0);
            rightDriveF.setPower(0);
            leftDriveB.setPower(0);
            turnToHeading(TURN_SPEED, 90.0, isMirrored);
            swingArm.setPosition(armDeliveryID);
            driveStraight(SCORE_DRIVE_SPEED, driveDistanceToScore, 90.0, isMirrored);
            ElapsedTime rotationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            int fullRotationTime = 1809 * 2;
            rotationTimer.reset();
            while (rotationTimer.time() < fullRotationTime) { //double 360 rotation to lose the pixel
                deliveryBoard.setServoPosition(0);
            }
            deliveryBoard.setServoPosition(0.5);
            driveStraight(DRIVE_SPEED, 2.0, 90.0, isMirrored);
            swingArm.setPosition(armCarryID);
            if (trianglePark) {
                scoringDistanceToTrianglePark = wallToTriangleDriveDistance - aprilTagDriveDistance;
                turnToHeading(TURN_SPEED, 180, notMirrored);
                driveStraight(DRIVE_SPEED, scoringDistanceToTrianglePark, 180, notMirrored);
                turnToHeading(TURN_SPEED, 90, isMirrored);
                driveStraight(DRIVE_SPEED, -12, 180, notMirrored);
            }
            //DO MORE STUFF!!
        }


           /* if (!parkOnly && isBlue){
                turnToHeading(TURN_SPEED, 0.0, notMirrored)
                if(SELECTION.LEFT){
           driveStraight(DRIVE_SPEED, 12 + 6, heading, notMirrored)
           }
                else if(SELECTION.MIDDLE){
                driveStraight(DRIVE_SPEED, 12 + 12, heading, notMirrored)
                }
                else (SELECTION.RIGHT){
                driveStraight(DRIVE_SPEED, 12 + 18, heading, notMirrored)
              }
           }
            if (!parkOnly && isRed){
                turnToHeading(TURN_SPEED, 0.0, notMirrored)
                if(SELECTION.RIGHT){
                    driveStraight(DRIVE_SPEED, 12 + 6, heading, notMirrored)
                }
                else if(SELECTION.MIDDLE){
                    driveStraight(DRIVE_SPEED, 12 + 12, heading, notMirrored)
                }
                else(SELECTION.LEFT){
                    driveStraight(DRIVE_SPEED, 12 + 18, heading, notMirrored)
                }
            }
            */
//            if (trianglePark) {
//                distance = -32.0;
//            } else {
//                distance = 32.0;
//            }
//            ElapsedTime strafeTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
//            int strafeTime = 6;
//            strafeTimer.reset();
//            if (isRed) {
//                while (!findTag && strafeTimer.time() <= strafeTime) {
//                    startStrafe(strafeSpeed, 90.0, "right", isMirrored);
//                    if (tagProcessor.getDetections().size() > 0) {
//                        AprilTagDetection tag = tagProcessor.getDetections().get(0);
//
//                        telemetry.addData("ID", tag.id);
//
//                        telemetry.addLine(String.format("XYZ %6.2f %6.2f %6.2f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
//
//                        telemetry.addLine(String.format("RPY %6.2f %6.2f %6.2f", tag.ftcPose.roll, tag.ftcPose.pitch, tag.ftcPose.yaw));
//
//                        telemetry.addData("STRAFE TIME ELAPSED: ", strafeTimer.time());
//
//                        telemetry.update();
//
//                        startStrafe(strafeSpeed, 90.0, "right", isMirrored);
//
//                        switch (selected) {
//                            case LEFT:
//                                if (tag.id == 4 && (tag.ftcPose.x < 0.5 && tag.ftcPose.x > -0.5)) { //>= 0?
//                                    stopStrafe();
//                                    turnToHeading(TURN_SPEED, 0.0, notMirrored);
//                                    distance -= 6;
//                                    findTag = true;
//                                }
//                            case MIDDLE:
//                                if (tag.id == 5 && (tag.ftcPose.x < 0.5 && tag.ftcPose.x > -0.5)) {
//                                    stopStrafe();
//                                    turnToHeading(TURN_SPEED, 0.0, notMirrored);
//                                    findTag = true;
//                                }
//                            case RIGHT:
//                                if (tag.id == 6 && (tag.ftcPose.x < 0.5 && tag.ftcPose.x > -0.5)) {
//                                    stopStrafe();
//                                    turnToHeading(TURN_SPEED, 0.0, notMirrored);
//                                    distance += 6;
//                                    findTag = true;
//                                }
//                            case NONE:
//                                if (tag.id == 5 && (tag.ftcPose.x < 0.5 && tag.ftcPose.x > -0.5)) {
//                                    stopStrafe();
//                                    turnToHeading(TURN_SPEED, 0.0, notMirrored);
//                                    findTag = true;
//                                }
//                        }
//                    }
//                }
//            } else { //else it must be blue
//                while (!findTag && strafeTimer.time() <= strafeTime) {
//                    startStrafe(strafeSpeed, 90.0, "left", isMirrored);
//                    if (tagProcessor.getDetections().size() > 0) {
//                        AprilTagDetection tag = tagProcessor.getDetections().get(0);
//
//                        telemetry.addData("ID", tag.id);
//
//                        telemetry.addLine(String.format("XYZ %6.2f %6.2f %6.2f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
//
//                        telemetry.addLine(String.format("RPY %6.2f %6.2f %6.2f", tag.ftcPose.roll, tag.ftcPose.pitch, tag.ftcPose.yaw));
//
//                        telemetry.addData("STRAFE TIME ELAPSED: ", strafeTimer.time());
//
//                        telemetry.update();
//
//                        startStrafe(strafeSpeed, 90.0, "left", isMirrored);
//
//                        switch (selected) {
//                            case LEFT:
//                                if (tag.id == 1 && (tag.ftcPose.x < 0.5 && tag.ftcPose.x > -0.5)) { //<= 0?
//                                    stopStrafe();
//                                    turnToHeading(TURN_SPEED, 0.0, notMirrored);
//                                    distance -= 6;
//                                    findTag = true;
//                                }
//                            case MIDDLE:
//                                if (tag.id == 2 && (tag.ftcPose.x < 0.5 && tag.ftcPose.x > -0.5)) {
//                                    stopStrafe();
//                                    turnToHeading(TURN_SPEED, 0.0, notMirrored);
//                                    findTag = true;
//                                }
//                            case RIGHT:
//                                if (tag.id == 3 && (tag.ftcPose.x < 0.5 && tag.ftcPose.x > -0.5)) {
//                                    stopStrafe();
//                                    turnToHeading(TURN_SPEED, 0.0, notMirrored);
//                                    distance += 6;
//                                    findTag = true;
//                                }
//                            case NONE:
//                                if (tag.id == 2 && (tag.ftcPose.x < 0.5 && tag.ftcPose.x > -0.5)) {
//                                    stopStrafe();
//                                    turnToHeading(TURN_SPEED, 0.0, notMirrored);
//                                    findTag = true;
//                                }
//                        }
//                    }
//                }
//            }
//            turnToHeading(TURN_SPEED, 0.0, notMirrored);
//            driveStraight(DRIVE_SPEED, distance, 0.0, notMirrored);
//            turnToHeading(TURN_SPEED, 90.0, isMirrored);
//            driveStraight(DRIVE_SPEED, -38.0, 90.0, isMirrored);
//
//
//            /*
//
//            aprilTag_CenterGoal = 0; // zero or how far to the left or right we want to be
//            aprilTag_Threshold = 0.5;
//            aprilTag_AdjustedX = 0; // this will be our adjusted value off center goal of ftcPose.X
//            LFRBSpeed = 0;
//            RFLBSpeed = 0;
//            if(!parkOnly){
//                driveStraight(DRIVE_SPEED, aprilTagDriveDistance, 0.0, notMirrored)
//                turnToHeading(TURN_SPEED, 90.0, isMirrored)
//                driveStraight(DRIVE_SPEED, backupToSeeAprilTag, 90.0, notMirrored}
//                if(tag.id == aprilTagIdNumber){aprilTag_AdjustedX = tag.ftcPose.x - aprilTag_CenterGoal
//                RFLBSpeed = strafeSpeed * (aprilTag_AdjustedX/Math.abs(aprilTag_AdjustedX))
//                LFRBSpeed = -1 * RFLBSpeed
//                while(Math.abs(aprilTag_AdjustedX) > aprilTag_Threshold){
//                    leftDriveF.setpower(LFRBSpeed)
//                    rightDriveB.setpower(LFRBSpeed)
//                    rightDriveF.setpower(RFLBSpeed)
//                    leftDriveB.setpower(RFLBSpeed)
//                    if(tag.id == aprilTagIdNumber){aprilTag_AdjustedX = tag.ftcPose.x - aprilTag_CenterGoal}
//                }
//                leftDriveF.setpower(0)
//                rightDriveB.setpower(0)
//                rightDriveF.setpower(0)
//                leftDriveB.setpower(0)
//                driveStraight(DRIVE_SPEED, 1, 90, notMirrored)
//                swingArm.setposition(armDelivery)
//                set
//                }
//                */
//        }
//        if (parkOnly) { //since we're only parking, let's drive forward facing and score the yellow pixel.
//            distance *= -1;
//            driveStraight(DRIVE_SPEED, distance, -90.0, isMirrored);
//            if (trianglePark) {
//                turnToHeading(TURN_SPEED, 0.0, notMirrored);
//                driveStraight(DRIVE_SPEED, -80.0, 0.0, notMirrored);
//                turnToHeading(TURN_SPEED, -90.0, isMirrored);
//                driveStraight(DRIVE_SPEED, 35.0, -90.0, isMirrored);
//            }
//            intake.intakeMotor.setPower(-1);
//            driveStraight(SLOW_DRIVE_SPEED, 3, -90.0, isMirrored);
//            intake.intakeMotor.setPower(0);
//        } else {
//            driveStraight(DRIVE_SPEED, distance, 0.0, notMirrored);
//            turnToHeading(TURN_SPEED, 90.0, isMirrored);
//            driveStraight(DRIVE_SPEED, -38.0, 90.0, isMirrored);
//        }

        swingArm.setPosition(armHardStopID);
        telemetry.addData( "SwingArmPosition", swingArm.armMotor.getCurrentPosition());

        if (parkOnly) {
            holdHeading(TURN_SPEED, -90, 1.5, isMirrored);
        } else {
            holdHeading(TURN_SPEED, 90, 1.5, isMirrored);
        }
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     * Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading,
                              boolean isMirrored) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            if (isMirrored && isRed) {
                heading *= -1;
            }

            //reverse the heading if you start on the left side. this turns a right heading into a left heading and vice versa.
            //heading = heading * reverseTurnsForAllianceColor;

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance / inchesPerTick());

            leftTargetF = leftDriveF.getCurrentPosition() + moveCounts;
            leftTargetB = leftDriveB.getCurrentPosition() + moveCounts;
            rightTargetF = rightDriveF.getCurrentPosition() + moveCounts;
            rightTargetB = rightDriveB.getCurrentPosition() + moveCounts;

//            telemetry.addData("left front moved:", leftDriveF.getCurrentPosition());
//            telemetry.addData("left back moved:", leftDriveB.getCurrentPosition());
//            telemetry.addData("right front moved:", rightDriveF.getCurrentPosition());
//            telemetry.addData("right back moved:", rightDriveB.getCurrentPosition());

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftDriveF.setTargetPosition(leftTargetF);
            leftDriveB.setTargetPosition(leftTargetB);
            rightDriveF.setTargetPosition(rightTargetF);
            rightDriveB.setTargetPosition(rightTargetB);

            telemetry.addData("driveStraight", "opModeIsActive");
            telemetry.addData("targetPositions", "%d : %d : %d : %d", leftTargetF, leftTargetB, rightTargetF, rightTargetB);
            telemetry.addData("move counts:", moveCounts);
            telemetry.update();

            leftDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("maxDriveSpeed", maxDriveSpeed);
            telemetry.addData("active", opModeIsActive());
            telemetry.addData("ldf", leftDriveF.isBusy());
            telemetry.addData("rdf", rightDriveF.isBusy());
            telemetry.addData("ldb", leftDriveB.isBusy());
            telemetry.addData("rdb", rightDriveB.isBusy());
            // Unfortunately we need this because sometimes the motor hasn't recognized yet that it's busy!!
            //sleep(1000);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);
            driveStraightLoops += 1;

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive()) {

                telemetry.addData("driveStraight", "opModeIsActive and all motors are busy!");
                telemetry.addData("drive straight loops: ", driveStraightLoops);

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                mechanismLoop();

                // Display drive status for the driver.
                sendTelemetry(true);

                clearBulkCache();

                // Check if ALL motors report not busy
                if (!(leftDriveF.isBusy() || rightDriveF.isBusy() || leftDriveB.isBusy() || rightDriveB.isBusy())) {
                    break;
                }
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to start strafe for april tag detection
     * @param strafeSpeed speed to strafe at
     * @param heading heading to maintain while strafing
     * @param direction direction to strafe string ("left" or "right")
     * @param isMirrored mirroring for red/blue side?
     */
    public void startStrafe(double strafeSpeed, double heading, String direction, boolean isMirrored) {
        if (opModeIsActive()) {
//            ElapsedTime holdTimer = new ElapsedTime();
//            holdTimer.reset();

            if (isMirrored && isRed) {
                heading *= -1;
            }

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -0.5, 0.5);
            // Pivot in place by applying the turning correction
            strafeMoveRobot(direction, strafeSpeed, turnSpeed);

            Range.clip(strafeSpeed, 0, 1.0);
            if (direction == "left") {
                leftDriveF.setPower(-strafeSpeed);
                leftDriveB.setPower(strafeSpeed);
                rightDriveF.setPower(strafeSpeed);
                rightDriveB.setPower(-strafeSpeed);
            } else { //direction must be right
                leftDriveF.setPower(strafeSpeed);
                leftDriveB.setPower(-strafeSpeed);
                rightDriveF.setPower(-strafeSpeed);
                rightDriveB.setPower(strafeSpeed);
            }
            clearBulkCache();
            //mechanismLoop();
        }
    }

    /**
     * Method to stop strafe (and all motor movement)
     */
    public void stopStrafe() {
        leftDriveF.setPower(0);
        leftDriveB.setPower(0);
        rightDriveF.setPower(0);
        rightDriveB.setPower(0);
        clearBulkCache();
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading, boolean isMirrored) {

        //reverse the heading if you start on the left side. this turns a right turn into a left turn and vice versa.
        if (isMirrored && isRed) {
            heading *= -1;
        }

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            mechanismLoop();

            // Display drive status for the driver.
            sendTelemetry(false);

            clearBulkCache();
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     * This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime, boolean reverseSides) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        if (reverseSides && isRed) {
            heading *= -1;
        }

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            mechanismLoop();

            // Display drive status for the driver.
            sendTelemetry(false);

            clearBulkCache();

        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    public void stall(double tbegin, int time) {
        while ((double) (30 - (getRuntime() - tbegin)) > time) {
            //runtime.reset();
            telemetry.addData("RUN TIME REMAINING:", (double) (30 - (getRuntime() - tbegin)));
            telemetry.addData("STALL TIME REMAINING:", (double) ((30 - (getRuntime() - tbegin)) - time));
            telemetry.update();
        }
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftDriveF.setPower(leftSpeed);
        leftDriveB.setPower(leftSpeed);
        rightDriveF.setPower(rightSpeed);
        rightDriveB.setPower(rightSpeed);
    }

    public void strafeMoveRobot(String direction, double drive, double turn) {
        //driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        //turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        frontSpeed = drive + turn;
        backSpeed = drive - turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        if (direction == "left") {
            leftDriveF.setPower(-frontSpeed);
            leftDriveB.setPower(backSpeed);
            rightDriveF.setPower(frontSpeed);
            rightDriveB.setPower(-backSpeed);
        } else {
            leftDriveF.setPower(frontSpeed);
            leftDriveB.setPower(-backSpeed);
            rightDriveF.setPower(-frontSpeed);
            rightDriveB.setPower(backSpeed);
        }
    }

    protected void clearBulkCache() {
        // Clears the cache so that it will be refreshed the next time you ask for a sensor value!
        // Be very sure that this gets called in every loop in your code!!!!
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    /**
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) { //SHOULD BE LABELED SOMETHING LIKE "driveSendTelemetry"
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos LF:RF:LB:RB", "%7d:%7d:%7d:%7d",
                    leftTargetF, rightTargetF, leftTargetB, rightTargetB);
            telemetry.addData("Actual Pos LF:RF:LB:RB", "%7d:%7d:%7d:%7d", leftDriveF.getCurrentPosition(),
                    rightDriveF.getCurrentPosition(), leftDriveB.getCurrentPosition(), rightDriveB.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);

        telemetry.addData("drive straight loops: ", driveStraightLoops);
        telemetry.addData("current driveSpeed value: ", driveSpeed);

        //checks the time spent on the loop and adds it to telemetry

        telemetry.addData("Loop Time", (int) runtime.milliseconds());

        runtime.reset();

        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double botHeading = orientation.getYaw(AngleUnit.DEGREES);  // + 180.0; // +/- 180 to flip heading
        return botHeading;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    // TODO: can we rename these moveTo* functions for setHeightTo*
    public void moveToGroundPosition() {
        //linearSlide.setPosition(0);
        //swingArm.setPosition(1);
    }

    public void moveToLowPosition() {
        //linearSlide.setPosition(3);
        //swingArm.setPosition(1);
    }

    public void moveToMediumPosition() {
        //linearSlide.setPosition(1);
        //swingArm.setPosition(2);
    }

    public void moveToHighPosition() {
        //linearSlide.setPosition(3);
        //swingArm.setPosition(2);
    }

    public void moveToIntakePosition() {
        //linearSlide.setPosition(2);
        //swingArm.setPosition(1);
    }

    //Step 1: detect the team prop
//    TestAutonomous.SpikeMark teamPropMark = detectTeamProp();
//    //Step 2: drive to the team prop
//    //driveToCorrectSpikeMark(teamPropMark);
//    //Step 3: place purple pixel on same line as team prop
//    //ejectPurplePixel();
//    //Step 4: return to original position
//    //driveFromSpikeMark(teamPropMark);
//    //Step 5: drive under truss closest to wall to get to backdrop
//    //driveToBackdrop(isNear);
//    //Step 6: place yellow pixel on correct Apriltag
//    //depositYellowPixel(teamPropMark);
//    //Step 7: drive off to side
//    //parkInBackstage(parkLeft);
//
//    void driveForwardInches(double amount) {
//    }
//    void turnDegrees(double amount) {
//    }
//    enum SpikeMark {RIGHT, LEFT, CENTER}
//
//    TestAutonomous.SpikeMark detectTeamProp() {
//        return TestAutonomous.SpikeMark.CENTER;
//        //
//    }
//
//    final double SPIKE_MARK_DECISION_DISTANCE = -1;
//    final double CENTER_SPIKE_MARK_INCHES = -1;
//    final double OFFCENTER_SPIKE_MARK_INCHES = -1;
//    final double OFFCENTER_DECISION_TURN_DEGREES = -1;
//
//    void driveToCorrectSpikeMark(TestAutonomous.SpikeMark teamPropMark) {
//        driveForwardInches(SPIKE_MARK_DECISION_DISTANCE);
//
//        if (teamPropMark == TestAutonomous.SpikeMark.CENTER) {
//            driveForwardInches(CENTER_SPIKE_MARK_INCHES);
//        } else if (teamPropMark == TestAutonomous.SpikeMark.LEFT) {
//            turnDegrees(OFFCENTER_DECISION_TURN_DEGREES);
//            driveForwardInches(OFFCENTER_SPIKE_MARK_INCHES);
//        }
//        else if (teamPropMark == TestAutonomous.SpikeMark.RIGHT) {
//            turnDegrees(-OFFCENTER_DECISION_TURN_DEGREES);
//            driveForwardInches(OFFCENTER_SPIKE_MARK_INCHES);
//        }
//    }
//
//    void ejectPurplePixel() {
//        intake.ejectPixel();
//    }
//
//    void driveFromSpikeMark(TestAutonomous.SpikeMark teamPropMark) {
//        driveForwardInches(-SPIKE_MARK_DECISION_DISTANCE);
//
//        if (teamPropMark == TestAutonomous.SpikeMark.CENTER) {
//            driveForwardInches(-CENTER_SPIKE_MARK_INCHES);
//        } else if (teamPropMark == TestAutonomous.SpikeMark.LEFT) {
//            turnDegrees(-OFFCENTER_DECISION_TURN_DEGREES);
//            driveForwardInches(-OFFCENTER_SPIKE_MARK_INCHES);
//        }
//        else if (teamPropMark == TestAutonomous.SpikeMark.RIGHT) {
//            turnDegrees(OFFCENTER_DECISION_TURN_DEGREES);
//            driveForwardInches(-OFFCENTER_SPIKE_MARK_INCHES);
//        }
//    }
//
//    void driveToBackdrop(boolean isNear) {
//    }
//
//    void depositYellowPixel(TestAutonomous.SpikeMark teamPropMark) {
//    }
//
//    void parkInBackstage(boolean parkLeft) {
//    }
//

}





