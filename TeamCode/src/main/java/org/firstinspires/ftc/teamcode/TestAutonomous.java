/////* -Pseudocode Notes-
////
////Encoders:
////UltraPlanetary motor: 28 cpr (counts per rotation)
////Core Hex motor encoder: 4 cpr @ motor, 288 cpr @ output
////
////Ratios:
////3:1 real - 84:29
////4:1 real - 76:21
////5:1 real - 68:13
////
////Math for distance function:
////Wheel diameter (mm) * PI / Gear ratio / Encoder ticks * mm. to in.
////
////Functions:
////turn(int deg) must turn the robot deg degrees. It should have a variability value of about 1 degree.
////move(String dir, double dist) moves the robot dist inches. It should account for speed-up and ramp-down of the motors.
////
////constants:
////
////MMperIN = 25.4
////wheelDiaMM = 75
////wheelDiaIN = wheelDiaMM / MMperIN //or input just inches as constant
////wheelCircum = wheelDiaIN * Pi() //import math lib later
////ultPlanHexEncoderTicks = 28
////3to1 = 84/29 // real 3:1
////4to1 = 76/21 // real 4:1
////drivetrainMotorGearRatio = 3to1 * 4to1
////
////public double inchesToTicks(inches) {
////    (wheelCircum/(drivetrainMotorGearRatio * ultPlanHexEncoderTicks)) * inches
////}
////
////
////
//// */
//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@Autonomous(name="CenterStage Auto Test", group="Robot")
////@Disabled
//public class TestAutonomous extends LinearOpMode {
//
//    boolean isNear;
//    boolean parkLeft;
//    Intake intake = new Intake();
//    private DcMotor leftDriveFront = null;
//    private DcMotor rightDriveFront = null;
//    private DcMotor leftDriveBack = null;
//    private DcMotor rightDriveBack = null;
//
//    private int leftTargetF = 0;
//    private int leftTargetB = 0;
//    private int rightTargetF = 0;
//    private int rightTargetB = 0;
//
//    int moveCounts = 0;
//
//    static final double FORWARD_SPEED = 0.6;
//    static final double TURN_SPEED = 0.5;
//    // These values are from the sample code, we can change them later
//
//    //    void turn(String dir, int time) {
////    //Make turning rely on absolute cardinal direction instead of turn time (later)
////        if (dir == "l") {
////            for (int i = 0; i < time; i++) {
////                leftDriveFront.setPower(-TURN_SPEED);
////                leftDriveBack.setPower(-TURN_SPEED);
////                rightDriveFront.setPower(TURN_SPEED);
////                rightDriveBack.setPower(TURN_SPEED);
////            }
////            leftDriveFront.setPower(0);
////            leftDriveBack.setPower(0);
////            rightDriveFront.setPower(0);
////            rightDriveBack.setPower(0);
////        }
////        else {
////            for (int i = 0; i < time; i++) {
////                leftDrive.setPower(TURN_SPEED);
////                rightDrive.setPower(-TURN_SPEED);
////            }
////            leftDrive.setPower(0);
////            rightDrive.setPower(0);
////        }
////    }
//    double MMperIN = 25.4;
//    int wheelDiaMM = 75;
//    double wheelDiaIN = wheelDiaMM / MMperIN; //or input just inches as constant
//    double wheelCircum = wheelDiaIN * 3.14; //Pi(); import math lib later
//    int ultPlanHexEncoderTicks = 28; //ticks per motor rotation
//    double threeToOne = 84 / 29; // real 3:1
//    double fourToOne = 76 / 21; // real 4:1
//    double drivetrainMotorGearRatio = threeToOne * fourToOne; //get gear ratio
//
//    public double inchesPerTick() {
//        return (wheelCircum / (drivetrainMotorGearRatio * ultPlanHexEncoderTicks)); //Inches per tick
//        //return ((drivetrainMotorGearRatio * ultPlanHexEncoderTicks)/wheelCircum) * inches; //Ticks per inch
//    }
//
//    //define target motor positions by adding ticks to go to absolute tick position
//    void setStraightTarget(int moveCounts) {
//        leftTargetF = leftDriveFront.getCurrentPosition() + moveCounts;
//        leftTargetB = leftDriveBack.getCurrentPosition() + moveCounts;
//        rightTargetF = rightDriveFront.getCurrentPosition() + moveCounts;
//        rightTargetB = rightDriveBack.getCurrentPosition() + moveCounts;
//        //set targets
//        leftDriveFront.setTargetPosition(leftTargetF);
//        leftDriveBack.setTargetPosition(leftTargetB);
//        rightDriveFront.setTargetPosition(rightTargetF);
//        rightDriveBack.setTargetPosition(rightTargetB);
//    }
//
//    void powerMotors(double driveSpeed) {
//        leftDriveFront.setPower(driveSpeed);
//        leftDriveBack.setPower(driveSpeed);
//        rightDriveFront.setPower(driveSpeed);
//        rightDriveBack.setPower(driveSpeed);
//    }
//
//    void driveToTarget() {
//        //run to targets
//        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        powerMotors(0.5);
//    }
//
//    void move(int Direction, double inchesToMove) {
//        //
//        double ticksToGo = inchesToMove / inchesPerTick(); //get amount of ticks to go for inches to move using helper function
//
//        int moveCounts = (int) (ticksToGo) * Direction; //convert to inches
//
//        setStraightTarget(moveCounts); //pass ticks for distance to move through target function
//        driveToTarget(); //drive motors to encoder target
//    }
//
//    public void runOpMode() {
//        while (opModeInInit()) {
//            telemetry.addLine("waiting...");
//            telemetry.update();
//        }
//        leftDriveFront = hardwareMap.get(DcMotor.class, "left_driveF");
//        rightDriveFront = hardwareMap.get(DcMotor.class, "right_driveF");
//        leftDriveBack = hardwareMap.get(DcMotor.class, "left_driveB");
//        rightDriveBack = hardwareMap.get(DcMotor.class, "right_driveB");
//
//        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
//        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
//        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
//        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
//        //Fixed motor directions (21764 had reversed motor) - this would be changed back to normal this year's robot
////        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        move(1, 48);
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
////        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //Step 1: detect the team prop
//        SpikeMark teamPropMark = detectTeamProp();
//        //Step 2: drive to the team prop
//        driveToCorrectSpikeMark(teamPropMark);
//        //Step 3: place purple pixel on same line as team prop
//        ejectPurplePixel();
//        //Step 4: return to original position
//        driveFromSpikeMark(teamPropMark);
//        //Step 5: drive under truss closest to wall to get to backdrop
//        driveToBackdrop(isNear);
//        //Step 6: place yellow pixel on correct Apriltag
//        depositYellowPixel(teamPropMark);
//        //Step 7: drive off to side
//        parkInBackstage(parkLeft);
//    }
//
//    void driveForwardInches(double amount) {
//    }
//    void turnDegrees(double amount) {
//    }
//    enum SpikeMark {RIGHT, LEFT, CENTER}
//
//    SpikeMark detectTeamProp() {
//        return SpikeMark.CENTER;
//        //
//    }
//
//    final double SPIKE_MARK_DECISION_DISTANCE = -1;
//    final double CENTER_SPIKE_MARK_INCHES = -1;
//    final double OFFCENTER_SPIKE_MARK_INCHES = -1;
//    final double OFFCENTER_DECISION_TURN_DEGREES = -1;
//
//    void driveToCorrectSpikeMark(SpikeMark teamPropMark) {
//        driveForwardInches(SPIKE_MARK_DECISION_DISTANCE);
//
//        if (teamPropMark == SpikeMark.CENTER) {
//            driveForwardInches(CENTER_SPIKE_MARK_INCHES);
//        } else if (teamPropMark == SpikeMark.LEFT) {
//            turnDegrees(OFFCENTER_DECISION_TURN_DEGREES);
//            driveForwardInches(OFFCENTER_SPIKE_MARK_INCHES);
//        }
//        else if (teamPropMark == SpikeMark.RIGHT) {
//            turnDegrees(-OFFCENTER_DECISION_TURN_DEGREES);
//            driveForwardInches(OFFCENTER_SPIKE_MARK_INCHES);
//        }
//    }
//
//    void ejectPurplePixel() {
//        intake.ejectPixel();
//    }
//
//    void driveFromSpikeMark(SpikeMark teamPropMark) {
//        driveForwardInches(-SPIKE_MARK_DECISION_DISTANCE);
//
//        if (teamPropMark == SpikeMark.CENTER) {
//            driveForwardInches(-CENTER_SPIKE_MARK_INCHES);
//        } else if (teamPropMark == SpikeMark.LEFT) {
//            turnDegrees(-OFFCENTER_DECISION_TURN_DEGREES);
//            driveForwardInches(-OFFCENTER_SPIKE_MARK_INCHES);
//        }
//        else if (teamPropMark == SpikeMark.RIGHT) {
//            turnDegrees(OFFCENTER_DECISION_TURN_DEGREES);
//            driveForwardInches(-OFFCENTER_SPIKE_MARK_INCHES);
//        }
//    }
//
//    void driveToBackdrop(boolean isNear) {
//    }
//
//    void depositYellowPixel(SpikeMark teamPropMark) {
//    }
//
//    void parkInBackstage(boolean parkLeft) {
//    }
//
//
//}