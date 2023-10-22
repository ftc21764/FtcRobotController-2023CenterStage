/* -Pseudocode Notes-

Encoders:
UltraPlanetary motor: 28 cpr (counts per rotation)
Core Hex motor encoder: 4 cpr @ motor, 288 cpr @ output

Ratios:
3:1 real - 84:29
4:1 real - 76:21
5:1 real - 68:13

Math for distance function:
Wheel diameter (mm) * PI / Gear ratio / Encoder ticks * mm. to in.

Functions:
turn(int deg) must turn the robot deg degrees. It should have a variability value of about 1 degree.
move(String dir, double dist) moves the robot dist inches. It should account for speed-up and ramp-down of the motors.

constants:

MMperIN = 25.4
wheelDiaMM = 75
wheelDiaIN = wheelDiaMM / MMperIN //or input just inches as constant
wheelCircum = wheelDiaIN * Pi() //import math lib later
ultPlanHexEncoderTicks = 28
3to1 = 84/29 // real 3:1
4to1 = 76/21 // real 4:1
drivetrainMotorGearRatio = 3to1 * 4to1

public double inchesToTicks(inches) {
    (wheelCircum/(drivetrainMotorGearRatio * ultPlanHexEncoderTicks)) * inches
}



 */
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Autonomous {

    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    // These values are from the sample code, we can change them later

//    void turn(String dir, int time) {
//    //Make turning rely on absolute cardinal direction instead of turn time (later)
//        if (dir == "l") {
//            for (int i = 0; i < time; i++) {
//                leftDriveFront.setPower(-TURN_SPEED);
//                leftDriveBack.setPower(-TURN_SPEED);
//                rightDriveFront.setPower(TURN_SPEED);
//                rightDriveBack.setPower(TURN_SPEED);
//            }
//            leftDriveFront.setPower(0);
//            leftDriveBack.setPower(0);
//            rightDriveFront.setPower(0);
//            rightDriveBack.setPower(0);
//        }
//        else {
//            for (int i = 0; i < time; i++) {
//                leftDrive.setPower(TURN_SPEED);
//                rightDrive.setPower(-TURN_SPEED);
//            }
//            leftDrive.setPower(0);
//            rightDrive.setPower(0);
//        }
//    }
    double MMperIN = 25.4;
    int wheelDiaMM = 75;
    double wheelDiaIN = wheelDiaMM / MMperIN; //or input just inches as constant
    double wheelCircum = wheelDiaIN * 3.14; //Pi(); import math lib later
    int ultPlanHexEncoderTicks = 28;
    double threeToOne = 84/29; // real 3:1
    double fourToOne = 76/21; // real 4:1
    double drivetrainMotorGearRatio = threeToOne * fourToOne;

    public double inchesToTicks(double inches) {
        return (wheelCircum/(drivetrainMotorGearRatio * ultPlanHexEncoderTicks)) * inches;
    }

    void move(int xDirection, double inchesToMove) {
        //
        double ticks = inchesToTicks(inchesToMove);
    }

    public void runOpMode() {
        leftDriveFront = hardwareMap.get(DcMotor.class, "left_driveF");
        rightDriveFront = hardwareMap.get(DcMotor.class, "right_driveF");
        leftDriveBack = hardwareMap.get(DcMotor.class, "left_driveB");
        rightDriveBack = hardwareMap.get(DcMotor.class, "right_driveB");


        //Step 1: detect the team prop

        //Step 2: drive to the team prop

        //Step 3: place purple pixel on same line as team prop

        //Step 4: return to original position

        //Step 5: drive under truss closest to wall to get to backdrop

        //Step 6: place yellow pixel on correct Apriltag

        //Step 7: drive to white pixel stacks going under truss

        //Step 8: pick up two pixels

        //Step 9: return to backdrop and place white pixels

        //Step 10: drive off to side
    }
}