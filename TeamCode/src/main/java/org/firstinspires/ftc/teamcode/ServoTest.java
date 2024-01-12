package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ServoTest", group="Robot")
public class ServoTest extends LinearOpMode{

    public void runOpMode () {

        DroneLauncherServo droneLauncherServo = new DroneLauncherServo(hardwareMap, gamepad2);

        waitForStart();

        while (opModeIsActive()) {

            droneLauncherServo.loop();
        }
    }

//
//    @Override
//    public void init() {
//        board.init(hardwareMap);
//    }
//
//    @Override
//    public void loop() {
////        DRONE LAUNCHER SERVO
//        if (gamepad2.dpad_up) {
//            if (!launched) {
//                telemetry.addData("INPUTTED", gamepad2.dpad_up);
//                telemetry.update();
//                rotationTimer.reset();
//                board.setServoPosition(0.50);
//                launched = true;
//                telemetry.addData("Launched?", launched);
//                telemetry.update();
//            }
//            //telemetry.addData("SERVO POS", board.servo.getPosition());
//        }


//        DELIVERY SERVO
//        rotationTimer.reset();
//        if (gamepad2.a) {
//            while (rotationTimer.time() < rotationTime) {
//                board.setServoPosition(0);
//            }
//        } else if (gamepad2.b) {
//            while (rotationTimer.time() < rotationTime) {
//                board.setServoPosition(1);
//            }
//        } else {
//            board.setServoPosition(0.5);
//        }
    }

















//    @Override
//    public void loop() {
//        if (gamepad2.a) {
//            board.setServoPosition(0.45);
//            try {
//                sleep(666);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        } else if (gamepad2.b) {
//            board.setServoPosition(0.55);
//            try {
//                sleep(666);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        } else {
//            board.setServoPosition(0.5);
//        }
//    }
