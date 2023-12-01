package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ContinousServo {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    CRServo deliveryServo;
    double fullRotationTime = 840; //time it takes for full servo rotation in milliseconds
    double timeToRotateAngle; //amount of time it takes to go to angle

    public ContinousServo (HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        deliveryServo = hardwareMap.crservo.get("servo");
    }

    public void init () {
        deliveryServo.resetDeviceConfigurationForOpMode();
        //deliveryServo.setDirection(FORWARD);
    }

    public void rotateDelivery (double power, double rotationDegrees) {
//        if (power != 0 || rotationDegrees != 0) {
//            timeToRotateAngle = (rotationDegrees/360) * fullRotationTime;
//            Range.clip(power, -1.0, 1.0);
//            timeToRotateAngle = timeToRotateAngle / power;
            deliveryServo.setPower(power);
            telemetry.addData("TARGET SERVO POWER", power);
            telemetry.addData("ACTUAL SERVO POWER", deliveryServo.getPower());
            telemetry.addData("CONTROLLER:", deliveryServo.getController());
//            try {
//                deliveryServo.wait((long)timeToRotateAngle);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
            //deliveryServo.wait(timeToRotateAngle);
            //deliveryServo.setPower(0);
        }
    }


//
//        if(gamepad1.dpad_up)
//        {
//            cntPower = -0.45;
//            telemetry.addData("Keypad" , "dpad_up clicked. power = " + cntPower);
//        }
//        else if(gamepad1.dpad_down)
//        {
//            cntPower = 0.45;
//            telemetry.addData("Keypad" , "dpad_down clicked. power = " + cntPower);
//        }
//        else
//        {
//            cntPower = 0.0;
//            telemetry.addData("Keypad" , "Nothing pressed. power = " + cntPower);
//
//        }
//
//        contServo.setPower(cntPower);
//
//        telemetry.update();
