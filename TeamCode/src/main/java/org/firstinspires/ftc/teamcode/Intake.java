package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    protected DcMotor intakeMotor;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    static final double MAX_SPEED = 0.75;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        intakeMotor = hardwareMap.get(DcMotor.class, "intake"); //Define hardware
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void loop() {
        readGamepad();
        telemetry.addData("Intake Power:", intakeMotor.getPower());
    }

    private void readGamepad() {
        if (gamepad.right_trigger > 0) {
            intakeMotor.setPower(MAX_SPEED);
        } else if (gamepad.left_trigger > 0) {
            intakeMotor.setPower(-MAX_SPEED);
        } else {
            intakeMotor.setPower(0);
        }
    }


}