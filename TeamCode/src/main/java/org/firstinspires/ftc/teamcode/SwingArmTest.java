package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test: SwingArm", group="Robot")
public class SwingArmTest extends LinearOpMode {
    protected SwingArm      swingArm = null;
    protected ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        swingArm = new SwingArm(hardwareMap, telemetry, gamepad2, false);
        swingArm.setBrakeMode(false);
        while (opModeInInit()) {
            swingArm.initLoop();
            telemetry.update();
        }
        swingArm.setBrakeMode(true);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                swingArm.loop();
                telemetry.addData("Loop Time", (int)runtime.milliseconds());
                runtime.reset();
                telemetry.update();
            }
        }
    }
}