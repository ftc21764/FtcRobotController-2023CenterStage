package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ServoController;

@TeleOp(name="ServoTest", group="Robot")
public class ServoTest extends OpMode {
    ServoController board = new ServoController();

    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad2.a) {
            board.setServoPosition(1.0);
        } else if (gamepad2.b) {
            board.setServoPosition(0.0);
        } else {
            board.setServoPosition(0.5);
        }
    }
}
