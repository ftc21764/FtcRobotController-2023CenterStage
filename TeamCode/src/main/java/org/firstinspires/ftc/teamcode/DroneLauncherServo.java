package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DroneLauncherServo {

    ServoController droneLauncherServo = new ServoController();
    private HardwareMap hardwareMap;
    private Gamepad gamepad;

    public DroneLauncherServo(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        droneLauncherServo.init(hardwareMap, "droneLauncherServo");

        //droneLauncherServo.setServoPosition(0.9);
    }

    public void loop() {
        if (gamepad.dpad_up) {
            droneLauncherServo.setServoPosition(0.50);
        }
    }
}

