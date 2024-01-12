package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncherServoController {
    public Servo servo;

    public void init(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "droneLauncherServo");
    }

    public void setServoPosition(double position) {
        servo.setPosition(position);
    }
}
