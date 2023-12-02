package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncherServoController {
    private Servo servo;

    public void init(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "launcherServo");
    }
    public void setServoPosition(double position){
        servo.setPosition(position);
    }
}
