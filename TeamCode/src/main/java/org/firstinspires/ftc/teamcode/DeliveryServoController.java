package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DeliveryServoController {
    private Servo servo;

    public void init(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "deliveryServo");
    }
    public void setServoPosition(double position){
        servo.setPosition(position);
    }
}