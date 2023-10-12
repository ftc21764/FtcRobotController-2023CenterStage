package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    private static final double MAX_SPEED = 0.5;

    DcMotor intakeMotor;

    void grabPixel(){
        // will grab the pixel
        intakeMotor.setPower(MAX_SPEED);
    }


}
