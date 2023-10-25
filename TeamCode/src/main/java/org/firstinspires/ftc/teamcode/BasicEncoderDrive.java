package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
//@Disabled
public class BasicEncoderDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Encoder code will run here (this is the auto base)
        // It will also call encoderDrive() to simplify the code
    }

    public void encoderDrive() {
        // The juicy encoder details will be in here
    }
}
