package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
//@Disabled
public class BasicCheezitEncoderAuto extends LinearOpMode {

    // Define the motors
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Telemetry and timer
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // As of 10.31.23 the following order matches the order
        // the motors are plugged into the ports (0-3)
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders and set their encoder mode
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Give phone an update
        telemetry.addData("Status", "Going to crap, but ready");
        telemetry.update();

        waitForStart(); // Waiting for start button
        runtime.reset();

        // Drive the robot with the following commands
        encoderDrive(1, 5, 5, 1000); // Forward
        encoderDrive(2, 5, 5, 1000); // Turn left
        encoderDrive(3, 5, 5, 1000); // Forward
        encoderDrive(4, 5, 5, 1000); // Turn right
        encoderDrive(5, 5, 5, 1000); // Backward

        // Let drive know when finished
        telemetry.addData("Status", "'Tis Complete, your Majesty");
        telemetry.addData("Duration", runtime.toString());
        telemetry.update();

        sleep(5000); // Give 5 sec to view message before finishing
    }

    public void encoderDrive(double leftCount, double rightCount, double speed, long timeOut) {
        // The juicy encoder details will be in here
        while (!opModeIsActive()) {
            // turn on motors
            // give speed
            // wait while they're moving
            // finishing and reset speed
            // print to screen that move finished
        }
        telemetry.addData("test", leftCount);
        telemetry.update();
        sleep(timeOut);
    }
}
