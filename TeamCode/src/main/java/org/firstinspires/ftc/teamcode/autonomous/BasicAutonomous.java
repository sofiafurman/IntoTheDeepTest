package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: TEST Auto Encoder Drive", group="Robot")

public class BasicAutonomous extends LinearOpMode {
    // COUNTS_PER_INCH is the conversion multiplier. Multiply by an inch count,
    // and it will convert to the same encoder count. (~538 counts is one rotation)
    static final double WHEEL_DIAMETER_INCHES = 3.75;
    static final double COUNTS_PER_INCH = 537.6898395722 / WHEEL_DIAMETER_INCHES / 3.1415;

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
        telemetry.addData("Status", "Going to crap");
        telemetry.update();

        waitForStart(); // Waiting for start button
        runtime.reset();

        // Set target encoder counts. 1 rotation is...
        // 1120 Counts (AndyMark) or 1440 Counts (Tetrix) or ~538 Counts (GoBilda)
        leftFrontDrive.setTargetPosition((int)(24*3*COUNTS_PER_INCH));
        rightBackDrive.setTargetPosition((int)(24*3*COUNTS_PER_INCH));   // (3 wheel turns)
        rightFrontDrive.setTargetPosition((int)(24*3*COUNTS_PER_INCH));
        leftBackDrive.setTargetPosition((int)(24*3*COUNTS_PER_INCH));

        // Turn on motors
        leftFrontDrive.setPower(.4);
        rightBackDrive.setPower(.4);
        rightFrontDrive.setPower(.4);
        leftBackDrive.setPower(.4);

        // Set motor mode to use encoders
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Do nothing while motors are busy.
        // Update status on phone and print out encoder counts
        while (opModeIsActive() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() &&
                leftFrontDrive.isBusy() && rightBackDrive.isBusy()) {
            telemetry.addData("Status", "Waiting for all motors to stop running.");
            telemetry.addData("Positions (lf, lb, rf, rb)", "%7d; %7d; %7d; %7d",
                    leftFrontDrive.getCurrentPosition(),
                    leftBackDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        // Turn off motors
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Turn off the "RUN_TO_POSITION" mode on each motor
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Let drive know when finished
        telemetry.addData("Status", "'Tis Complete, your Majesty");
        telemetry.addData("Duration", runtime.toString());
        telemetry.update();

        sleep(5000); // Give 5 sec to view message before finishing
    }
}
