package org.firstinspires.ftc.teamcode.cheezitbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Encoder (Cheezitbot)", group="Robot")
//@Disabled
public class CheezitEncoderAuto extends LinearOpMode {
    // COUNTS_PER_INCH is the conversion multiplier. Multiply by an inch count,
    // and it will convert to the same encoder count. (1120 counts is one rotation for AndyMark motors)
    static final double WHEEL_DIAMETER_INCHES = 4;
    static final double COUNTS_PER_INCH = 1120 / WHEEL_DIAMETER_INCHES / 3.1415;

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

        // Drive the robot with the following commands
        // (Inch targets for each wheel and speed to move)
        /*encoderDrive(48, 48, .5);
        encoderDrive(20, -20, .3);
        encoderDrive(-24, -24, .5);
        encoderDrive(-20, 20, .3);
        encoderDrive(24, 24, .5);*/
        encoderDrive(15, -15, .3);
        encoderDrive(20, -20, .3);
        encoderDrive(25, -25, .3);
        encoderDrive(30, -30, .3);

        // Let drive know when finished
        telemetry.addData("Status", "'Tis Complete, your Majesty");
        telemetry.addData("Duration", runtime.toString());
        telemetry.update();

        sleep(5000); // Give 5 sec to view message before finishing
    }

    public void encoderDrive(double leftIn, double rightIn, double speed) {
        // The juicy encoder details will be in here
        int leftBackTarget;
        int leftFrontTarget;
        int rightBackTarget;
        int rightFrontTarget;

        if (opModeIsActive()) {
            // Convert inputted inches into encoder counts
            // Also, set the target encoder counts per motor
            leftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftIn * COUNTS_PER_INCH);
            leftBackDrive.setTargetPosition(leftBackTarget);
            leftFrontTarget = leftBackDrive.getCurrentPosition() + (int)(leftIn * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            rightBackTarget = leftBackDrive.getCurrentPosition() + (int)(rightIn * COUNTS_PER_INCH);
            rightBackDrive.setTargetPosition(rightBackTarget);
            rightFrontTarget = leftBackDrive.getCurrentPosition() + (int)(rightIn * COUNTS_PER_INCH);
            rightFrontDrive.setTargetPosition(rightFrontTarget);

            // Turn on the motors
            leftBackDrive.setPower(Math.abs(speed));
            leftFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));

            // This will turn off the "RUN_TO_POSITION" mode on each motor
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // The movement will automatically stop if the runtime exceeds
            // 200 seconds (190 seconds is the length of an whole ftc match)
            // Don't want the robot to run forever by accident
            while (opModeIsActive() &&
                    leftFrontDrive.isBusy() || leftBackDrive.isBusy() &&
                    rightFrontDrive.isBusy() && rightBackDrive.isBusy() &&
                    runtime.seconds() < 200) {
                telemetry.addData("Status", "Busy af rn leave me alone");
                telemetry.addData("Positions (lf, lb, rf, rb)", "%7d; %7d; %7d; %7d",
                        leftFrontDrive.getCurrentPosition(),
                        leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            // Turn off motors
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            // This will turn off the "RUN_TO_POSITION" mode on each motor
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Finished move
            telemetry.addData("Status", "Move finished");
            telemetry.update();
            sleep(10000); // Wait a second
        }
    }
}
