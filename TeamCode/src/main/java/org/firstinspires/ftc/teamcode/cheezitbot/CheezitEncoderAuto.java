package org.firstinspires.ftc.teamcode.cheezitbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Robot: Auto Drive By Encoder (Cheezitbot)", group="Robot")
//@Disabled
public class CheezitEncoderAuto extends LinearOpMode {
    // camera and opencv stuff
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera
    OpenCvWebcam webcam;
    APipeline pipeline;

    // COUNTS_PER_INCH is the conversion multiplier. Multiply by an inch count,
    // and it will convert to the same encoder count. (1120 counts is one rotation for AndyMark motors)
    static final double WHEEL_DIAMETER_INCHES = 4;
    static final double COUNTS_PER_INCH = 1120 / WHEEL_DIAMETER_INCHES / 3.1415;
    static final double STRAFE_COUNTS_PER_INCH = 1120 / 11.981875;

    // Define the motors
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Telemetry and timer
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new APipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        });

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

        resetEncoders();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Give phone an update
        telemetry.addData("Status", "Going to crab");
        telemetry.update();

        waitForStart(); // Waiting for start button
        runtime.reset();

        // AUTO COMMANDS:
        // get analysis of object location
        int analysis = pipeline.getAnalysis();
        boolean keepLooping = true;
        double t0 = getRuntime();
        double t1 = getRuntime() - t0;
        while(keepLooping) {
            if (analysis == 1) {
                encoderStrafe(-24, .4);//left
                keepLooping = false;
            }
            else if (analysis == 2){
                encoderDrive(24, 24, .4);//forward
                keepLooping = false;
            }
            else if (analysis == 3){
                encoderStrafe(24, .4);//right
                keepLooping = false;
            }else
                analysis = pipeline.getAnalysis();
            t1 = getRuntime() - t0;
            if (t1 > 3){
                keepLooping = false;
                encoderDrive(3, 3, .4);//forward
            }
        }


        // Let drive know when finished
        telemetry.addData("Status", "'Tis Complete, your Majesty");
        telemetry.addData("Duration", runtime.toString());
        telemetry.addData("ELAPSED TIME FOR VISION", t1 + "seconds");
        telemetry.update();

        sleep(5000); // Give 5 sec to view message before finishing
    }

    private void resetEncoders() {
        // Reset encoders and set their encoder mode
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            resetEncoders();
            leftBackTarget = (int)(leftIn * COUNTS_PER_INCH);
            leftBackDrive.setTargetPosition(leftBackTarget);
            leftFrontTarget = (int)(leftIn * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            rightBackTarget = (int)(rightIn * COUNTS_PER_INCH);
            rightBackDrive.setTargetPosition(rightBackTarget);
            rightFrontTarget = (int)(rightIn * COUNTS_PER_INCH);
            rightFrontDrive.setTargetPosition(rightFrontTarget);

            // Run the robot auto with the new encoder targets
            startWaitFinish(speed, 1000);
        }
    }

    public void encoderStrafe(double targetIn, double speed) {
        // This func will move the robot to a desired inches target
        // Right is positive inches, left is negative
        int leftBackTarget;
        int leftFrontTarget;
        int rightBackTarget;
        int rightFrontTarget;

        if (opModeIsActive()) {
            // Set targets using conversion factor to turn inches into encoder counts
            resetEncoders();
            double newTarget = (targetIn * STRAFE_COUNTS_PER_INCH);

            rightFrontTarget = -(int)newTarget;
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            leftBackTarget = -(int)newTarget;
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightBackTarget = (int)newTarget;
            rightBackDrive.setTargetPosition(rightBackTarget);
            leftFrontTarget = (int)newTarget;
            leftFrontDrive.setTargetPosition(leftFrontTarget);

            // Run
            startWaitFinish(speed, 1000);
        }
    }

    public void startWaitFinish(double speed, long waitMillis) {
        // This code is reused a couple times, so all it does is turn on
        // the motors (after their encoder targets are set) and wait until
        // they are done running to their position.
        // Turn on the motors
        leftBackDrive.setPower(Math.abs(speed));
        leftFrontDrive.setPower(Math.abs(speed));
        rightBackDrive.setPower(Math.abs(speed));
        rightFrontDrive.setPower(Math.abs(speed));

        // Set encoder mode to run
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // The movement will automatically stop if the runtime exceeds
        // 120 seconds (length of an whole ftc match).
        // Don't want the robot to run forever by accident
        while (opModeIsActive() &&
                leftFrontDrive.isBusy() || leftBackDrive.isBusy() &&
                rightFrontDrive.isBusy() && rightBackDrive.isBusy() &&
                runtime.seconds() < 120) {
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
        sleep(waitMillis); // Wait a second before next move
    }
}
class APipeline extends OpenCvPipeline {
    Mat HSV = new Mat();
    Mat hsv1 = new Mat();
    Mat hsv2 = new Mat();
    int avgr;
    int avgm;
    int avgl;
    Scalar lowerBound;
    Scalar upperBound;
    Mat RectR = new Mat();
    Mat RectM = new Mat();
    Mat RectL = new Mat();
    Rect lrect = new Rect(220, 620, 500, 460);
    Rect mrect = new Rect(800, 600, 500, 360);
    Rect rrect = new Rect(1500, 620, 420, 460);
    boolean debugview = true;
    boolean doBlue = false;
    int returnlocation;
    /*
     * This function takes the RGB frame, converts to HSV,
     * and extracts the red color to the '' variable
     */
    void extractColor(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);  // convert the RGB to HSV
        if (!doBlue){
            // get red pixels (hue > 0)
            lowerBound = new Scalar(0, 200, 30);
            upperBound = new Scalar(5, 255, 255);
            Core.inRange(HSV, lowerBound, upperBound, hsv1);
            // get red pixels (hue < 180)
            lowerBound = new Scalar(170, 200, 30);
            upperBound = new Scalar(180, 255, 255);
            Core.inRange(HSV, lowerBound, upperBound, hsv2);

            Core.add(hsv1, hsv2, HSV);
        }                                     // get red pixels
        else {
            // get blue pixels
            lowerBound = new Scalar(100, 150, 0);
            upperBound = new Scalar(140, 255, 255);
            Core.inRange(HSV, lowerBound, upperBound, hsv1);
            Core.add(hsv1, hsv1, HSV);
        }                                            // get blue pixels
    }
    void drawRects(Mat input) {
        Point ptl = new Point(290, 300);
        Point ptm = new Point(970, 300);
        Point ptr = new Point(1580, 300);
        Scalar col = new Scalar(255);
        Imgproc.putText(HSV, avgl + "", ptl, 1, 10, col, 5);
        Imgproc.putText(HSV, avgm + "", ptm, 1, 10, col, 5);
        Imgproc.putText(HSV, avgr + "", ptr, 1, 10, col, 5);
        Imgproc.rectangle(HSV, lrect, col, 5);
        Imgproc.rectangle(HSV, mrect, col, 5);
        Imgproc.rectangle(HSV, rrect, col, 5);
        if (avgl > avgm && avgl > avgr) {
            Imgproc.rectangle(HSV, lrect, col, 40);
        } else if (avgm > avgr) {
            Imgproc.rectangle(HSV, mrect, col, 40);
        } else {
            Imgproc.rectangle(HSV, rrect, col, 40);
        }
    }

    @Override
    public void init(Mat firstFrame) {
        extractColor(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        extractColor(input);
        RectL = HSV.submat(lrect);
        RectM = HSV.submat(mrect);
        RectR = HSV.submat(rrect);
        System.out.println("processing requested");
        avgl = (int) Core.mean(RectL).val[0];
        avgm = (int) Core.mean(RectM).val[0];
        avgr = (int) Core.mean(RectR).val[0];
        if(debugview) {
            drawRects(HSV);
        }
        if (avgl >= (avgm + 3) && avgl >= (avgr + 3)) {
            returnlocation = 1;
        } else if (avgm >= (avgl + 3) && avgm >= (avgr + 3)) {
            returnlocation = 2;
        } else if (avgr >= (avgl + 3) && avgr >= (avgm + 3)){
            returnlocation = 3;
        }else{
            returnlocation = 4;
        }

        hsv1.release();   // don't leak memory
        hsv2.release();   //        |
        RectL.release();  //        |
        RectM.release();  //        |
        RectR.release();  //________V__________
        if (debugview)
            return HSV;
        else {
            HSV.release(); // don't leak memory
            return input;
        }
    }
    public int getAnalysis() {
        return returnlocation;
    }
}