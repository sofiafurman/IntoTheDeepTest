package org.firstinspires.ftc.teamcode.autonomous;

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

@Autonomous(name="Robot: TEST Auto Encoder Drive", group="Robot")

public class BasicAutonomous extends LinearOpMode {
    // camera and opencv stuff
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera
    OpenCvWebcam webcam;
    ThePipeline pipeline;

    // COUNTS_PER_INCH is the conversion multiplier. Multiply by an inch count,
    // and it will convert to the same encoder count. (~538 counts is one rotation)
    static final double WHEEL_DIAMETER_INCHES = 3.779528;
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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new ThePipeline();
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

        // AUTO COMMANDS:
        // get analysis of object location
        int analysis = pipeline.getAnalysis();
        boolean keepLooping = true;
        double t0 = getRuntime();
        double t1 = getRuntime() - t0;
        while(keepLooping) {
            telemetry.addData("analysis", "start_"+analysis);
            telemetry.update();
            if (analysis == 1) {//left
                telemetry.addData("location", "left");
                telemetry.update();
                leftFrontDrive.setTargetPosition(-3836);
                keepLooping = false;
            }
            if (analysis == 2){//forward
                telemetry.addData("location", "forward");
                telemetry.update();
                rightBackDrive.setTargetPosition(3836);
                keepLooping = false;
            }
            if (analysis == 3){//right
                telemetry.addData("location", "right");
                telemetry.update();
                rightFrontDrive.setTargetPosition(-3836);
                keepLooping = false;
            }else
                analysis = pipeline.getAnalysis();
            t1 = getRuntime() - t0;
            if (t1 > 3){
                telemetry.addData("location", "guessed, ran out of time");
                telemetry.update();
                keepLooping = false;
            }
        }

        // Set target encoder counts. 1 rotation is...
        // 1120 Counts (AndyMark) or 1440 Counts (Tetrix) or ~538 Counts (GoBilda)
        leftFrontDrive.setTargetPosition(-3836*2);
        rightBackDrive.setTargetPosition(3836*2);   // (1 wheel turn)
        rightFrontDrive.setTargetPosition(3836*2);
        leftBackDrive.setTargetPosition(-3836*2);

        // Turn on motors
        leftFrontDrive.setPower(.3);
        rightBackDrive.setPower(.3);
        rightFrontDrive.setPower(.3);
        leftBackDrive.setPower(.3);

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
class ThePipeline extends OpenCvPipeline {
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
    Rect lrect2 = new Rect(0, 620, 500, 460);
    Rect mrect2 = new Rect(550, 600, 500, 360);
    Rect rrect2 = new Rect(1200, 620, 420, 460);
    boolean rightsideofbarrier = true;
    boolean debugview = true;
    boolean doBlue = true;
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
        if(rightsideofbarrier){
            Imgproc.rectangle(HSV, lrect2, col, 5);
            Imgproc.rectangle(HSV, mrect2, col, 5);
            Imgproc.rectangle(HSV, rrect2, col, 5);
            if (avgl > avgm && avgl > avgr) {
                Imgproc.rectangle(HSV, lrect2, col, 40);
            } else if (avgm > avgr) {
                Imgproc.rectangle(HSV, mrect2, col, 40);
            } else {
                Imgproc.rectangle(HSV, rrect2, col, 40);
            }
        }else {
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
    }

    @Override
    public void init(Mat firstFrame) {
        extractColor(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        extractColor(input);
        if (rightsideofbarrier){
            RectL = HSV.submat(lrect2);
            RectM = HSV.submat(mrect2);
            RectR = HSV.submat(rrect2);
        }else {
            RectL = HSV.submat(lrect);
            RectM = HSV.submat(mrect);
            RectR = HSV.submat(rrect);
        }
        System.out.println("processing requested");
        avgl = (int) Core.mean(RectL).val[0];
        avgm = (int) Core.mean(RectM).val[0];
        avgr = (int) Core.mean(RectR).val[0];
        if(debugview) {
            drawRects(HSV);
        }
        if (avgl >= (avgm + 5) && avgl >= (avgr + 5)) {
            returnlocation = 1;
        } else if (avgm >= (avgl + 5) && avgm >= (avgr + 5)) {
            returnlocation = 2;
        } else if (avgr >= (avgl + 5) && avgr >= (avgm + 5)){
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