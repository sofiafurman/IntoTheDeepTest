package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

@Autonomous(name="Robot: Auto Blue Left", group="Robot")
//@Disabled
public class AutoBlueLeft extends LinearOpMode {
    // camera and opencv stuff
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera
    OpenCvWebcam webcam;
    ThePipeline4 pipeline;

    // COUNTS_PER_INCH is the conversion multiplier. Multiply by an inch count,
    // and it will convert to the same encoder count. (~538 counts is one rotation)
    static final double WHEEL_DIAMETER_INCHES = 3.779528;
    static final double COUNTS_PER_INCH = 537.6898395722 / WHEEL_DIAMETER_INCHES / 3.1415;
    static final double STRAFE_COUNTS_PER_INCH = 537 / 10.3775;
    static final double TURN_COUNTS_PER_DEGREE = 3836.0 / 360;

    // Define the motors
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor spinMotor = null;
    private DcMotor extendMotor = null;
    private Servo servoTilt = null;
    private Servo servoRelease = null;

    final double MAX_POS_TILT     =  0.73;     // Maximum rotational position
    final double MIN_POS_TILT     =  0.413;     // Minimum rotational position
    final double MAX_POS_RELEASE = 0.4;    // Highest position (all pixels released)
    final double MID_POS_RELEASE = 0.36;     // Middle position (1 pixel released)
    final double MIN_POS_RELEASE = 0.31;   // Lowest position (default)

    private int camera_val = 1;/////////////////////////////////////////////////////////////


    // Telemetry and timer
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new ThePipeline4();
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

        // Servos, intake, and slide motors
        spinMotor = hardwareMap.get(DcMotor.class, "spin_motor");
        extendMotor = hardwareMap.get(DcMotor.class, "extend_motor");
        servoTilt = hardwareMap.get(Servo.class, "servo_motor");
        servoRelease = hardwareMap.get(Servo.class, "servo_release");

        // Set directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Give phone an update
        telemetry.addData("Status", "'Dis piece o' s*** is READY baby.");
        telemetry.update();

        waitForStart(); // Waiting for start button
        runtime.reset();

        // VISION CODE START
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
                camera_val = 1;
                keepLooping = false;
            }
            if (analysis == 2){//forward
                telemetry.addData("location", "forward");
                telemetry.update();
                camera_val = 2;
                keepLooping = false;
            }
            if (analysis == 3){//right
                telemetry.addData("location", "right");
                telemetry.update();
                camera_val = 3;
                keepLooping = false;
            }else
                analysis = pipeline.getAnalysis();
            t1 = getRuntime() - t0;
            if (t1 > 3){
                telemetry.addData("location", "guessed, ran out of time");
                telemetry.update();
                camera_val = 2;
                keepLooping = false;
            }
        }
        // VISION CODE END

        // Reset servos
        servoTilt.setPosition(MIN_POS_TILT);
        servoRelease.setPosition(MIN_POS_RELEASE);


        ////////// AUTO COMMANDS:
        if (camera_val == 1) {
            // Drop pixel on left line
            encoderDrive(8, 8, .3);
            encoderStrafe(-5, .2);
            encoderDrive(12.5, 12.5, .3);
            encoderDrive(-1, -1, .2);
            dropPixel(3);
            encoderStrafe(-20, .2);
            encoderTurn(90, .2);
        } else if (camera_val == 2) {
            // Middle line auto
            encoderDrive(8, 8, .3);
            encoderStrafe(3, .3);
            encoderDrive(20, 20, .3);
            spinMotor.setPower(-.3);
            encoderDrive(-10, -10, .3);
            spinMotor.setPower(0);
            // Get in front of board
            encoderStrafe(-30, .3);
            encoderDrive(8, 8, .3);
            encoderTurn(90, .2);
        } else {
            // Drop pixel on right line
            encoderDrive(18, 18, .3);
            encoderTurn(60, .2);
            encoderDrive(9, 9, .3);
            dropPixel(29);
            encoderStrafe(-20, .2);
            encoderTurn(30, .2);
        }
        // Score on board
        encoderDrive(-12, -12, .2);
        placePixel((int)(900*1.4), 1);
        encoderStrafe(20, .31);
        /////////// END OF AUTO


        // Let drive know when finished
        telemetry.addData("Status", "'Tis Complete, your Majesty");
        telemetry.addData("Duration", runtime.toString());
        telemetry.update();

        sleep(5000); // Give 5 sec to view message before finishing
    }

    private void dropPixel(double backUp) {
        spinMotor.setPower(-.3);                        // Spit out pixel
        encoderDrive(-backUp, -backUp, .3);   // Backup to make sure it's out
        spinMotor.setPower(0);
    }

    private void placePixel(int height, double slideSpeed) {
        // This function extends the slides and drops a pixel onto the board
        // First, move slide to a desired height
        extendMotor.setTargetPosition(-height); // slide moves in negative direction
        extendMotor.setPower(slideSpeed);
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        // Release pixel
        servoTilt.setPosition(MAX_POS_TILT);
        sleep(400);
        servoRelease.setPosition(MAX_POS_RELEASE);
        sleep(400);
        encoderDrive(2, 2, .2);     // Back away from the board
        // Reset
        servoTilt.setPosition(MIN_POS_TILT);
        servoRelease.setPosition(MIN_POS_RELEASE);
        sleep(200);
        extendMotor.setTargetPosition(0);
        sleep(1000);
        extendMotor.setPower(0);        // (Turn off slide)
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            startWaitFinish(speed, 250);
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
            startWaitFinish(speed, 250);
        }
    }

    public void encoderTurn(double angle, double speed) {
        // This function will turn the robot to an exact angle
        // to the best ability of the GoBilda encoders.
        int leftBackTarget;
        int leftFrontTarget;
        int rightBackTarget;
        int rightFrontTarget;

        if (opModeIsActive()) {
            resetEncoders();
            int newTarget = (int)(angle * TURN_COUNTS_PER_DEGREE);

            rightFrontTarget = -newTarget;
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            leftBackTarget = newTarget;
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightBackTarget = -newTarget;
            rightBackDrive.setTargetPosition(rightBackTarget);
            leftFrontTarget = newTarget;
            leftFrontDrive.setTargetPosition(leftFrontTarget);

            // Run
            startWaitFinish(speed, 250);
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

class ThePipeline4 extends OpenCvPipeline {
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
    boolean rightsideofbarrier = false;
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