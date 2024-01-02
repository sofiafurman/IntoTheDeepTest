package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import java.util.ArrayList;

@TeleOp(name = "Concept: OpenCV", group = "Concept")
public class OpenCVExampleOpMode extends OpMode {
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera
    OpenCvWebcam webcam;
    Pipeline pipeline;
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new Pipeline();
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

    }

    @Override
    public void loop() {
        telemetry.addData("Game Piece Location:",pipeline.getAnalysis());
        telemetry.update();
    }


}

class Pipeline extends OpenCvPipeline {
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
    int returnlocation;
    /*
     * This function takes the RGB frame, converts to HSV,
     * and extracts the red color to the '' variable
     */
    void inputToRedBinary(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        // get red pixels (hue > 0)
        lowerBound = new Scalar(0, 200, 30);
        upperBound = new Scalar(5, 255, 255);
        Core.inRange(HSV, lowerBound, upperBound, hsv1);
        // get red pixels (hue < 180)
        lowerBound = new Scalar(170, 200, 30);
        upperBound = new Scalar(180, 255, 255);
        Core.inRange(HSV, lowerBound, upperBound, hsv2);

        Core.add(hsv1, hsv2, HSV);
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
            Imgproc.rectangle(HSV, lrect, col, 10);
        } else if (avgm > avgr) {
            Imgproc.rectangle(HSV, mrect, col, 10);
        } else {
            Imgproc.rectangle(HSV, rrect, col, 10);
        }
    }

    @Override
    public void init(Mat firstFrame) {
        inputToRedBinary(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToRedBinary(input);
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
        if (avgl > avgm && avgl > avgr) {
            returnlocation = 1;
        } else if (avgm > avgr) {
            returnlocation = 2;
        } else {
            returnlocation = 3;
        }

        hsv1.release();   // don't leak memory
        hsv2.release();   //  do not leak memory
        RectL.release();  //  |
        RectM.release();  //  |
        RectR.release();  //  V
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