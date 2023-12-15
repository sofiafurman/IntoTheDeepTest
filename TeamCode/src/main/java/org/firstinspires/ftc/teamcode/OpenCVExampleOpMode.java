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
    SamplePipeline pipeline;
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SamplePipeline();
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
        telemetry.addData("Image Analysis:",pipeline.getAnalysis());
        telemetry.update();
    }


}

class SamplePipeline extends OpenCvPipeline {

    //Mat Binary = new Mat();
    Mat HSV = new Mat();
    Mat hsv1 = new Mat();
    Mat hsv2 = new Mat();
    int avg;
    Scalar lowerBound;
    Scalar upperBound;


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

    @Override
    public void init(Mat firstFrame) {
        inputToRedBinary(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToRedBinary(input);
        System.out.println("processing requested");
        //avg = (int) Core.mean(HSV).val[0];
        //Point pt = new Point(50,100);
        Scalar col = new Scalar(255);
        //Imgproc.putText(HSV, avg + "", pt, 1, 10, col, 5);
        Rect lrect = new Rect(120, 250, 300, 300);
        Imgproc.rectangle(HSV, lrect, col, 5);
        Rect mrect = new Rect(900, 290, 250, 250);
        Imgproc.rectangle(HSV, mrect, col, 5);
        Rect rrect = new Rect(1650, 250, 300, 300);
        Imgproc.rectangle(HSV, rrect, col, 5);

        //HSV.release(); // don't leak memory! // lol nvm we leakin fs
        hsv1.release(); // don't leak memory!
        hsv2.release(); // don't leak memory!
        //Binary.release();
        return HSV;
    }
    public int getAnalysis() {
        return avg;
    }
}