package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ServoTester", group = "TeleOp")

public class ServoTester extends LinearOpMode {
    //All hardware
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor spinMotor = null;

    private DcMotor extendMotor = null;

    private Servo servoMotor = null;

    double servoVal = 0.461;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        spinMotor = hardwareMap.get(DcMotor.class, "spin_motor");
        extendMotor = hardwareMap.get(DcMotor.class, "extend_motor");
        servoMotor = hardwareMap.get(Servo.class, "servo_plane");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                servoVal += 0.001;
            }
            else if(gamepad1.dpad_down){
                servoVal -= 0.001;
            }
            /*if(gamepad1.dpad_left){
                servo2 += 0.001;
            }
            else if(gamepad1.dpad_right){
                servo2 -= 0.001;
            }
             */
            servoMotor.setPosition(servoVal);
            //ringPullPivotServo.setPosition(servo2);

            //telemetry.addData("unloadServo",wobbleArmServo.getPosition());
            telemetry.addData("servo", servoVal);
            telemetry.addData("servoMotor Loc ", servoMotor);
            /*
            telemetry.addData("Distance Left Top", distanceLeftTop.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Left Bottom", distanceLeftBottom.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Forward Right", distanceFwdRight.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Forward Left", distanceFwdLeft.getDistance(DistanceUnit.MM));
*/
            telemetry.update();
        }
    }
}

