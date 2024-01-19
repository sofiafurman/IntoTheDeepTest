/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//This is just a test

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="COMPETITION TELE-OP", group="Linear OpMode")
//@Disabled
public class BasicMecanumTeleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor spinMotor = null;

    private DcMotor extendMotor = null;

    private Servo servoTilt = null;
    private Servo servoRelease = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        spinMotor = hardwareMap.get(DcMotor.class, "spin_motor");
        extendMotor = hardwareMap.get(DcMotor.class, "extend_motor");
        servoTilt = hardwareMap.get(Servo.class, "servo_motor");
        servoRelease = hardwareMap.get(Servo.class, "servo_release");


        // Servo Values

        final double INCREMENT = 0.001;     // amount to slew servo each CYCLE_MS cycle
        final int CYCLE_MS = 50;     // period of each cycle
        final double MAX_POS_TILT = 0.72;     // Maximum rotational position 65
        final double MIN_POS_TILT = 0.43;     // Minimum rotational position og .47
        final double MAX_POS_RELEASE = 0.53;    // Highest position (all pixels released)
        final double MID_POS_RELEASE = 0.35;     // Middle position (1 pixel released)
        final double MIN_POS_RELEASE = 0.31;   // Lowest position (default) OG .429

        double extendMultiplier = 1;

        double tiltServoPos = MIN_POS_TILT;


        double mode = 1;    // For the timing between the servos
        double timeSet = 80;
        double time = timeSet;

        double increment = 0.001;

        // Toggle
        double Toggle = 1;
        boolean changed1 = false;

        // Speed
        double speed = 1;
        boolean changed2 = false;

        //Spin and spin toggle
        double spinFactor = 0;

        // Level
        int levelZero = 0; //originally encoder level set to -5 but was trying to go into itself????
        int levelOne = -1540; //og -1100
        int levelTwo = -1995; //og -1425
        int levelThree = -2555; //og -1825
        int levelFour = -3360; //og -2400
        int currentPos = levelZero;
        boolean changed3 = false;
        boolean changed4 = false;
        boolean extendToggle = false;
        boolean changed5 = false;



        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);
        extendMotor.setDirection(DcMotor.Direction.FORWARD);
        servoTilt.setDirection(Servo.Direction.FORWARD);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);






        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        //telemetry.addData("Starting at",  "%7d", -1 * extendMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Set the starting positions of the servos
        servoTilt.setPosition(tiltServoPos);
        servoRelease.setPosition(MIN_POS_RELEASE);

        // Set starting position of extend motor
        extendMotor.setTargetPosition(levelZero);
        //currentPos is levelZero



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad2.dpad_left){ // && currentPos == levelZero
                extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            /*
            if (gamepad2.dpad_left && !changed5) {

                extendToggle = !extendToggle;
                telemetry.addData("status", "switched!");
                changed5 = true;

               // extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



            } else if (!gamepad2.dpad_left) {
                changed5 = false;

            }
            */


            //float extendPower = gamepad2.left_stick_y;

            if(!extendToggle) {
                telemetry.addData("status", "button not pressed");
                if (gamepad2.right_bumper && !changed3) {
                    if ((currentPos == levelZero )) {
                        currentPos = levelOne;

                    } else if (currentPos == levelOne) {
                        currentPos = levelTwo;

                    } else if (currentPos == levelTwo) {
                        currentPos = levelThree;

                    }
                    else if (currentPos == levelThree) {
                        currentPos = levelFour;
                    }
                    extendMotor.setTargetPosition(currentPos);
                    extendMotor.setPower(0.7);
                    changed3 = true;

                } else if (!gamepad2.right_bumper) {
                    changed3 = false;
                }


                if (gamepad2.left_bumper && !changed4) {
                    if (currentPos == levelFour){
                        currentPos = levelThree;
                    }
                    else if (currentPos == levelThree) {
                        currentPos = levelTwo;

                    } else if (currentPos == levelTwo) {
                        currentPos = levelOne;

                    } else if (currentPos == levelOne) {
                        currentPos = levelZero;
                        extendMotor.setTargetPosition(currentPos);
                        //sleep(1000);
                        //extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        //currentPos = 0;



                    }
                    extendMotor.setTargetPosition(currentPos);


                    extendMotor.setPower(0.7);
                    changed4 = true;

                } else if (!gamepad2.left_bumper) {
                    changed4 = false;
                }
                extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);//og the only place where run to pos is found
            }


            else if(extendToggle) {
                telemetry.addData("status", "joystick");

                //extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                float stickInput = gamepad2.left_stick_y;
                double extendPower = stickInput;

                extendMotor.setPower(extendPower);

                //telemetry.addData("varible", "%7d", extendPower);
                telemetry.addData("stick",  gamepad2.left_stick_y);
                telemetry.addData("power val",  extendMotor.getPower());//"%4.2f",
                telemetry.addData("encoder/not", extendMotor.getMode());

                //telemetry.addData("AAAAAAAA", "%7d", extendMotor.getCurrentPosition() * -1);


            }




            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y * Toggle;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x * Toggle;
            double yaw = gamepad1.right_stick_x;



            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Toggle code (flip driver controls)
            if (gamepad1.dpad_left && !changed1) {
                Toggle = Toggle * -1;
                changed1 = true;

            } else if (!gamepad1.dpad_left) {
                changed1 = false;
            }

            //speed code (slow down the robot controls)
            if (gamepad1.dpad_right && !changed2) {
                if (speed == 0.7) {
                    speed = 1;
                } else if (speed == 1) {
                    speed = 0.7;
                }
                changed2 = true;

            } else if (!gamepad1.dpad_right) {
                changed2 = false;
            }

            //spinny code (intake motor)
            if (gamepad2.a || gamepad1.right_bumper) {
                spinFactor = 0.87;
                servoRelease.setPosition(0.62);
            } else if (gamepad2.b ||  gamepad1.left_bumper) {
                spinFactor = -0.87;
            } else {
                spinFactor = 0;
            }

            /*
            if (gamepad1.right_bumper) {
                spinFactor = 0.87;
                servoRelease.setPosition(MAX_POS_RELEASE);
            } else if (gamepad1.left_bumper) {
                spinFactor = -0.87;
            } else {
                spinFactor = 0;
            }
            */

            //reset servo pos to zero when no intake
            if (!gamepad2.a && ((currentPos == levelZero))){
                servoRelease.setPosition(MIN_POS_RELEASE);
            }

            // Arm servo code (tilt mechanism)

            if (gamepad2.y && !gamepad2.dpad_down && (currentPos != levelZero) ) {

                servoTilt.setPosition(MAX_POS_TILT);
            } else if (gamepad2.x && (currentPos != levelZero)  ) {
                servoTilt.setPosition(MIN_POS_TILT);
            }


            // Release servo code (dpad controls)
            if (gamepad2.dpad_down) {
                servoTilt.setPosition(MIN_POS_TILT);
                servoRelease.setPosition(MIN_POS_RELEASE);
                //extendMotor.setTargetPosition(-5);
                currentPos = levelZero;
                extendMotor.setTargetPosition(currentPos);
                sleep(1000);
                //extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //currentPos = 0;
                //extendMotor.setTargetPosition(currentPos);

                extendMotor.setPower(0.6); //og 0.7 but lower for voltage
            } else if (gamepad2.dpad_right  && ((currentPos != levelZero)))  { //&& ((currentPos != 0)  ) og
                servoRelease.setPosition(MID_POS_RELEASE);
            }




            else if (gamepad2.dpad_up) {
                servoRelease.setPosition(MAX_POS_RELEASE);
            }



            /*
            if (extendMotor.getCurrentPosition() == 0) {
                extendMultiplier = 1;
            } else if (extendMotor.getCurrentPosition() == 2914) {
                extendMultiplier = -1;
            }
            */






            // This servo code times the movement of the servos such
            // that one will finish before the other one (tilt then release)
            /*if (!gamepad2.right_bumper) {
                mode = 1;
                time = timeSet;
                servoTilt.setPosition(MIN_POS_TILT);
                if (!gamepad2.a) {
                    servoRelease.setPosition(MIN_POS_RELEASE);
                }
            } else {
                if (mode == 1) {
                    servoTilt.setPosition(MAX_POS_TILT);
                } else if (mode == 2) {
                    servoRelease.setPosition(MAX_POS_RELEASE);
                }
                time -= 1;
                if (time <= 0) {
                    time = timeSet;
                    mode = 2;
                }
            }*/

            //idle();

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * speed);
            rightFrontDrive.setPower(rightFrontPower * speed);
            leftBackDrive.setPower(leftBackPower * speed);
            rightBackDrive.setPower(rightBackPower * speed);
            spinMotor.setPower(spinFactor);
            //extendMotor.setPower(0.7);

            // Show the elapsed game time and wheel power.
            telemetry.addData("This is the toggle value", Toggle);
            telemetry.addData("This is the speed multiplier", speed);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Tilt Servo Position", "%5.2f", tiltServoPos);
            telemetry.addData("Currently at", "%7d", extendMotor.getCurrentPosition() * -1);
            telemetry.addData("Target pos", "%7d", currentPos);
            telemetry.update();
        }
    }
}