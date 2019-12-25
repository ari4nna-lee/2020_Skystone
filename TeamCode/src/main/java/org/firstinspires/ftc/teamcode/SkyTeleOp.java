package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "SkyTeleOp (Studio)", group = "")
public class SkyTeleOp extends LinearOpMode {

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor leftInduction;
    private DcMotor rightInduction;
    private Servo hookLeft;
    private Servo hookRight;
    private DcMotor swing;
    private DcMotor extend;
    private Servo yaw;
    private Servo pitch;
    private Servo armGrabber;
    private Servo push;
    private TouchSensor switch_TouchSensor;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftInduction = hardwareMap.dcMotor.get("leftInduction");
        rightInduction = hardwareMap.dcMotor.get("rightInduction");
        hookLeft = hardwareMap.servo.get("hookLeft");
        hookRight = hardwareMap.servo.get("hookRight");
        swing = hardwareMap.dcMotor.get("swing");
        extend = hardwareMap.dcMotor.get("extend");
        yaw = hardwareMap.servo.get("yaw");
        pitch = hardwareMap.servo.get("pitch");
        push = hardwareMap.servo.get("push");
        armGrabber = hardwareMap.servo.get("armGrabber");
        switch_TouchSensor = hardwareMap.touchSensor.get("switch");
        // Put initialization blocks here.

        waitForStart();
        if (opModeIsActive()) {
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightInduction.setDirection(DcMotorSimple.Direction.REVERSE);

            swing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            swing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            swing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            swing.setDirection(DcMotorSimple.Direction.FORWARD);

            extend.setDirection(DcMotorSimple.Direction.FORWARD);
            hookLeft.setDirection(Servo.Direction.REVERSE);
            extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int initialPosition = swing.getCurrentPosition();

            // Put run blocks here.
            yaw.setPosition(0);
            armGrabber.setPosition(1);
            pitch.setPosition(0.75);
            push.setPosition(0.5);


            while (opModeIsActive() || !isStopRequested()) {
                float swingSpeed = gamepad2.left_stick_y;
                float extendSpeed = gamepad2.right_stick_y;
                telemetry.addData("YAW", yaw.getPosition());
                // Mecanum Drive
                double r = Math.hypot(gamepad1.left_stick_x * -1, gamepad1.left_stick_y);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x * -1) - Math.PI / 4;
                double rightX = gamepad1.right_stick_x * -1;

                double lFront = r * Math.cos(robotAngle) + rightX;
                double rFront = r * Math.sin(robotAngle) - rightX;
                double lRear = r * Math.sin(robotAngle) + rightX;
                double rRear = r * Math.cos(robotAngle) - rightX;
                leftFront.setPower(lFront);
                rightFront.setPower(rFront);
                leftBack.setPower(lRear);
                rightBack.setPower(rRear);
                telemetry.addData("leftFront", lFront);
                telemetry.addData("rightFront", rFront);
                telemetry.addData("leftRear", lRear);
                telemetry.addData("rightRear", rRear);

                //Intake System
                if (gamepad1.dpad_down) {
                    leftInduction.setPower(0.8);
                    rightInduction.setPower(0.8);
                } else if (gamepad1.dpad_up) {
                    leftInduction.setPower(-0.8);
                    rightInduction.setPower(-0.8);
                    if (push.getPosition() != 1.0) {
                        push.setPosition(1.0);
                    }
                } else {
                    leftInduction.setPower(0);
                    rightInduction.setPower(0);
                }
                //Manual Push Control
                if (gamepad1.x){
                    push.setPosition(0);
                }
                if (gamepad1.y){
                    push.setPosition(1);
                }


                //Hooks
                if (gamepad1.a) {
                    hookLeft.setPosition(1);
                    hookRight.setPosition(1);
                } else if (gamepad1.b) {
                    hookLeft.setPosition(0);
                    hookRight.setPosition(0);
                } else {
                    // do nothing
                }

                //Arm Movement
                if (swingSpeed > 0) {
                    if (switch_TouchSensor.isPressed()) {
                        swing.setPower(0);
                    } else {
                        pitch.setPosition(0.8 - (swing.getCurrentPosition() - initialPosition) / 7500.0);
                        swing.setPower(swingSpeed * -0.8);
                    }
                } else if (swingSpeed < 0) {
                    pitch.setPosition(0.8 - (swing.getCurrentPosition() - initialPosition) / 7500.0);
                    swing.setPower(swingSpeed * -0.8);
                } else {
                    swing.setPower(0);
                }

                extend.setPower(extendSpeed);
                //Arm Servos (Yaw, Grabber, Pitch)

                if (gamepad2.a) {
                    while (!(yaw.getPosition() == 1)) {
                        yaw.setPosition(yaw.getPosition() + 0.02);
                        sleep(50);
                        telemetry.addData("YAW", yaw.getPosition());
                        telemetry.update();
                    }
                }
                if (gamepad2.b) {
                    while (!(yaw.getPosition() == 0)) {
                        yaw.setPosition(yaw.getPosition() - 0.02);
                        sleep(50);
                        telemetry.addData("YAW", yaw.getPosition());
                        telemetry.update();

                        if (gamepad2.dpad_left) {
                            armGrabber.setPosition(armGrabber.getPosition() + 0.005);
                        }
                        if (gamepad2.dpad_right) {
                            armGrabber.setPosition(armGrabber.getPosition() - 0.005);
                        }

                        if (gamepad2.dpad_down) {
                            pitch.setPosition(pitch.getPosition() + 0.005);
                        }
                        if (gamepad2.dpad_up) {
                            pitch.setPosition(pitch.getPosition() - 0.005);
                        }


                    }
                }
                telemetry.addData("push", push.getPosition());
                telemetry.addData("Left Joy value", gamepad2.left_stick_y);
                telemetry.addData("Swing", swing.getCurrentPosition());
                telemetry.addData("extend", extend.getCurrentPosition());
                telemetry.addData("pitch", pitch.getPosition());
                telemetry.update();
            }
        }
    }
}










