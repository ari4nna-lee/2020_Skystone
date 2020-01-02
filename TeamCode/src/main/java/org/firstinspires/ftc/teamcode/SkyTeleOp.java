package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "SkyTeleOp (Studio)", group = "")
public class SkyTeleOp extends LinearOpMode {

    private final double SWING_POS_DIVISOR = 8500.0;
    private final double PITCH_LEFT_POS = 0.70;
    private final double PITCH_RIGHT_POS = 0.51;

    private final double PUSH_POS_UP = 1.0;
    private final double PUSH_POS_DOWN = 0;

    private final double HOOK_POS_UP = 1.0;
    private final double HOOK_POS_DOWN = 0;

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
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightInduction.setDirection(DcMotorSimple.Direction.REVERSE);

            swing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            swing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            swing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            swing.setDirection(DcMotorSimple.Direction.FORWARD);

            extend.setDirection(DcMotorSimple.Direction.FORWARD);
            hookLeft.setDirection(Servo.Direction.REVERSE);
            extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int initialPosition = swing.getCurrentPosition();

            // Put run blocks here.
            yaw.setPosition(0);
            armGrabber.setPosition(0.1839);
            pitch.setPosition(0.72);
            push.setPosition(1);


            while (opModeIsActive() || !isStopRequested()) {
                float swingSpeed = gamepad2.left_stick_y;
                float extendSpeed = gamepad2.right_stick_y;

                // Mecanum Drive
                double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y * -1);
                double robotAngle = Math.atan2(gamepad1.left_stick_y * -1, gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = gamepad1.right_stick_x;

                double lFront = r * Math.cos(robotAngle) + rightX;
                double rFront = r * Math.sin(robotAngle) - rightX;
                double lRear = r * Math.sin(robotAngle) + rightX;
                double rRear = r * Math.cos(robotAngle) - rightX;
                leftFront.setPower(lFront);
                rightFront.setPower(rFront);
                leftBack.setPower(lRear);
                rightBack.setPower(rRear);

                //Intake System
                if (gamepad1.right_bumper) {
                    leftInduction.setPower(0.8);
                    rightInduction.setPower(0.8);
                } else if (gamepad1.right_trigger > 0) {
                    leftInduction.setPower(-0.8);
                    rightInduction.setPower(-0.8);
                    if (push.getPosition() != PUSH_POS_UP) {
                        push.setPosition(PUSH_POS_UP);
                    }
                } else {
                    leftInduction.setPower(0);
                    rightInduction.setPower(0);
                }
                //Manual Push Control
                if (gamepad1.x){
                    push.setPosition(PUSH_POS_DOWN);
                }
                if (gamepad1.y){
                    push.setPosition(PUSH_POS_UP);
                }


                //Hooks
                if (gamepad1.a) {
                    hookLeft.setPosition(HOOK_POS_UP);
                    hookRight.setPosition(HOOK_POS_UP);
                } else if (gamepad1.b) {
                    hookLeft.setPosition(HOOK_POS_DOWN);
                    hookRight.setPosition(HOOK_POS_DOWN);
                } else {
                    // do nothing
                }

                //Arm Movement
                if (swingSpeed > 0) {  // ARM MOVING DOWN
                    if (switch_TouchSensor.isPressed()) {
                        swing.setPower(0);
                        pitch.setPosition(0.72);
                        armGrabber.setPosition(0.1839);
                    } else {
                        if (yaw.getPosition() > 0.9){
                            pitch.setPosition(PITCH_RIGHT_POS + (swing.getCurrentPosition() - initialPosition) / SWING_POS_DIVISOR);
                        } else {
                            pitch.setPosition(PITCH_LEFT_POS - (swing.getCurrentPosition() - initialPosition) / SWING_POS_DIVISOR);
                        }
                        swing.setPower(swingSpeed * -0.45);
                    }
                } else if (swingSpeed < 0) {   // ARM MOVING UP
                    if (yaw.getPosition() > 0.9){
                        pitch.setPosition(PITCH_RIGHT_POS + (swing.getCurrentPosition() - initialPosition) / SWING_POS_DIVISOR);
                    } else {
                        pitch.setPosition(PITCH_LEFT_POS - (swing.getCurrentPosition() - initialPosition) / SWING_POS_DIVISOR);
                    }
                    swing.setPower(swingSpeed * -0.45);
                } else {
                    swing.setPower(0);
                }

                extend.setPower(extendSpeed * 0.5);
                //Arm Servos (Yaw, Grabber, Pitch)

                if (gamepad2.a) {
                    swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    swing.setPower(0.4);
                    // move the arm to horizontal position
                    swing.setTargetPosition(1000);
                    while(swing.isBusy() && !isStopRequested()) {
                        sleep(50);
                        pitch.setPosition(PITCH_LEFT_POS - (swing.getCurrentPosition() - initialPosition) / SWING_POS_DIVISOR);
                    }
                    swing.setPower(0);
                    swing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    pitch.setPosition(PITCH_RIGHT_POS + 0.11);
                    while (!(yaw.getPosition() == 1) && !isStopRequested()) {
                        yaw.setPosition(yaw.getPosition() + 0.02);
                        sleep(50);
                        telemetry.update();
                    }
                }
                if (gamepad2.b) {
                    while (!(yaw.getPosition() == 0)) {
                        yaw.setPosition(yaw.getPosition() - 0.02);
                        sleep(50);
                    }
                }
                if (gamepad2.x) {
                    pitch.setPosition(0.82);
                    armGrabber.setPosition(0.013);
                    sleep(100);
                    push.setPosition(PUSH_POS_UP);
                }
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
                telemetry.addData("push", push.getPosition());
                telemetry.addData("Left Joy value", gamepad2.left_stick_y);
                telemetry.addData("Swing", swing.getCurrentPosition());
                telemetry.addData("extend", extend.getCurrentPosition());
                telemetry.addData("pitch", pitch.getPosition());
                telemetry.addData("arm grabber", armGrabber.getPosition());
                telemetry.addData("yaw", yaw.getPosition());
                telemetry.update();
            }
        }
    }
}










