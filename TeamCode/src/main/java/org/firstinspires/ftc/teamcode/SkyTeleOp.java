package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "SkyTeleOp (Studio)", group = "")
public class SkyTeleOp extends LinearOpMode {

    private final double SERVO_POS_INTERVAL = 0.02;
    private final int SERVO_SLEEP_INTERVAL = 20;

    private final double ARM_GRABBER_MIN_POS = 0.3;
    private final double ARM_GRABBER_MAX_POS = 0.55;

    private final double CAPSTONE_MIN_POS = 0.33;
    private final double CAPSTONE_MAX_POS = 1.0;

    private final double PUSH_POS_UP = 1.0;
    private final double PUSH_POS_DOWN = 0;

    private final double SIDE_GRABBER_UP_POS = 0;

    private final double HOOK_POS_UP = 1.0;
    private final double HOOK_POS_DOWN = 0;

    final double POWER_MULTIPLIER = 9.0 / 7.0;

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor leftInduction;
    private DcMotor rightInduction;
    private Servo hookLeft;
    private Servo hookRight;
    private DcMotor extend;
    private DcMotor vertical;
    private Servo armGrabber;
    private Servo push;
    private Servo sideGrabber;
    private Servo capstone;
    private TouchSensor touch;

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
        vertical = hardwareMap.dcMotor.get("vertical");
        extend = hardwareMap.dcMotor.get("extend");
        push = hardwareMap.servo.get("push");
        armGrabber = hardwareMap.servo.get("armGrabber");
        sideGrabber = hardwareMap.servo.get("sideGrabber");
        capstone = hardwareMap.servo.get("capstone");
        touch = hardwareMap.touchSensor.get("switch");

        // Put initialization blocks here.
        // Set the armgrabber and pitch positions so it's within the box
        armGrabber.setPosition(ARM_GRABBER_MAX_POS);
        sideGrabber.setPosition(SIDE_GRABBER_UP_POS);
        capstone.setPosition(CAPSTONE_MIN_POS);

        waitForStart();

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightInduction.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extend.setDirection(DcMotorSimple.Direction.FORWARD);
        hookLeft.setDirection(Servo.Direction.REVERSE);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Put run blocks here.
        // armGrabber.setPosition(ARM_GRABBER_RESET_POS);
        push.setPosition(PUSH_POS_UP);
        sideGrabber.setPosition(SIDE_GRABBER_UP_POS);

        while (opModeIsActive() || !isStopRequested()) {

            // By default using Mecanum drive control
            // Holding Gamepad1 'Start' button to use Tank Drive mode
            if (gamepad1.start) {
                tankDrive();
            } else {
                mecanumDrive();
            }

            //Intake System
            if (gamepad1.left_bumper) {
                leftInduction.setPower(0.8);
                rightInduction.setPower(0.8);
            } else if (gamepad1.right_bumper) {
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
            if (gamepad1.x) {
                push.setPosition(PUSH_POS_DOWN);
            }
            if (gamepad1.y) {
                push.setPosition(PUSH_POS_UP);
            }

            if (gamepad2.left_stick_y > 0){
                if (touch.isPressed()){
                    vertical.setPower(0);
                } else {
                    if (vertical.getCurrentPosition() > -200){
                        vertical.setPower(0.5);
                    } else {
                        vertical.setPower(1);
                    }
                }
            } else if (gamepad2.left_stick_y < 0){
                vertical.setPower(-1);
            } else {
                vertical.setPower(0);
            }
            extend.setPower(gamepad2.right_stick_y);

            //Hooks
            if (gamepad1.a) {
                hookLeft.setPosition(HOOK_POS_UP);
                hookRight.setPosition(HOOK_POS_UP);
            }
            if (gamepad1.b) {
                hookLeft.setPosition(HOOK_POS_DOWN);
                hookRight.setPosition(HOOK_POS_DOWN);
            }

            if (gamepad2.dpad_up) {
                if (capstone.getPosition() != CAPSTONE_MAX_POS) {
                    capstone.setPosition(capstone.getPosition() + SERVO_POS_INTERVAL);
                    sleep(SERVO_SLEEP_INTERVAL);
                }
            }
            if (gamepad2.dpad_down) {
                if (capstone.getPosition() > CAPSTONE_MIN_POS) {
                    capstone.setPosition(capstone.getPosition() - SERVO_POS_INTERVAL);
                    sleep(SERVO_SLEEP_INTERVAL);
                }
            }

            if (gamepad2.a) {
                armGrabber.setPosition(ARM_GRABBER_MIN_POS);
            }
            if (gamepad2.b) {
                armGrabber.setPosition(ARM_GRABBER_MAX_POS);
            }

            telemetry.addData("push", push.getPosition());
            telemetry.addData("Left Joy value", gamepad2.left_stick_y);
            telemetry.addData("extend", extend.getCurrentPosition());
            telemetry.addData("vertical", vertical.getCurrentPosition());
            telemetry.addData("arm grabber", armGrabber.getPosition());
            telemetry.update();
        }

    }

    // Tank drive mode
    private void tankDrive() {
        double leftPower = gamepad1.left_stick_y * -1;
        double rightPower = gamepad1.right_stick_y * -1;
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);
    }

    // Mecanum Drive
    private void mecanumDrive() {
        float rawX = gamepad1.left_stick_x;
        float rawLeftTrigger = gamepad1.right_trigger;
        float rawRightTrigger = gamepad1.left_trigger;
        if (rawLeftTrigger > 0) {
            rawX = rawLeftTrigger;
        }
        if (rawRightTrigger > 0) {
            rawX = rawRightTrigger * -1;
        }

        double r = Math.hypot(rawX, gamepad1.left_stick_y * -1);
        double robotAngle = Math.atan2(gamepad1.left_stick_y * -1, rawX) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        double lFront = (r * Math.cos(robotAngle) + rightX) * POWER_MULTIPLIER;
        double rFront = (r * Math.sin(robotAngle) - rightX) * POWER_MULTIPLIER;
        double lRear = (r * Math.sin(robotAngle) + rightX) * POWER_MULTIPLIER;
        double rRear = (r * Math.cos(robotAngle) - rightX) * POWER_MULTIPLIER;
        leftFront.setPower(lFront);
        rightFront.setPower(rFront);
        leftBack.setPower(lRear);
        rightBack.setPower(rRear);
    }
}












