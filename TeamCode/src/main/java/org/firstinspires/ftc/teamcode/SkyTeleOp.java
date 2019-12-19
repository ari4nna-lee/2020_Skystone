package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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
        // Put initialization blocks here.

        waitForStart();
        if (opModeIsActive()) {
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightInduction.setDirection(DcMotorSimple.Direction.REVERSE);
            swing.setDirection(DcMotorSimple.Direction.REVERSE);
            extend.setDirection(DcMotorSimple.Direction.REVERSE);
            hookLeft.setDirection(Servo.Direction.REVERSE);
            swing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Put run blocks here.

            while (opModeIsActive()) {
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
                } else {
                    leftInduction.setPower(0);
                    rightInduction.setPower(0);
                }
                //Hooks
                if (gamepad1.a) {
                    hookLeft.setPosition(1);
                    hookRight.setPosition(1);
                } else if (gamepad1.b) {
                    hookLeft.setPosition(-1);
                    hookRight.setPosition(-1);
                } else {
                    // do nothing
                }

                //Arm Movement
                if (gamepad2.left_stick_y > 0) {
                    swing.setPower(0.7);
                } else if (gamepad2.left_stick_y < 0) {
                    swing.setPower(-0.7);
                } else {
                    swing.setPower(0);
                }
                if (gamepad2.left_stick_x > 0) {
                    extend.setPower(0.7);
                } else if (gamepad2.left_stick_x < 0) {
                    extend.setPower(-0.7);
                } else {
                    extend.setPower(0);
                }


               }
            }
        }
    }










