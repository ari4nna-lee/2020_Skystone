package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "ParkFoundation Op", group = "")
@Disabled
public class ParkFoundationOp extends LinearOpMode {
    final double COUNTS_PER_INCH = 306.382254;

    private final double HOOK_POS_UP = 1.0;

    private final double ARM_GRABBER_MAX_POS = 0.55;
    private final double CAPSTONE_MIN_POS = 0.33;

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private Servo hookLeft;
    private Servo hookRight;
    private Servo armGrabber;
    private Servo capstone;
    private Servo frontSwing;
    private Servo backSwing;
    private Servo frontGrabber;
    private Servo backGrabber;

    @Override
    public void runOpMode() {

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        hookLeft = hardwareMap.servo.get("hookLeft");
        hookRight = hardwareMap.servo.get("hookRight");
        armGrabber = hardwareMap.servo.get("armGrabber");
        capstone = hardwareMap.servo.get("capstone");
        frontSwing = hardwareMap.servo.get("frontSwing");
        backSwing = hardwareMap.servo.get("backSwing");
        frontGrabber = hardwareMap.servo.get("frontGrabber");
        backGrabber = hardwareMap.servo.get("backGrabber");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        hookLeft.setDirection(Servo.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hookLeft.setPosition(HOOK_POS_UP);
        hookRight.setPosition(HOOK_POS_UP);

        armGrabber.setPosition(ARM_GRABBER_MAX_POS);
        capstone.setPosition(CAPSTONE_MIN_POS);

        frontSwing.setPosition(0);
        backSwing.setPosition(0);
        frontGrabber.setPosition(0);
        backGrabber.setPosition(0);

        waitForStart();

        if (opModeIsActive()) {

            while (!(leftFront.getCurrentPosition() <= -4320 || rightFront.getCurrentPosition() <= -4320)) {
                sleep(24000);
                leftFront.setPower(0.2);
                leftBack.setPower(0.2);
                rightFront.setPower(0.2);
                rightBack.setPower(0.2);
                telemetry.addData("left encoder", leftFront.getCurrentPosition());
                telemetry.addData("right encoder", rightFront.getCurrentPosition());
                telemetry.update();
            }
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }

    }
}





