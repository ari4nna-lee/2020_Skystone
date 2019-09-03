package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MecaTestOpMode3 (studio)", group = "")
public class MecanumOpMode extends LinearOpMode {

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftRearSpeed;
        double rightRearSpeed;

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.update();
                leftFrontSpeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
                telemetry.addData("Left Front Position", leftFront.getCurrentPosition());
                rightFrontSpeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
                telemetry.addData("Right Front Position", rightFront.getCurrentPosition());
                leftRearSpeed = gamepad1.right_stick_y + gamepad1.right_stick_x;
                telemetry.addData("Left Back Position", leftBack.getCurrentPosition());
                rightRearSpeed = gamepad1.right_stick_y - gamepad1.right_stick_x;
                telemetry.addData("Right Back Position", rightBack.getCurrentPosition());
                // clip the right/left values so they never exceced +/- 1
                leftFrontSpeed = Range.clip(leftFrontSpeed, -1, 1);
                rightFrontSpeed = Range.clip(rightFrontSpeed, -1, 1);
                leftRearSpeed = Range.clip(leftRearSpeed, -1, 1);
                rightRearSpeed = Range.clip(rightRearSpeed, -1, 1);
                // Write the values to the motors
                leftFront.setPower(leftFrontSpeed);
                rightFront.setPower(rightFrontSpeed);
                leftBack.setPower(leftRearSpeed);
                rightBack.setPower(rightRearSpeed);
                telemetry.update();
            }
        }
    }
}
