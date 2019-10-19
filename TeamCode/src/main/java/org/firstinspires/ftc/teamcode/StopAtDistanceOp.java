package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "StopAtDistanceOp (Studio)", group = "")
public class StopAtDistanceOp extends LinearOpMode {

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DistanceSensor distanceSensor;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        // Put initialization blocks here.
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            leftFront.setPower(-0.5);
            rightFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightBack.setPower(-0.5);
            while (!isStopRequested()) {
                // Put loop blocks here.
                telemetry.update();
                telemetry.addData("Distance ", distanceSensor.getDistance(DistanceUnit.CM));
                if (distanceSensor.getDistance(DistanceUnit.CM) <= 30 && (distanceSensor.getDistance(DistanceUnit.CM) > 10)) {
                    leftFront.setPower(-0.2);
                    rightFront.setPower(0.2);
                    leftBack.setPower(0.2);
                    rightBack.setPower(-0.2);
                }
                else if (distanceSensor.getDistance(DistanceUnit.CM) <= 10) {
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                }
                telemetry.update();
            }
        }
    }
}

