package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name = "BlueColorTest (studio)", group = "")
public class ColorLineTest extends LinearOpMode {
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private ColorSensor rightColor;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int currentColor;

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightColor = hardwareMap.colorSensor.get("rightColor");

        // Put initialization blocks here.
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        leftFront.setPower(0.2);
        rightFront.setPower(0.2);
        leftBack.setPower(0.2);
        rightBack.setPower(0.2);
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                currentColor = Color.rgb(rightColor.red(), rightColor.green(), rightColor.blue());
                if (JavaUtil.colorToSaturation(currentColor) >= 0.4 && JavaUtil.colorToHue(currentColor) > 180 && JavaUtil.colorToHue(currentColor) < 275) {
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                }
                telemetry.addData("Saturation", JavaUtil.colorToSaturation(currentColor));
                telemetry.addData("Hue", JavaUtil.colorToHue(currentColor));
                telemetry.update();
            }
        }
    }
}
