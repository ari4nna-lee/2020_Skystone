package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "MecanumAuto (Studio)", group = "")
public class MecanumAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station
     */
    @Override
    public void runOpMode() {

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");

        waitForStart();
        if (opModeIsActive()) {
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

            while (opModeIsActive() || !isStopRequested()) {
                while (runtime.seconds() < 5) {
                    drive(0,0,1);
                }
                drive(0, 0, 0);

            }

        }
    }

    public void drive(double y, double x, double rot) {
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double rightX = rot;

        double lFront = r * Math.cos(robotAngle) + rightX;
        double rFront = r * Math.sin(robotAngle) - rightX;
        double lRear = r * Math.sin(robotAngle) + rightX;
        double rRear = r * Math.cos(robotAngle) - rightX;
        leftFront.setPower(lFront);
        rightFront.setPower(rFront);
        leftBack.setPower(lRear);
        rightBack.setPower(rRear);
    }
}
