package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivetrain.Navigator;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

@Autonomous(name = "Blue FoundationOp", group = "")
public class BlueFoundationOp extends LinearOpMode {
    final double COUNTS_PER_INCH = 306.382254;
    final double MOTOR_POWER = 0.7;
    final double ROTATION_MOTOR_POWER = 0.8;

    private final double HOOK_POS_UP = 1.0;
    private final double HOOK_POS_DOWN = 0.2;
    private final double HOOK_LOCK = 0;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DistanceSensor distanceFront;
    private Servo hookLeft;
    private Servo hookRight;
    private Servo yaw;

    @Override
    public void runOpMode() {

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        hookLeft = hardwareMap.servo.get("hookLeft");
        hookRight = hardwareMap.servo.get("hookRight");
        yaw = hardwareMap.servo.get("yaw");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        hookLeft.setDirection(Servo.Direction.REVERSE);

        hookLeft.setPosition(HOOK_LOCK);
        hookRight.setPosition(HOOK_LOCK);
        yaw.setPosition(0);

        waitForStart();

        // create robot Navigator
        Navigator navigator = new Navigator(this);
        // start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = navigator.getGlobalPositionRunnable();
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        if (opModeIsActive()) {
            hookLeft.setPosition(HOOK_POS_UP);
            hookRight.setPosition(HOOK_POS_UP);

            navigator.goToPosition(-17, 0, 0.6, 0, 1);
            navigator.goToPosition(-17, 22, 0.5, 0, 1);
            navigator.stop();
            sleep(100);
            double distance = (distanceFront.getDistance(DistanceUnit.INCH) - 4);

            navigator.goToPosition(-17, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH + distance, 0.3, 0, 1);
            hookLeft.setPosition(HOOK_POS_DOWN);
            hookRight.setPosition(HOOK_POS_DOWN);
            sleep(500);

            navigator.tankDrive(-1.0, -0.4, 1000);
            navigator.pivotToOrientation(-93, ROTATION_MOTOR_POWER, 5);

            hookLeft.setPosition(HOOK_POS_UP);
            hookRight.setPosition(HOOK_POS_UP);

            navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH - 2, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, MOTOR_POWER, -90, 1);
            navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, 15, MOTOR_POWER, -90, 1);
            navigator.goToPosition(40, 15, MOTOR_POWER, -90, 2);

            navigator.stop();

            while (opModeIsActive()) {
                //Display Global (x, y, theta) coordinates
                telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

                telemetry.addData("Thread Active", positionThread.isAlive());
                telemetry.update();
            }
        }
        globalPositionUpdate.stop();
    }
}





