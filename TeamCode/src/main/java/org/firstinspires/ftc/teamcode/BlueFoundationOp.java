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
    final double BASE_POWER_VERTICAL = 0.3;
    final double BASE_POWER_HORIZONTAL = 0.55;
    final double ROTATION_MOTOR_POWER = 0.8;

    private final double HOOK_POS_UP = 1.0;
    private final double HOOK_POS_DOWN = 0.25;
    private final double HOOK_LOCK = 0;

    private final double ARM_GRABBER_MAX_POS = 0.55;
    private final double SIDE_GRABBER_UP_POS = 0;
    private final double CAPSTONE_MIN_POS = 0.33;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DistanceSensor distanceFront;
    private DistanceSensor distanceBack;
    private DistanceSensor distanceRight;
    private Servo hookLeft;
    private Servo hookRight;
    private Servo frontSwing;
    private Servo frontGrabber;
    private Servo backSwing;
    private Servo backGrabber;
    private Servo armGrabber;
    private Servo capstone;

    @Override
    public void runOpMode() {

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        hookLeft = hardwareMap.servo.get("hookLeft");
        hookRight = hardwareMap.servo.get("hookRight");
        armGrabber = hardwareMap.servo.get("armGrabber");
        frontSwing = hardwareMap.servo.get("frontSwing");
        frontGrabber = hardwareMap.servo.get("frontGrabber");
        backSwing = hardwareMap.servo.get("backSwing");
        backGrabber = hardwareMap.servo.get("backGrabber");
        capstone = hardwareMap.servo.get("capstone");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");
        distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        hookLeft.setDirection(Servo.Direction.REVERSE);

        hookLeft.setPosition(HOOK_POS_UP);
        hookRight.setPosition(HOOK_POS_UP);
        frontSwing.setPosition(0);
        frontGrabber.setPosition(0);
        backSwing.setPosition(0);
        backGrabber.setPosition(0);

        armGrabber.setPosition(ARM_GRABBER_MAX_POS);
        capstone.setPosition(CAPSTONE_MIN_POS);

        waitForStart();

        // create robot Navigator
        Navigator navigator = new Navigator(this);
        // start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = navigator.getGlobalPositionRunnable();
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        if (opModeIsActive()) {

            navigator.goToPosition(14, 0, BASE_POWER_HORIZONTAL, 0, 1);
            navigator.goToPosition(14, -22, BASE_POWER_VERTICAL, 0, 1);
            navigator.stop();
            sleep(500);
            double m = distanceFront.getDistance(DistanceUnit.INCH);
            if (m > 22) {
                m = 22;
            }
            double distance = (31 - m);

            navigator.goToPosition(14, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH - distance, BASE_POWER_VERTICAL, 0, 1);
            hookLeft.setPosition(HOOK_POS_DOWN);
            hookRight.setPosition(HOOK_POS_DOWN);
            sleep(500);

            navigator.tankDrive(0.4, 1.0, 1000);
            navigator.pivotToOrientation(-90, ROTATION_MOTOR_POWER, 5);

            hookLeft.setPosition(HOOK_POS_UP);
            hookRight.setPosition(HOOK_POS_UP);
            sleep(500);

            double l = (distanceRight.getDistance(DistanceUnit.INCH) - 2);

            navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH + 4, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, BASE_POWER_HORIZONTAL, -90, 1);
            navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH + l, BASE_POWER_HORIZONTAL, -90, 0.5);
            navigator.goToPosition(-28, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, BASE_POWER_HORIZONTAL, -90, 1);

            navigator.stop();

            while (opModeIsActive()) {
                //Display Global (x, y, theta) coordinates
                telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
                telemetry.addData("DistanceRight", distanceRight.getDistance(DistanceUnit.INCH));

                telemetry.addData("Thread Active", positionThread.isAlive());
                telemetry.update();
            }
        }
        globalPositionUpdate.stop();
    }
}





