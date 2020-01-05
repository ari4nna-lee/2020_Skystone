package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivetrain.Navigator;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

@Autonomous(name = "RedFoundationOp(Studio)", group = "")
public class RedFoundationOp extends LinearOpMode {
    final double COUNTS_PER_INCH = 306.382254;
    final double MOTOR_POWER = 0.7;
    final double ROTATION_MOTOR_POWER = 0.8;

    private final double HOOK_POS_UP = 1.0;
    private final double HOOK_POS_DOWN = 0.28;
    private final double HOOK_LOCK = 0;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    enum RobotLocation {
        BLUE_FOUNDATION,
        RED_FOUNDATION
    }

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
            RobotLocation loc = getLocation();
            hookLeft.setPosition(HOOK_POS_UP);
            hookRight.setPosition(HOOK_POS_UP);

            if (loc == RobotLocation.BLUE_FOUNDATION) {
                navigator.goToPosition(-17, 0, MOTOR_POWER, 0, 1);
                navigator.goToPosition(-17, 22, 0.5, 0, 1);
                navigator.stop();
                sleep(100);
                double distance = (distanceFront.getDistance(DistanceUnit.INCH) - 4);
                /*
                while (opModeIsActive()) {
                    sleep(100);
                    telemetry.addData("distance", distance);
                    telemetry.update();
                }
                */
                navigator.goToPosition(-17, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH + distance, 0.3, 0, 1);
                hookLeft.setPosition(HOOK_POS_DOWN);
                hookRight.setPosition(HOOK_POS_DOWN);
                sleep(500);
                navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH + 2, 20, 0.8, -15, 1);
                navigator.pivotToOrientation(-90, ROTATION_MOTOR_POWER, 5);
                //navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH - 6.0, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, 0.7, -90, 1);
                hookLeft.setPosition(HOOK_POS_UP);
                hookRight.setPosition(HOOK_POS_UP);
                navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, 15, MOTOR_POWER, -90, 1);
                navigator.goToPosition(40, 15, MOTOR_POWER, -90, 2);





            } else {
                navigator.goToPosition(17, 0, MOTOR_POWER, 0, 1);
                navigator.goToPosition(17, 22, MOTOR_POWER, 0, 1);
                navigator.stop();
                sleep(100);
                double distance = (distanceFront.getDistance(DistanceUnit.INCH) - 4);
                navigator.goToPosition(17, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH + distance, 0.3, 0, 1);
                hookLeft.setPosition(HOOK_POS_DOWN);
                hookRight.setPosition(HOOK_POS_DOWN);
                sleep(500);
                navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH - 2, 20, MOTOR_POWER, 15, 1);
                navigator.pivotToOrientation(90, ROTATION_MOTOR_POWER, 5);
                hookLeft.setPosition(HOOK_POS_UP);
                hookRight.setPosition(HOOK_POS_UP);
                sleep(500);
                navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH + 2, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, MOTOR_POWER, 90, 1);
                navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, 15, MOTOR_POWER, 90, 1);
                navigator.goToPosition(-40, 15, MOTOR_POWER, 90, 1);
            }

            navigator.stop();

            while(opModeIsActive()){
                //Display Global (x, y, theta) coordinates
                telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

                telemetry.addData("Thread Active", positionThread.isAlive());
                telemetry.update();
            }
            globalPositionUpdate.stop();
            /*

            ElapsedTime runtime = new ElapsedTime();
            hookLeft.setPosition(1);
            hookRight.setPosition(1);


            while (runtime.seconds() < 3 && (!isStopRequested())) {
                drive(0, -0.8, 0);
                sleep(50);
            }
            drive(0, 0, 0);
            while (!(distanceFront.getDistance(DistanceUnit.CM) <= 15) && (!isStopRequested())) {
                drive(0.5, 0, 0);
            }
            // drive(0, 0, 0);
            stopRobot();
            hookLeft.setPosition(0);
            hookRight.setPosition(0);
            sleep(1000);

            runtime = new ElapsedTime();
            while (runtime.seconds() < 2 && (!isStopRequested())) {
                drive(-0.7, 0.7, 0);
            }
            // drive(0, 0, 0);
            stopRobot();
            runtime = new ElapsedTime();
            while (runtime.seconds() < 1.5 && !isStopRequested()) {
                drive(0, 0, -1);
            }
            // drive(0, 0, 0);
            stopRobot();
            runtime = new ElapsedTime();
            while (runtime.seconds() < 1 && !isStopRequested()) {
                drive(0.5, 0, 0);
            }
            // drive(0, 0, 0);
            stopRobot(); */

        }

    }

    private RobotLocation getLocation() {
        // TODO - use vuforia to locate the robot
        return RobotLocation.RED_FOUNDATION;
    }
}





