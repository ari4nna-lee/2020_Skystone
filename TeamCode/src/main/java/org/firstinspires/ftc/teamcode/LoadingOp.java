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

@Autonomous(name = "LoadingOp (Studio)", group = "")
public class LoadingOp extends LinearOpMode {
    final double COUNTS_PER_INCH = 306.382254;
    final double MOTOR_POWER = 0.7;
    OdometryGlobalCoordinatePosition globalPositionUpdate;

    enum Location {
        BLUE_SIDE,
        RED_SIDE
    }

    enum SkystoneLocation {
        FIRST,
        SECOND,
        THIRD
    }

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DistanceSensor distanceFront;
    private Servo sideGrabber;
    private DistanceSensor distanceLeft;

    @Override
    public void runOpMode() {

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        sideGrabber = hardwareMap.servo.get("sideGrabber");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // create robot Navigator
        Navigator navigator = new Navigator(this);
        // start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = navigator.getGlobalPositionRunnable();
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        if (opModeIsActive()) {
            Location location = getLocation();
            SkystoneLocation skystoneLocation = getSkystoneLocation();
            sideGrabber.setPosition(0);

            if (location == Location.BLUE_SIDE) {
                if (skystoneLocation == SkystoneLocation.FIRST) {
                    navigator.goToPosition(0, 4, MOTOR_POWER, 0, 1);
                    navigator.goToPosition(-25, 4, MOTOR_POWER, 0, 1);
                    navigator.stop();
                    sleep(100);
                    double distance = distanceLeft.getDistance(DistanceUnit.INCH);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH - distance, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, 0.5, 0, 1 );
                    sleep(1000);
                    sideGrabber.setPosition(1);
                    sleep(500);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH + 8, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, MOTOR_POWER, 0, 1);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, 46, MOTOR_POWER, 0, 1);
                    sideGrabber.setPosition(0);
                    sleep(500);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, -18, MOTOR_POWER, 0, 1);
                    navigator.goToPosition(-27, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, MOTOR_POWER, 0, 1);
                    navigator.stop();
                    sleep(100);
                    distance = distanceLeft.getDistance(DistanceUnit.INCH);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH - distance, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, MOTOR_POWER, 0, 1);
                    sideGrabber.setPosition(1);
                    sleep(500);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH + 8, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, MOTOR_POWER, 0, 1);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, 46, MOTOR_POWER, 0, 1);
                    sideGrabber.setPosition(0);
                    sleep(500);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, 35, MOTOR_POWER, 0, 1);
                    sleep(500);

                } else if (skystoneLocation == SkystoneLocation.SECOND) {

                } else {

                }

            } else {
                if (skystoneLocation == SkystoneLocation.FIRST) {
                    navigator.goToPosition(0, -4, MOTOR_POWER, 0, 1);
                    navigator.goToPosition(-25, -4, MOTOR_POWER, 0, 1);
                    navigator.stop();
                    sleep(100);
                    double distance = distanceLeft.getDistance(DistanceUnit.INCH);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH - distance, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, 0.5, 0, 1 );
                    sleep(1000);
                    sideGrabber.setPosition(1);
                    sleep(500);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH + 8, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, MOTOR_POWER, 0, 1);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, -46, MOTOR_POWER, 0, 1);
                    sideGrabber.setPosition(0);
                    sleep(500);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, 18, MOTOR_POWER, 0, 1);
                    navigator.goToPosition(-20, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, MOTOR_POWER, 0, 1);
                    navigator.stop();
                    sleep(100);
                    distance = distanceLeft.getDistance(DistanceUnit.INCH);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH - distance, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, 0.5, 0, 1 );
                    sleep(1000);
                    sideGrabber.setPosition(1);
                    sleep(500);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH + 8, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, MOTOR_POWER, 0, 1);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, -46, MOTOR_POWER, 0, 1);
                    sideGrabber.setPosition(0);
                    sleep(500);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, -35, MOTOR_POWER, 0, 1);
                    sleep(500);




                } else if (skystoneLocation == SkystoneLocation.SECOND) {

                } else {

                }

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


        }

    }

    private Location getLocation() {
        // TODO - use vuforia to locate the robot
        return Location.RED_SIDE;
    }
    private SkystoneLocation getSkystoneLocation() {
        // TODO - use vuforia to locate the skystones
        return SkystoneLocation.FIRST;
    }
}
