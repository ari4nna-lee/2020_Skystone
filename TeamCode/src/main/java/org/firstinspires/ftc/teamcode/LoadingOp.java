package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.drivetrain.Navigator;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "LoadingOp (Studio)", group = "")
public class LoadingOp extends LinearOpMode {
    final double COUNTS_PER_INCH = 306.382254;
    final double MOTOR_POWER = 0.7;

    private OdometryGlobalCoordinatePosition globalPositionUpdate;


    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
            "AUgLF0r/////AAABme3TJAJ9xkFXqQanGuvAG80MUHIB7KLDWdGB2wR9lFc/TbFj4oDCEw3YsiLBQwzd6LAqyKG0yKPlrOQxINXiyrttKI3uG8Fs5zuAnIMw+ebKxkg7oLLQCJeFBxu9pjYyu8hE3Tg7kmTJSA0JRGaZYGoBllbNlmAXrjdcNfhYL5WQL3tFwaIs4PLOla8JjMofAdCZvoSmtGYXzBhreJoIkD7SX+dldjI3yEWsoLN8E9Xex8YRSRbylJivW8dxxzWcfPvqsunn8Vbf4ZkaPQD9cbPomyHI3ooLSLad6CYMgnuuLTep990W0jrX+QpxAK8cQzM7CPDKwV/un3xgkS1gmvJuYYp2LyML21CSCvCrsTvk";

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targetsSkyStone = null;
    private List<VuforiaTrackable> allTrackables = new ArrayList<>();

    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

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
    private Servo yaw;

    @Override
    public void runOpMode() {
        initDriveHardwareMap();
        initiateVuforia();
        // Activate Vuforia Skystone tracking
        targetsSkyStone.activate();

        waitForStart();

        sideGrabber.setPosition(0);
        yaw.setPosition(0);


        // create robot Navigator
        Navigator navigator = new Navigator(this);
        // start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = navigator.getGlobalPositionRunnable();
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        if (opModeIsActive()) {
            //1. Move to 12 inches in front of stone wall
            navigator.goToPosition(-12, 0, MOTOR_POWER, 0, 1);
            navigator.stop();

            //2. Call getSkystoneLocation
            ElapsedTime elapsedTime = new ElapsedTime();
            targetsSkyStone.activate();
            telemetry.addData("before ", "getSkystoneLocation");
            telemetry.update();
            SkystoneLocation skystoneLocation = getSkystoneLocation(allTrackables);
            telemetry.addData("Time needed in ms", elapsedTime.milliseconds());

            if (skystoneLocation == SkystoneLocation.FIRST) {
                telemetry.addData("Skystone located", "RIRST");
                /*
                double distance = distanceLeft.getDistance(DistanceUnit.INCH);
                navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH - distance, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, 0.5, 0, 1);
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
               */
            } else if (skystoneLocation == SkystoneLocation.SECOND) {
                // do 2nd
                telemetry.addData("Skystone located", "SECOND");
            } else {
                // do 3rd routine
                telemetry.addData("Skystone located", "THIRD");
            }
            telemetry.update();


            /*} else {
                if (skystoneLocation == SkystoneLocation.FIRST) {
                    //navigator.goToPosition(0, -4, MOTOR_POWER, 0, 1);
                    navigator.goToPosition(-12, 0, MOTOR_POWER, 0, 1);
                    navigator.stop();
                    //Detect the skystones
                    sleep(10000);
                    sleep(100);
                    double distance = distanceLeft.getDistance(DistanceUnit.INCH);
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH - distance, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, 0.5, 0, 1);
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
                    navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH - distance, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, 0.5, 0, 1);
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
                */

            while (opModeIsActive()) {
                //Display Global (x, y, theta) coordinates

//                 telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
//                 telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
//                 telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
//
//                 telemetry.addData("Thread Active", positionThread.isAlive());
//                 telemetry.update();
            }

        }
        navigator.stop();

        targetsSkyStone.deactivate();

        if (globalPositionUpdate != null) {
            globalPositionUpdate.stop();
        }
    }

    private void initDriveHardwareMap() {
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        sideGrabber = hardwareMap.servo.get("sideGrabber");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        yaw = hardwareMap.servo.get("yaw");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Drive Hardware Map Init Complete");
        telemetry.update();
    }

    private void initiateVuforia() {
        // TODO - set 'cameraMonitorViewId' to zero to disable camera monitoring
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        allTrackables.addAll(targetsSkyStone);

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        telemetry.addData("Status", "Vuforia Init Complete");
        telemetry.update();
    }

    private SkystoneLocation getSkystoneLocation(List<VuforiaTrackable> allTrackables) {
        SkystoneLocation skystoneLocation = SkystoneLocation.THIRD;

        ElapsedTime elapsedTime = new ElapsedTime();
        boolean targetVisible = false;

        while (!isStopRequested() &&  !targetVisible && (elapsedTime.milliseconds() < 200)) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                double yValue = translation.get(1);
                if (yValue < 0) {
                    skystoneLocation = SkystoneLocation.FIRST;
                } else {
                    skystoneLocation = SkystoneLocation.SECOND;
                }

            } else {
                telemetry.addData("Visible Target", "none");
                skystoneLocation = SkystoneLocation.THIRD;
            }
            telemetry.update();
        }
        return skystoneLocation;
    }
}
