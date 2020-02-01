package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@Autonomous(name = "RED LoadingOp", group = "")
public class RedLoadingOp extends LinearOpMode {
    final double COUNTS_PER_INCH = 306.382254;
    final double BASE_POWER_VERTICAL = 0.3;
    final double BASE_POWER_HORIZONTAL = 0.55;

    private final double ARM_GRABBER_MAX_POS = 0.55;
    private final double SIDE_GRABBER_UP_POS = 0;
    private final double CAPSTONE_MIN_POS = 0.33;

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
    Navigator navigator = null;

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
    private Servo armGrabber;
    private Servo capstone;
    private Servo hookLeft;
    private Servo hookRight;
    private DistanceSensor distanceLeft;
    private DistanceSensor distanceBack;

    @Override
    public void runOpMode() {
        initDriveHardwareMap();
        initiateVuforia();

        hookLeft.setPosition(0);
        hookRight.setPosition(0);
        sideGrabber.setPosition(0);

        armGrabber.setPosition(ARM_GRABBER_MAX_POS);
        sideGrabber.setPosition(SIDE_GRABBER_UP_POS);
        capstone.setPosition(CAPSTONE_MIN_POS);

        //TODO initiate arm/pitch servo

        waitForStart();

        // create robot Navigator
        navigator = new Navigator(this);
        // start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = navigator.getGlobalPositionRunnable();
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        if (opModeIsActive()) {
            // Activate Vuforia Skystone tracking
            targetsSkyStone.activate();
            //1. Move to about 10 inches in front of stone wall
            double distance = distanceLeft.getDistance(DistanceUnit.INCH) - 8;
            double targetX = -20.0;
            if ((distance > 17.0) && (distance < Math.abs(targetX))) {
                targetX = distance * -1;
            }
            navigator.goToPosition(targetX, 0, BASE_POWER_HORIZONTAL + 0.15, 0, 1);
            sleep(1000);
            navigator.stop();

            //2. Call getSkystoneLocation
            ElapsedTime elapsedTime = new ElapsedTime();
            double yValue = getSkystoneYValue(allTrackables);
            // targetsSkyStone.deactivate();

            double offset;
            double yDistanceToNextStone;
            double nextYTarget = 0.0;
            SkystoneLocation skystoneLocation = getSkystoneLocation(yValue);
            telemetry.addData("Initial distance", distance);
            telemetry.addData("Initial Target X", targetX);
            telemetry.addData("Y value", yValue);
            telemetry.addData("Y POS", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Time needed in ms", elapsedTime.milliseconds());

            String where = "";

            double raw = 0.0;

            if (skystoneLocation == SkystoneLocation.FIRST) {
                where = "FIRST";
                offset = (Math.abs(yValue) / mmPerInch + 1.5) * -1;
                raw = offset;
                offset = Range.clip(offset, -5.0, -3.0);
                yDistanceToNextStone = 23.5;
                nextYTarget = getFirstStoneRoutine(offset, yDistanceToNextStone);

            } else if (skystoneLocation == SkystoneLocation.SECOND) {
                // do 2nd
                where = "SECOND";
                offset = Math.abs(yValue) /mmPerInch - 2;
                raw = offset;
                offset = Range.clip(offset, 1.0, 2.0);
                yDistanceToNextStone = 23.5;
                nextYTarget = getFirstStoneRoutine(offset, yDistanceToNextStone);
            } else {
                // do 3rd routine
                where = "THIRD";
                offset = 7.5;
                yDistanceToNextStone = 8.0;
                nextYTarget = getFirstStoneRoutine(offset, yDistanceToNextStone);
            }

            telemetry.addData("Skystone", where);
            telemetry.addData("RAW OFFSET", raw);
            telemetry.addData("CLIP OFFSET", offset);
            getNextStoneRoutine(nextYTarget);
            telemetry.update();

            while (opModeIsActive()) {
                sleep(50);
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
        hookLeft = hardwareMap.servo.get("hookLeft");
        hookRight = hardwareMap.servo.get("hookRight");
        hookLeft.setDirection(Servo.Direction.REVERSE);
        armGrabber = hardwareMap.servo.get("armGrabber");
        sideGrabber = hardwareMap.servo.get("sideGrabber");
        capstone = hardwareMap.servo.get("capstone");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Drive Hardware Map Init Complete");
        telemetry.update();
    }

    private void initiateVuforia() {
        // TODO - set 'cameraMonitorViewId' to zero to disable camera monitoring
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(0);
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

        final float CAMERA_FORWARD_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 3.0f;     // eg: Camera is ON the robot's center line

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


    private void grabAndMoveStone() {
        sideGrabber.setPosition(1);
        sleep(500);
        navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH - 1, BASE_POWER_VERTICAL, 0, 0.5);
        navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH + 10, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, BASE_POWER_HORIZONTAL, 0, 1);
        navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, -40, BASE_POWER_VERTICAL, 0, 1);
        sideGrabber.setPosition(0);
        sleep(500);
    }

    private double getFirstStoneRoutine(double offset, double yDistanceToNextStone) {
        // move slowly
        navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH,
                globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH + offset, BASE_POWER_HORIZONTAL,0, 1);

        double distance = measureDistanceToTarget();
        double nextYTarget = globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH + yDistanceToNextStone;
        navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH - distance, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, BASE_POWER_HORIZONTAL, 0, 1);

        grabAndMoveStone();
        return nextYTarget;
    }

    private void getNextStoneRoutine(double nextYTarget) {
        //move forward to next Skystone
        navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, nextYTarget, BASE_POWER_VERTICAL, 0, 1);
        navigator.stop();

        double distance = measureDistanceToTarget();
        navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH - distance, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, BASE_POWER_HORIZONTAL, 0, 1);
        navigator.stop();

        grabAndMoveStone();

        // park under the bridge
        navigator.goToPosition(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH -2, -35, BASE_POWER_VERTICAL + 0.1, 0, 1);

        navigator.stop();
    }

    private double measureDistanceToTarget() {
        double distance = distanceLeft.getDistance(DistanceUnit.INCH) + 1;
        if (distance > 15) {
            distance = 12.0;
        }
        return distance;
    }

    private SkystoneLocation getSkystoneLocation(double offset) {
        if (offset == 1000.0) {
            return SkystoneLocation.THIRD;
        }
        if (offset < 0) {
            return SkystoneLocation.FIRST;
        } else {
            return SkystoneLocation.SECOND;
        }
    }

    private double getSkystoneYValue(List<VuforiaTrackable> allTrackables) {
        double yValue = 1000.0;
        ElapsedTime elapsedTime = new ElapsedTime();
        boolean targetVisible = false;

        while (!isStopRequested() &&  !targetVisible && (elapsedTime.milliseconds() < 2000)) {

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

                yValue = translation.get(1);
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }
        return yValue;
    }
}
