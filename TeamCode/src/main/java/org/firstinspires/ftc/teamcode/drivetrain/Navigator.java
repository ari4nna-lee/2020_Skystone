package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

public class Navigator {
    //Drive motors
    private DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    private DcMotor verticalLeft, verticalRight, horizontal;

    private final double COUNTS_PER_INCH = 306.382254;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    private String rfName = "rightFront", rbName = "rightBack", lfName = "leftFront", lbName = "leftBack";
    private String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = lbName;

    private OdometryGlobalCoordinatePosition globalPositionUpdate;
    private LinearOpMode opMode;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Navigator(LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        this.globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        this.globalPositionUpdate.reverseNormalEncoder();
    }

    public OdometryGlobalCoordinatePosition getGlobalPositionRunnable() {
        return this.globalPositionUpdate;
    }

    /**
     * Go to a location in the field
     * @param targetX X coordinate in inches
     * @param targetY Y coordinate in inches
     * @param robotPower desired motor power for the wheels
     * @param desiredRobotOrientation desired robot orientation
     * @param allowableDistanceError allowable distance error in inches
     */
    public void goToPosition(double targetX, double targetY, double robotPower, double desiredRobotOrientation, double allowableDistanceError) {
        double targetXPosition = targetX * COUNTS_PER_INCH;
        double targetYPosition = targetY * COUNTS_PER_INCH;
        double tolerance = allowableDistanceError * COUNTS_PER_INCH;

        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while (opMode.opModeIsActive() && !opMode.isStopRequested() && (distance > tolerance)) {
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget)) - globalPositionUpdate.returnOrientation();

            double robotMovementXComponent = calculateX(robotMovementAngle, robotPower);
            double robotMovementYComponent = calculateY(robotMovementAngle, robotPower);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();


            drive(robotMovementYComponent, robotMovementXComponent, pivotCorrection);

            telemetry.addData("Distance to target", distance);
            telemetry.addData("Allowable Error", tolerance);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.update();
        }
        stop();
    }

    /**
     * Pivot the robot to a specific orientation
     * @param targetAngle target robot orientation in degrees
     * @param robotPower desired robot power
     * @param allowableAngleError allowable angle error
     */
    public void pivotToOrientation(double targetAngle, double robotPower, double allowableAngleError) {

        double currentAngle = globalPositionUpdate.returnOrientation();

        double angleError = Math.abs(targetAngle) - Math.abs(currentAngle);

        while (opMode.opModeIsActive() && !(angleError <= allowableAngleError || opMode.isStopRequested())) {
            currentAngle = globalPositionUpdate.returnOrientation();
            angleError = Math.abs(targetAngle) - Math.abs(currentAngle);

            if (targetAngle < 0) {
                rotate(robotPower * -1);
            } else {
                rotate(robotPower);
            }

            telemetry.addData("Distance to Desired Angle", angleError);
            telemetry.addData("Allowable Error", allowableAngleError);

            telemetry.addData("Orientation", globalPositionUpdate.returnOrientation());
            telemetry.update();
        }
        stop();
    }

    public void stop() {
        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
    }

    private void rotate(double robotPower) {
            double lFront = robotPower;
            double rFront = robotPower * -1;
            double lRear = robotPower;
            double rRear = robotPower * -1;
            left_front.setPower(lFront);
            right_front.setPower(rFront);
            left_back.setPower(lRear);
            right_back.setPower(rRear);
        }

    private void drive(double y, double x, double rot) {
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double rightX = Range.clip(rot / 50.0, -0.7, 0.7);

        telemetry.addData("RAW Rot", rot);
        telemetry.addData("Control rightX", rightX);

        double lFront = r * Math.cos(robotAngle) + rightX;
        double rFront = r * Math.sin(robotAngle) - rightX;
        double lRear = r * Math.sin(robotAngle) + rightX;
        double rRear = r * Math.cos(robotAngle) - rightX;
        left_front.setPower(lFront);
        right_front.setPower(rFront);
        left_back.setPower(lRear);
        right_back.setPower(rRear);

    }
    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.FORWARD);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

}



