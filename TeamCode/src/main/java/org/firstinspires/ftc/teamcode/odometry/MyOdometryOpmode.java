package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

;import org.firstinspires.ftc.teamcode.drivetrain.PIDController;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 306.382254;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "rightFront", rbName = "rightBack", lfName = "leftFront", lbName = "leftBack";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = lbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseRightEncoder();

        //goToPosition(0*COUNTS_PER_INCH, 24*COUNTS_PER_INCH, 0.25, 0, 1*COUNTS_PER_INCH);
        //goToPosition(24*COUNTS_PER_INCH, 24*COUNTS_PER_INCH, 0.45, 0, 1*COUNTS_PER_INCH);
        // goToPosition( 0*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.7, 0, 1*COUNTS_PER_INCH);
        goToRotation(90, 0.7, 2);
        drive(0, 0, 0);
        sleep(2000);
        /*
        goToPosition(globalPositionUpdate.returnXCoordinate() + 12 * COUNTS_PER_INCH,
                globalPositionUpdate.returnYCoordinate(),
                0.5,
                globalPositionUpdate.returnOrientation(),
                2*COUNTS_PER_INCH);
                */

        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void goToPosition(double targetXPosition, double targetYPosition, double basePower, double desiredRobotOrientation, double allowableDistanceError) {
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance  = Math.hypot(distanceToXTarget, distanceToYTarget);
        double curDistance = distance;

        PIDController pidController = new PIDController(0.00007, 0, 0);
        pidController.setSetpoint(distance);
        pidController.setInputRange(0, distance);
        pidController.setOutputRange(0, 0.4);
        pidController.setTolerance(allowableDistanceError);
        pidController.enable();

        while (opModeIsActive() && !isStopRequested() && ( curDistance > allowableDistanceError)) {
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            curDistance  = Math.hypot(distanceToXTarget, distanceToYTarget);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget)) - globalPositionUpdate.returnOrientation();

            double speedCorrection = pidController.performPID(distance - curDistance);

            double robotMovementXComponent = calculateX(robotMovementAngle, basePower + speedCorrection);
            double robotMovementYComponent = calculateY(robotMovementAngle, basePower + speedCorrection);

            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

            drive(robotMovementYComponent, robotMovementXComponent, pivotCorrection);
            //sleep(50);
            telemetry.addData("X to target", distanceToXTarget);
            telemetry.addData("Y to target", distanceToYTarget);
            telemetry.addData("Distance to target", distance);
            telemetry.addData("Robot Movement Angle", robotMovementAngle);
            telemetry.addData("Speed correction", speedCorrection);
            telemetry.addData("Allowable Error", allowableDistanceError);
            telemetry.addData("Control Y", robotMovementYComponent);
            telemetry.addData("Control X", robotMovementXComponent);
            telemetry.addData("Control Pivot", pivotCorrection);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            //telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            //telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            //telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.update();
        }
    }
    private void goToRotation(double targetAngle, double targetPower, double allowableAngleError) {
        double currentAngle = globalPositionUpdate.returnOrientation();

        double absError = Math.abs(targetAngle) - Math.abs(currentAngle);

        double p = Math.abs(targetPower / targetAngle);
        double i = p / 10000.0;
        double d = 0;
        PIDController pidController = new PIDController(p, i, d);
        pidController.setSetpoint(targetAngle);
        pidController.setInputRange(0, targetAngle);
        pidController.setOutputRange(0, targetPower);
        pidController.setTolerance(allowableAngleError);
        pidController.enable();

        while (opModeIsActive() && !(absError <= allowableAngleError || isStopRequested())) {
            currentAngle = globalPositionUpdate.returnOrientation();
            absError = Math.abs(targetAngle) - Math.abs(currentAngle);

            double power = pidController.performPID(currentAngle);
            power = Range.clip(power, 0.3, targetPower);
            if (targetAngle < 0){
                rotate(power * -1);
            } else {
                rotate(power);
            }

            telemetry.addData("Distance to Desired Angle", absError);
            telemetry.addData("Allowable Error", allowableAngleError);
            telemetry.addData("PID calculated power", power);

            telemetry.addData("Orientation", globalPositionUpdate.returnOrientation());
            telemetry.update();
        }
        drive(0, 0, 0);
            //left_front.setPower(0);
            //right_front.setPower(0);
            //left_back.setPower(0);
            //right_back.setPower(0);
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
        double rightX = Range.clip(rot / 40.0, -0.4, 0.4);

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

        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

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
