package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

@TeleOp(name = "My Navigation OpMode")
public class MyNavigationOpMode extends LinearOpMode {
    final double COUNTS_PER_INCH = 306.382254;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        // create robot Navigator
        Navigator navigator = new Navigator(this);
        // start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = navigator.getGlobalPositionRunnable();
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        navigator.goToPosition(0, 24, 0.6, 0, 1);
        //navigator.goToPosition(24, 24, 0.6, 0, 1);
        //navigator.goToPosition(0, 0, 0.6, 0, 1);
        //navigator.pivotToOrientation(90, 0.4, 3);
        navigator.stop();
        //sleep(2000);
        //navigator.goToPosition((globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH + 12.0), globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, 0.7, globalPositionUpdate.returnOrientation(), 1);

        //navigator.stop();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();


    }
}
