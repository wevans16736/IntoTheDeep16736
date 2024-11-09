package org.firstinspires.ftc.teamcode;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotverticalslides.DriveActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalIntakeActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalWristActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalGrabberActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalWristActions;


@Config
@Autonomous(name = "Backup Auto Left", group = "Autonomus")
public class BackupAutoLeft extends LinearOpMode{

    private DriveActions driveActions = null;
    private HorizontalSlideActions horizontalSlide = null;
    private HorizontalWristActions horizontalWrist = null;
    private HorizontalIntakeActions horizontalIntake = null;
    private VerticalSlideActions verticalSlide = null;
    private VerticalWristActions verticalWrist = null;
    private VerticalGrabberActions verticalGrabber = null;

    @Override
    public void runOpMode() throws InterruptedException {
        driveActions = new DriveActions(telemetry, hardwareMap);
        horizontalSlide = new HorizontalSlideActions(hardwareMap, telemetry);
        horizontalWrist = new HorizontalWristActions(telemetry, hardwareMap);
        horizontalIntake = new HorizontalIntakeActions(telemetry, hardwareMap);
        verticalSlide = new VerticalSlideActions(hardwareMap, telemetry);
        verticalWrist = new VerticalWristActions(telemetry, hardwareMap);
        verticalGrabber = new VerticalGrabberActions(telemetry, hardwareMap);

        //instantiate the robot to a particular pose.
        //todo find the correct initial position and put it below
        Pose2d initialPose = new Pose2d(0,0, Math.toRadians(90));
        Vector2d vector2d = new Vector2d(0,0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            //keep horzontal slide static
            horizontalSlide.setSlidePosition(0, 200);
            //Putting initial sample on rung
            //Flip wrist out
            verticalWrist.flipping(true);
            //set slide up
            verticalSlide.setSlidePosition(-540, 2500);
            //drive forward 30 inches

            driveActions.drive(0, 0.3, 0);
            sleep(3400);
            driveActions.drive(0, 0, 0);
            verticalGrabber.open();
            sleep(500);
            //drive back 5 inches
            driveActions.drive(0, -0.5, 0);
            sleep(600);
            driveActions.drive(0, 0, 0);

            //reset outtake arm
            verticalGrabber.close();
            sleep(500);
            verticalWrist.flipping(false);
            verticalWrist.flipping(true);
            verticalSlide.setSlidePosition(0, 2000);
            sleep(1000);

        }
    }
}
