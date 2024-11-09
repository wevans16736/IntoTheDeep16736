package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

//Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

//Team code imports
import org.firstinspires.ftc.teamcode.robotverticalslides.DriveActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalWristActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalIntakeActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalGrabberActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalWristActions;


@Config
@Autonomous(name = "Backup Auto", group = "Autonomus")
public class BackupAuto extends LinearOpMode{

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
            //Putting initial sample on rung
            //Flip wrist out
            verticalWrist.flipping(true);
            //set slide up
            verticalSlide.setSlidePosition(-540, 2500);
            //drive forward 30 inches

            driveActions.drive(0, 0.2, 0);
            sleep(3800);
            driveActions.drive(0, 0, 0);
            verticalGrabber.open();
            sleep(500);
            //drive back 5 inches
            driveActions.drive(0, -0.5, 0);
            sleep(400);
            driveActions.drive(0, 0, 0);

            driveActions.drive(0, 0, 0.5);
            sleep(800);
            driveActions.drive(0, 0, 0);

            //reset outtake arm
            verticalGrabber.close();
            sleep(500);
            verticalWrist.flipping(false);
            verticalWrist.flipping(true);
            verticalSlide.setSlidePosition(0, 2000);

            driveActions.drive(0, 0.5, 0);
            sleep(1300);
            driveActions.drive(0, 0, 0);
        }
    }
}
