package org.firstinspires.ftc.teamcode.Autonomus;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

//Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Team code imports
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotverticalslides.DriveActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalWristActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalIntakeActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalGrabberActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalWristActions;
//import org.firstinspires.ftc.teamcode.Autonomus.Configuration;


@Config
@Autonomous(name = "MainAuto", group = "Autonomus")
public class MainAutonomus extends LinearOpMode {

    private DriveActions driveActions = null;
    private HorizontalSlideActions horizontalSlide = null;
    private HorizontalWristActions horizontalWrist = null;
    private HorizontalIntakeActions horizontalIntake = null;
    private VerticalSlideActions verticalSlide = null;
    private VerticalWristActions verticalWrist = null;
    private VerticalGrabberActions verticalGrabber = null;

    //Configuration config = new Configuration;

    @Override
    public void runOpMode() throws InterruptedException {
        //set up all the hardware map for the robot and instantiate them as a object
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

        //trajectory from initial spot moving to blue parking spot
        //todo find the correct blue park position and put it below

        TrajectoryActionBuilder parkBlue = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(20,30), Math.PI/2);

        //trajectory from initial spot moving to red parking spot
        TrajectoryActionBuilder parkRed = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(20,30), Math.PI/2);


        //wait for the start button to be press
        waitForStart();
        //if stop button is press, automatically stop
        if (isStopRequested()) return;
        //choosing which trajectory to take, (so far only one is made)
        Action trajectoryActionChosen;
            trajectoryActionChosen = parkBlue.build();

        //run the chosen action blocking
        Actions.runBlocking(
            new SequentialAction(
                trajectoryActionChosen)

        );
    }
}



