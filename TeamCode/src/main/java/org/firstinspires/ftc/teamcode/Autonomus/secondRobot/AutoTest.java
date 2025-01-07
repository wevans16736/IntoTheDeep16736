package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name="AutoTest", group = "SecondRobot")
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //set up Pinpoint and Pose2d class
        Pose2d pose = new Pose2d(0,0,Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, pose);

        //all of these class is under Configuration.secondRobot
        VerticalSlideRR verticalSlideRR = new VerticalSlideRR(hardwareMap);
        VerticalWristRR verticalWristRR = new VerticalWristRR(hardwareMap);
        VerticalGrabberRR verticalGrabberRR = new VerticalGrabberRR(hardwareMap);

        HorizontalSlideRR horizontalSlideRR = new HorizontalSlideRR(hardwareMap);
        HorizontalRollRR horizontalRollRR = new HorizontalRollRR(hardwareMap);
        HorizontalGrabberRR horizontalGrabberRR = new HorizontalGrabberRR(hardwareMap);
        HorizontalWristRR horizontalWristRR = new HorizontalWristRR(hardwareMap);

        //Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        MecanumDrive.Params params = new MecanumDrive.Params();

        dashboardTelemetry.addData("driveVelocity", params.maxWheelVel);
        dashboardTelemetry.update();

        //set up a trajectory class
        Trajectory trajectory = new Trajectory();
        trajectory.setTrajectory(drive, pose, verticalSlideRR, verticalWristRR, verticalGrabberRR, horizontalSlideRR,
                horizontalRollRR, horizontalGrabberRR, horizontalWristRR, false);
        trajectory.setStartTrajectory();

        //todo ask the driver which trajectory to use

        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

//        run hanging trajectory
        Actions.runBlocking(new SequentialAction(
                trajectory.getTestTrajectory().build()
                ));
        double x = drive.pose.position.x;
        double y = drive.pose.position.y;
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        dashboardTelemetry.addData("x", x);
        dashboardTelemetry.addData("y", y);

        dashboardTelemetry.update();
        telemetry.update();
        Actions.runBlocking(new SleepAction(5));
//                pick up both butter
//                new ParallelAction(
//                        trajectory.getButterPickUpAttachment().build(),
//                        trajectory.getSecondButterPickUpTrajectory().build()
//                ),
//                trajectory.getButterPickUpAttachment().build(),
//                //go to human place
//                trajectory.getPostHangLocationTrajectory().build(),
//                new ParallelAction(
//                        trajectory.getPostHangAttachment().build(),
//                        trajectory.getHangTrajectory().build()
//                ),
//                trajectory.getPostHangLocationTrajectory().build(),
//                new ParallelAction(
//                        trajectory.getPostHangAttachment().build(),
//                        trajectory.getHangTrajectory().build()
//                ),
//                trajectory.getPark().build()
//                ));
    }
}
