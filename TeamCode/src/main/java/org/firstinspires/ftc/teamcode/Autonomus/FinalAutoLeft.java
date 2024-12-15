package org.firstinspires.ftc.teamcode.Autonomus;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.Configuration;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

import org.firstinspires.ftc.teamcode.Configuration.VerticalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.HorizontalRollRR;

@Config
@Autonomous(name = "1. Final Left Auto", group = "Autonomous")
public class FinalAutoLeft extends LinearOpMode {
    VerticalSlideRR verticalSlideRR = new VerticalSlideRR(hardwareMap);
    HorizontalSlideRR horizontalSlideRR = new HorizontalSlideRR(hardwareMap, telemetry);

    VerticalGrabberRR verticalGrabberRR = new VerticalGrabberRR(hardwareMap, telemetry);
    HorizontalGrabberRR horizontalIntakeRR = new HorizontalGrabberRR(hardwareMap, telemetry);

    VerticalWristRR verticalWristRR = new VerticalWristRR(hardwareMap, telemetry);
    HorizontalWristRR horizontalWristRR = new HorizontalWristRR(hardwareMap, telemetry);

    HorizontalRollRR horizontalRollRR = new HorizontalRollRR(hardwareMap, telemetry);
    RobotSpecial robotSpecial = new RobotSpecial();


    public class RobotSpecial {
        public class TransferSystem implements Action{
            boolean primeHorizontal = false;
            boolean pickLeft = false;
            public TransferSystem(){
                primeHorizontal = false;
            }
            public TransferSystem(boolean pickLeft, boolean requestHorizontal){
                primeHorizontal = requestHorizontal;
                this.pickLeft = pickLeft;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(pickLeft) {
                    Actions.runBlocking(new SequentialAction(
                            //let go of the butter on top
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            new SleepAction(.5),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            verticalSlideRR.verticalSlidePosition(Configuration.bottom),
                            new SleepAction(2),
                            //extend the horizontal Slide
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.extend),
                            horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut),
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                            new SleepAction(.5),
                            //grab the butter from the floor
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose),
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            new SleepAction(.5),
                            //retract the slide
                            horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide),
                            horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            new SleepAction(1),
                            //vertical grabber close
                            verticalGrabberRR.verticalGrabberPosition(Configuration.close),
                            new SleepAction(.25),
                            //horizontal grabber open
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                            new SleepAction(.25),
                            verticalSlideRR.verticalSlidePosition(Configuration.topBasket),
                            verticalWristRR.verticalWristPosition(Configuration.forwardUp)
                    ));
                }
                if(!pickLeft) {
                if (primeHorizontal) {
                    Actions.runBlocking(new SequentialAction(
                            //let go of the butter if it is up ontop of the basket
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            verticalSlideRR.verticalSlidePosition(Configuration.bottom),
                            //grab the butter from the floor
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose),
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            new SleepAction(.5),
                            //retract the slide
                            horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide),
                            horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            new SleepAction(1),
                            //vertical grabber close
                            verticalGrabberRR.verticalGrabberPosition(Configuration.close),
                            new SleepAction(.25),
                            //horizontal grabber open
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                            new SleepAction(.25),
                            //butter go to the other side while priming the horizontal slide for other butter
                            verticalWristRR.verticalWristPosition(Configuration.forwardDown),
                            horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut),
                            horizontalSlideRR.horizontalSlidePosition(Configuration.extend),
                            new SleepAction(1.5),
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            new SleepAction(.5),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos)
                    ));
                }
                if (!primeHorizontal) {
                    Actions.runBlocking(new SequentialAction(
                            //grab the butter from the floor
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose),
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            new SleepAction(.5),
                            //retract the slide
                            horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide),
                            horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            new SleepAction(1),
                            //vertical grabber close
                            verticalGrabberRR.verticalGrabberPosition(Configuration.close),
                            new SleepAction(.25),
                            //horizontal grabber open
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                            new SleepAction(.25),
                            verticalWristRR.verticalWristPosition(Configuration.forwardDown),
                            new SleepAction(1),
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            new SleepAction(.25),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos)
                    ));
                }
            }
                return false;
        }
    }
    public Action transferSystem(boolean pickLeft, boolean primeHorizontal) {return new TransferSystem(pickLeft, primeHorizontal);}
    public Action transferSystem() {return new TransferSystem();}
}

@Override
    public void runOpMode() throws InterruptedException  {
        Pose2d currentPose = new Pose2d(0, 0, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        //initialize the robot before starting
        Actions.runBlocking(new SequentialAction(
                horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide),
                verticalSlideRR.verticalSlidePosition(Configuration.bottom),

                horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                verticalGrabberRR.verticalGrabberPosition(Configuration.close),

                horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn),
                verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                horizontalRollRR.horizontalRollPosition(Configuration.flat)
        ));

        TrajectoryActionBuilder startPosition = drive.actionBuilder(currentPose);

        TrajectoryActionBuilder hang = drive.actionBuilder(currentPose)
            .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .waitSeconds(.4)
                .strafeTo(new Vector2d(-16, 28))
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterTime(.25, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .afterTime(.25, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
                .strafeTo(new Vector2d(-16, 24));

        TrajectoryActionBuilder ButterLeft = drive.actionBuilder(currentPose);

        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

        TrajectoryActionBuilder chosenTrajectory;
        Actions.runBlocking(new SequentialAction(startPosition.build()));
        chosenTrajectory = startPosition;

        Action ActionHang = chosenTrajectory.endTrajectory().fresh()
                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .waitSeconds(.4)
                .strafeTo(new Vector2d(-16, 28))
                .afterDisp(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterDisp(2, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .afterDisp(2, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
                .strafeTo(new Vector2d(-16, 24))
                .build();

        Actions.runBlocking(new SequentialAction(ActionHang));
        chosenTrajectory = hang;

        Action ActionButterLeft = chosenTrajectory.endTrajectory().fresh()
                .afterDisp(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterDisp(0, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .afterDisp(5, horizontalSlideRR.horizontalSlidePosition(Configuration.extend))
                .afterDisp(5, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
                .afterDisp(5, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
                .strafeToSplineHeading(new Vector2d(-36, 27.45), Math.toRadians(-90))
                .afterTime(0, robotSpecial.transferSystem(true, false))
                .strafeToSplineHeading(new Vector2d(-40, 5), Math.toRadians(200))
                //maybe need a wait not sure
                .afterTime(0, robotSpecial.transferSystem(true, false))
                .strafeToSplineHeading(new Vector2d(-48.25, 27.45), Math.toRadians(-90))
                //maybe need a wait not sure
                .strafeToSplineHeading(new Vector2d(-40, 5), Math.toRadians(200))
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterDisp(1, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .afterDisp(2, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
                .strafeToSplineHeading(new Vector2d(-30, 10), Math.toRadians(-90))
                .build();

        Actions.runBlocking(new SequentialAction(ActionButterLeft));
        chosenTrajectory = ButterLeft;
    }
}
