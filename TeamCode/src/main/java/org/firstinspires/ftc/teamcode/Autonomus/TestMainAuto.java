package org.firstinspires.ftc.teamcode.Autonomus;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

import java.util.Arrays;


@Config
@Autonomous(name = "Test Auto", group = "Autonomous")
public class TestMainAuto extends LinearOpMode {
    private VerticalSlideRR verticalSlideRR = null;
    private HorizontalSlideRR horizontalSlideRR = null;
    private VerticalGrabberRR verticalGrabberRR = null;
    private VerticalWristRR verticalWristRR = null;
    private HorizontalWristRR horizontalWristRR = null;
    private HorizontalGrabberRR horizontalGrabberRR = null;
    private HorizontalRollRR horizontalRollRR = null;

    boolean pickRightSide = false;
    boolean pickLeftSide = false;
    boolean pickHang = false;
    boolean pickHuman = false;
    boolean pickBasket = false;
    boolean pickPark = false;
    boolean pickButter = false;

    int hangSide = 1;
    int butterSide = 1;
    int parkSide = 1;
    int waitHuman = 0;

    public class VerticalSlideRR{
        public DcMotorEx verticalSlide1 = null;
        public DcMotorEx verticalSlide2 = null;

        public VerticalSlideRR(HardwareMap hardwareMap){
            verticalSlide1 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE1);
            verticalSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
            verticalSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide1.setTargetPosition(0);
            verticalSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            verticalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE2);
            verticalSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
            verticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide2.setTargetPosition(0);
            verticalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        public class Liftup implements Action {
            int position = Configuration.highBar;
            private boolean initialized = false;
            public Liftup(int p){
                this.position = p;
            }
            public Liftup(){
                this.position = Configuration.highBar;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double velocity = 1800;
                if (!initialized) {
                    verticalSlide1.setTargetPosition(position);
                    verticalSlide1.setVelocity(velocity);
                    verticalSlide2.setTargetPosition(-position);
                    verticalSlide2.setVelocity(-velocity);
                    initialized = true;
                }
                double currentPosition = verticalSlide1.getCurrentPosition();
                if(currentPosition > position) {
                    return true;
                } else {
                    return false;

                }
            }
        }
        public Action liftUp(int p){
            return new Liftup(p);
        }
        public class SetDown implements Action{
            private boolean initialized = false;
            public boolean run(@NonNull TelemetryPacket packet){
                int position = 0;
                double velocity = 1800;
                if(!initialized){
                    verticalSlide1.setTargetPosition(position);
                    verticalSlide1.setVelocity(velocity);
                    verticalSlide2.setTargetPosition(-position);
                    verticalSlide2.setVelocity(-velocity);
                    initialized = true;
                }
                double currentPosition = verticalSlide1.getCurrentPosition();
                return currentPosition > position;
            }
        }
        public Action setDown(){
            return new SetDown();
        }
    }
    //make a class for horizontal Slide
    public class HorizontalSlideRR{
        public DcMotorEx HorizontalSlide2 = null;
        private Telemetry telemetry;
        public HorizontalSlideRR(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;
            HorizontalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.HORIZONTAL_SLIDE2);
            HorizontalSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
            HorizontalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            HorizontalSlide2.setTargetPosition(0);
            HorizontalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        private boolean initialized = false;
        public class RetractSlide implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int position = Configuration.retractSlide;
                double velocity = 1800;
                if(!initialized){
                    HorizontalSlide2.setTargetPosition(position);
                    HorizontalSlide2.setVelocity(velocity);
                    initialized = true;
                }
                double currentPosition = HorizontalSlide2.getCurrentPosition();
                return currentPosition > position;
            }
        }
        public Action retractSlide(){
            return new RetractSlide();
        }
        public class ExtendButter implements Action{
            private boolean initialized = false;
            int extend = Configuration.extend;
            double extendVelocity = Configuration.extendVelocity;
            public boolean run(@NonNull TelemetryPacket packet) {
                int position = extend;
                double velocity = extendVelocity;
                if (!initialized) {
                    HorizontalSlide2.setTargetPosition(position);
                    HorizontalSlide2.setVelocity(velocity);
                    initialized = true;
                }
                double currentPosition = HorizontalSlide2.getCurrentPosition();
                return currentPosition > position;
            }
        }
        public Action extendButter(){
            return new ExtendButter();
        }
    }
    //add a class for the vertical grabber
    public class VerticalGrabberRR{
        public Servo verticalGrabberServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        private double open = Configuration.open;
        private double close = Configuration.close;
        public VerticalGrabberRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            verticalGrabberServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_GRABBER);
            verticalGrabberServo.setPosition(1.0);
        }
        public class CloseGrabber implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalGrabberServo.setPosition(close);
                return false;
            }
        }
        public Action closeGrabber(){
            return new CloseGrabber();
        }
        public class OpenGrabber implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalGrabberServo.setPosition(open);
                return false;
            }
        }
        public Action openGrabber(){
            return new OpenGrabber();
        }
    }
    //add Vertical Wrist Class
    public class VerticalWristRR{
        public Servo verticalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        double forwardUp = Configuration.forwardUp;
        double forwardDown = Configuration.forwardDown;
        double backwardPos = Configuration.backwardPos;
        public VerticalWristRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            verticalWristServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_WRIST);
            verticalWristServo.setPosition(backwardPos);
        }
        public class PlaceBasket implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalWristServo.setPosition(forwardUp);
                return false;
            }
        }
        public Action placeBasket(){
            return new PlaceBasket();
        }
        public class WallButter implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalWristServo.setPosition(forwardDown);
                return false;
            }
        }
        public Action wallButter(){
            return new WallButter();
        }
        public class TakeButter implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalWristServo.setPosition(backwardPos);
                return false;
            }
        }
        public Action takeButter(){
            return new TakeButter();
        }
    }
    public class HorizontalWristRR{
        public Servo horizontalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        private double backwardPosIn = Configuration.backwardPosIn;
        private double backwardPosOut = Configuration.backwardPosOut;
        private double forwardPosOut = Configuration.forwardPosOut;
        public HorizontalWristRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            horizontalWristServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_WRIST);
            horizontalWristServo.setPosition(backwardPosIn);
        }
        public class GrabButter implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                horizontalWristServo.setPosition(forwardPosOut);
                return false;
            }
        }
        public Action grabButter(){
            return new GrabButter();
        }
        public class InRobot implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                horizontalWristServo.setPosition(backwardPosIn);
                return false;
            }
        }
        public Action inRobot(){
            return new InRobot();
        }
        public class LiftButter implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                horizontalWristServo.setPosition(backwardPosOut);
                return false;
            }
        }
        public Action liftButter(){
            return new LiftButter();
        }
    }
    //HorizontalGrabber class
    public class HorizontalGrabberRR{
        public Servo intakeServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public HorizontalGrabberRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            intakeServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_INTAKE);
            intakeServo.setPosition(0);
        }
        private double floorClose = Configuration.floorClose;
        private double floorOpen = Configuration.floorOpen;
        public class FloorClose implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeServo.setPosition(floorClose);
                return false;
            }
        }
        public Action floorClose(){
            return new FloorClose();
        }
        public class FloorOpen implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeServo.setPosition(floorOpen);
                return false;
            }
        }
        public Action floorOpen(){
            return new FloorOpen();
        }
    }
    public class HorizontalRollRR{
        public Servo rollServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public double flat = Configuration.flat;
        public HorizontalRollRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            rollServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_ROLL);
            rollServo.setPosition(flat);
        }
        public class Flat implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                rollServo.setPosition(flat);
                return false;
            }
        }
        public Action flat(){
            return new Flat();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //instantiate the robot to a particular pose.
        verticalSlideRR = new VerticalSlideRR(hardwareMap);
        horizontalSlideRR = new HorizontalSlideRR(hardwareMap, telemetry);
        verticalGrabberRR = new VerticalGrabberRR(telemetry, hardwareMap);
        verticalWristRR = new VerticalWristRR(telemetry, hardwareMap);
        horizontalWristRR = new HorizontalWristRR(telemetry, hardwareMap);
        horizontalGrabberRR = new HorizontalGrabberRR(telemetry, hardwareMap);
        horizontalRollRR = new HorizontalRollRR(telemetry, hardwareMap);

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        Vector2d vector2d = new Vector2d(0, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        VelConstraint pushBlockVelOverride = new TranslationalVelConstraint(30);
        AccelConstraint pushBlockAccelOverride = new ProfileAccelConstraint(-10, 25);

        VelConstraint parkVelOverride = new TranslationalVelConstraint(80);
        VelConstraint parkAngularOverride = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(60),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint parkAccelOverride = new ProfileAccelConstraint(-50, 50);

        VelConstraint humanVelOverride = new TranslationalVelConstraint(30);
        AccelConstraint humanAccelOverride = new ProfileAccelConstraint(-7, 50);

        TrajectoryActionBuilder deprecatedPark = drive.actionBuilder(initialPose)
                .waitSeconds(.25)
                .strafeTo(new Vector2d(-10, 30), pushBlockVelOverride, pushBlockAccelOverride)
                .afterDisp(0, verticalGrabberRR.openGrabber())
                .waitSeconds(.2)
                .afterDisp(2, verticalSlideRR.setDown())
                .afterDisp(2, verticalWristRR.takeButter())
                .strafeTo(new Vector2d(-10, 15), parkVelOverride, parkAccelOverride)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(26, 20, Math.toRadians(-90)), (-1) * Math.toRadians(90), parkAngularOverride, parkAccelOverride)
                .strafeTo(new Vector2d(27, 60), parkVelOverride, parkAccelOverride)
                .strafeTo(new Vector2d(36, 60), parkAngularOverride, parkAccelOverride)
                .strafeTo(new Vector2d(36, 18), humanVelOverride, humanAccelOverride)
                .strafeTo(new Vector2d(36, 30), parkVelOverride, parkAccelOverride)
                .waitSeconds(3)
                .afterDisp(0.0, verticalWristRR.wallButter())
                .strafeTo(new Vector2d(36, 18), humanVelOverride, humanAccelOverride)
                .afterTime(0.0, verticalGrabberRR.closeGrabber())
                .afterTime(0.0, verticalWristRR.wallButter())
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(36, 30), parkVelOverride, parkAccelOverride)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(0, 12, Math.toRadians(90)), Math.toRadians(90), parkAngularOverride, parkAccelOverride)
                .afterTime(0.0, verticalSlideRR.liftUp(Configuration.highBar))
                .afterTime(0.0, verticalWristRR.wallButter())
                .waitSeconds(.25)
                .strafeTo(new Vector2d(-7, 31), pushBlockVelOverride, pushBlockAccelOverride)
                .afterDisp(0, verticalGrabberRR.openGrabber())
                .waitSeconds(.2)
                .afterDisp(2, verticalSlideRR.setDown())
                .afterDisp(2, verticalWristRR.takeButter())
                .strafeTo(new Vector2d(30, 8), parkVelOverride, parkAccelOverride);

        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(.5);

        TrajectoryActionBuilder hang = drive.actionBuilder(initialPose)
                .afterTime(0.0, verticalSlideRR.liftUp(Configuration.highBar))
                .afterTime(0.0, verticalWristRR.wallButter())
                .waitSeconds(.25)
                .strafeTo(new Vector2d(10*hangSide, 30), pushBlockVelOverride, pushBlockAccelOverride)
                .afterDisp(0, verticalGrabberRR.openGrabber())
                .waitSeconds(.2)
                .afterDisp(2, verticalSlideRR.setDown())
                .afterDisp(2, verticalWristRR.takeButter())
                .strafeTo(new Vector2d(10*hangSide, 15), pushBlockVelOverride, pushBlockAccelOverride);

        TrajectoryActionBuilder butterLeft = drive.actionBuilder(initialPose)
                .afterDisp(2, horizontalSlideRR.extendButter())
                .afterDisp(2, horizontalWristRR.grabButter())
                .afterDisp(2, horizontalGrabberRR.floorOpen())
                .strafeToLinearHeading(new Vector2d(35 * butterSide, 34), Math.toRadians(270))
                .afterTime(0, horizontalGrabberRR.floorClose())
                .afterTime(0, verticalGrabberRR.openGrabber())
                .afterTime(.4, horizontalWristRR.inRobot())
                .afterTime(.4, horizontalSlideRR.retractSlide())
                .afterTime(1.3, verticalGrabberRR.closeGrabber())
                .afterTime(1.6, horizontalGrabberRR.floorOpen())
                .afterTime(1.7, verticalSlideRR.liftUp(Configuration.highBar))
                .afterTime(1.7, verticalWristRR.placeBasket())
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(40 * butterSide, 20), Math.toRadians(220))
                .waitSeconds(2)
                .afterDisp(0, verticalGrabberRR.openGrabber())
                .afterDisp(1, verticalSlideRR.setDown())
                .afterDisp(1, verticalWristRR.takeButter())
                .strafeToLinearHeading(new Vector2d(40 * butterSide, 35), Math.toRadians(270));

        TrajectoryActionBuilder butterRight = drive.actionBuilder(initialPose)
                .afterDisp(2, horizontalSlideRR.extendButter())
                .afterDisp(2, horizontalWristRR.grabButter())
                .afterDisp(2, horizontalGrabberRR.floorOpen())
                .strafeToLinearHeading(new Vector2d(35 * butterSide, 34), Math.toRadians(270))
                .afterTime(0, horizontalGrabberRR.floorClose())
                .waitSeconds(.4)
                .afterDisp(0, horizontalWristRR.liftButter())
                .afterDisp(5, horizontalWristRR.grabButter())
                .strafeToLinearHeading(new Vector2d(35, 20), Math.toRadians(90))
                .afterDisp(0, horizontalGrabberRR.floorOpen())
                .afterDisp(2, horizontalGrabberRR.floorClose())
                .strafeToLinearHeading(new Vector2d(45, 34), Math.toRadians(270))
                .afterTime(0, horizontalGrabberRR.floorClose())
                .waitSeconds(.4)
                .afterDisp(0, horizontalWristRR.liftButter())
                .afterDisp(5, horizontalWristRR.grabButter())
                .strafeToLinearHeading(new Vector2d(45, 20), Math.toRadians(90))
                .afterDisp(0, horizontalGrabberRR.floorOpen())
                .afterDisp(2, horizontalGrabberRR.floorClose())
                .strafeToLinearHeading(new Vector2d(55, 34), Math.toRadians(90))
                .afterTime(0, horizontalGrabberRR.floorClose())
                .waitSeconds(.4)
                .afterDisp(0, horizontalWristRR.liftButter())
                .afterDisp(5, horizontalWristRR.grabButter())
                .strafeToLinearHeading(new Vector2d(55, 20), Math.toRadians(90))
                .afterDisp(0, horizontalGrabberRR.floorOpen())
                .afterTime(1, horizontalSlideRR.retractSlide());

        TrajectoryActionBuilder hangPost = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(40, 30), Math.toRadians(270))
                .waitSeconds(waitHuman)
                .strafeTo(new Vector2d(40, 18))
                .afterTime(0, verticalGrabberRR.closeGrabber())
                .afterTime(.125, verticalWristRR.takeButter())
                .strafeToLinearHeading(new Vector2d(10*hangSide,5),Math.toRadians(90));

        TrajectoryActionBuilder park = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(50 * parkSide, 10));

        //ask the driver which auto they want to run
//        telemetry.clear();
//        telemetry.addLine("Which side?");
//        telemetry.update();
//        //left side
//        if (gamepad1.square && !gamepad1.circle) {
//            pickLeftSide = true;
//            while (!gamepad1.right_bumper) {
//                telemetry.clear();
//                telemetry.addLine("square-hang: " + pickHang);
//                telemetry.addLine("triangle-Basket" + pickBasket);
//                telemetry.addLine("circle-Park" + pickPark);
//                if (gamepad1.square && !gamepad1.triangle && !gamepad1.circle && !gamepad1.cross) {
//                    pickHang = true;
//                    hangSide = -1;
//                }
//                if (!gamepad1.square && gamepad1.triangle && !gamepad1.circle && !gamepad1.cross) {
//                    pickBasket = true;
//                }
//                if (!gamepad1.square && !gamepad1.triangle && gamepad1.circle && !gamepad1.cross) {
//                    pickPark = true;
//                    parkSide = -1;
//                }
//                telemetry.update();
//            }
//        }
//        //right side
//        if (!gamepad1.square && gamepad1.circle) {
//            pickRightSide = true;
//            while (!gamepad1.right_bumper) {
//                telemetry.clear();
//                telemetry.addLine("square-Hang: " + pickHang);
//                telemetry.addLine("triangle-Human: " + pickHuman);
//                telemetry.addLine("circle-Park" + pickPark);
//                telemetry.addLine("cross-butter" + pickButter);
//                if (gamepad1.square && !gamepad1.triangle && !gamepad1.circle && !gamepad1.cross) {
//                    pickHang = true;
//                    hangSide = 1;
//                }
//                if (!gamepad1.square && gamepad1.triangle && !gamepad1.circle && !gamepad1.cross) {
//                    pickHuman = true;
//                }
//                if (!gamepad1.square && !gamepad1.triangle && gamepad1.circle && !gamepad1.cross) {
//                    pickPark = true;
//                }
//                if (!gamepad1.square && !gamepad1.triangle && !gamepad1.circle && gamepad1.cross) {
//                    pickButter = true;
//                    butterSide = 1;
//                }
//                telemetry.update();
//            }
//        }
        TrajectoryActionBuilder test = drive.actionBuilder(initialPose)
                .waitSeconds(5);




        //initialize the robot
        Actions.runBlocking(
                new SequentialAction(
                        verticalGrabberRR.closeGrabber(),
                        verticalWristRR.takeButter(),
                        horizontalSlideRR.retractSlide(),
                        horizontalWristRR.inRobot(),
                        horizontalGrabberRR.floorClose(),
                        horizontalRollRR.flat()
                )
        );

        //wait for the start button to be press
        waitForStart();
        //if stop button is press, automatically stop
        if (isStopRequested()) return;

        //choose the trajectory
//        //todo figure out how to incorporate this
//        Action trajectoryActionChosen;
//        if (pickLeftSide) {
//            if(pickHang) {
////                trajectoryActionChosen = trajectoryActionChosen+ hang.build();
//            }

            //run the chosen action blocking
        Actions.runBlocking(
                new SequentialAction(
                        verticalSlideRR.liftUp(-400),
                        wait.build()
        ));

    }
}



