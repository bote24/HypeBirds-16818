package org.firstinspires.ftc.teamcode.autonomo;

import static org.firstinspires.ftc.teamcode.drive.advanced.TeleOpFieldCentric.LiftState.LIFT_START;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomo.SleeveDetection;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Prueba simple")
public class aut extends LinearOpMode {
    private DcMotor Elevador1 = null, Elevador2 = null;
    private CRServo garra;

    enum State {

        ELEVATOR_1,
        ELEVATOR_2,
        STAGE_1,
        GARRA_1,
        STAGE_2,
        STAGE_3,
        STAGE_4,
        STAGE_5,
        IDLE
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(-35, 62.35, Math.toRadians(270));

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor UpLeft = null, UpRight = null, DownLeft = null, DownRight = null;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;

    private String color_vision;
    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";

    final int LIFT_ZERO = 0;
    final int LIFT_LOW = 2353;
    final int LIFT_MED = 4000;
    final int LIFT_HIGH = 5600;
    int LIFT_GOING;
    double motorpower = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Elevador1 = hardwareMap.get(DcMotor.class, "ElevadorIzq");
        Elevador2 = hardwareMap.get(DcMotor.class, "ElevadorDer");
        garra = hardwareMap.get(CRServo.class, "garra");
        Elevador2.setDirection(DcMotor.Direction.REVERSE);
        Elevador1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevador2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevador1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elevador2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elevador1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Elevador2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setPoseEstimate(startPose);


        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-12, 0, Math.toRadians(180)))
                //Subir elevador
                .lineToConstantHeading(new Vector2d(-14, 0))
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeRight(12)
                .lineToConstantHeading(new Vector2d(-56, 12))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToSplineHeading(new Pose2d(-34, 12, Math.toRadians(315)))
                .forward(4)
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .lineToSplineHeading(new Pose2d(-56, 12, Math.toRadians(180)))
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .lineToSplineHeading(new Pose2d(-34, 12, Math.toRadians(315)))
                .forward(4)
                .build();


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            SleeveDetection.ParkingPosition hola = sleeveDetection.getPosition();
            color_vision = hola.toString();

            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.ELEVATOR_1;
        //cerrar servo
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentState){
                case ELEVATOR_1:
                    Elevador1.setTargetPosition(LIFT_HIGH);
                    Elevador2.setTargetPosition(LIFT_HIGH);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    LIFT_GOING = LIFT_HIGH;
                    currentState = State.ELEVATOR_2;
                    break;

                case ELEVATOR_2:
                    if(Math.abs(Elevador1.getCurrentPosition() - LIFT_GOING) < 30){
                        currentState = State.GARRA_1;
                        break;

                        case GARRA_1:
                            if (!drive.isBusy()) {
                                currentState = State.STAGE_1;
                                //abrir garra
                            }

                        case STAGE_1:
                            if (!drive.isBusy()) {
                                currentState = State.TRAJECTORY_2;
                                drive.followTrajectoryAsync(trajectory2);
                            }
                            break;






            }

            if (color_vision == "LEFT") {
                UpLeft.setPower(1);
                UpRight.setPower(1);
                DownLeft.setPower(1);
                DownRight.setPower(1);
                sleep(3000);
                UpLeft.setPower(-.4);
                UpRight.setPower(.4);
                DownLeft.setPower(.4);
                DownRight.setPower(-.4);
                sleep(2000);

            } else if (color_vision == "RIGHT") {
                UpLeft.setPower(1);
                UpRight.setPower(1);
                DownLeft.setPower(1);
                DownRight.setPower(1);
                sleep(3000);
                UpLeft.setPower(.4);
                UpRight.setPower(-.4);
                DownLeft.setPower(-.4);
                DownRight.setPower(.4);
                sleep(2000);
            } else {
                UpLeft.setPower(1);
                UpRight.setPower(1);
                DownLeft.setPower(1);
                DownRight.setPower(1);
                sleep(3000);

            }
            UpLeft.setPower(0);
            UpRight.setPower(0);
            DownLeft.setPower(0);
            DownRight.setPower(0);
            sleep(3000);
        }
    }
}