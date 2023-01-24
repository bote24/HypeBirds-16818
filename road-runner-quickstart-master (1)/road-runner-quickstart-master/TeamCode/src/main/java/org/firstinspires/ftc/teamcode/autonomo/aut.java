package org.firstinspires.ftc.teamcode.autonomo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomo.SleeveDetection;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Prueba simple")
public class aut extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor UpLeft = null, UpRight = null, DownLeft = null, DownRight = null;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;

    private String color_vision;
    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);
        UpLeft  = hardwareMap.get(DcMotor.class, "rightFront");
        UpRight = hardwareMap.get(DcMotor.class, "leftFront");
        DownLeft  = hardwareMap.get(DcMotor.class, "rightRear");
        DownRight  = hardwareMap.get(DcMotor.class, "leftRear");
        UpLeft.setDirection(DcMotor.Direction.REVERSE);
        DownLeft.setDirection(DcMotor.Direction.REVERSE);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            SleeveDetection.ParkingPosition hola = sleeveDetection.getPosition();
            color_vision = hola.toString();

            telemetry.update();
        }

        waitForStart();
        if (color_vision == "LEFT"){
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

        }
        else if (color_vision == "RIGHT"){
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
        }
        else{
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