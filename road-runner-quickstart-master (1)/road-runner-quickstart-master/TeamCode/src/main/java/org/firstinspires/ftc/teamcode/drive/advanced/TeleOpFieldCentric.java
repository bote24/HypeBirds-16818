package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "advanced")
public class TeleOpFieldCentric extends LinearOpMode {
    private DcMotor Elevador1=null,Elevador2=null;
    private CRServo garra;
    @Override

    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Elevador1  = hardwareMap.get(DcMotor.class, "ElevadorIzq");
        Elevador2  = hardwareMap.get(DcMotor.class, "ElevadorDer");
        garra = hardwareMap.get(CRServo.class, "garra");
        Elevador2.setDirection(DcMotor.Direction.REVERSE);
        Elevador1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevador2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevador1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elevador2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor.setTargetPosition(1440);
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            double garraPoder=0;
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );
            if (gamepad2.a){
                SlidePOS(0,.5);
            }
            if (gamepad2.b){
                SlidePOS(1000,.2);
            }
            if (gamepad2.x){
                SlidePOS(2000,.2);
            }
            if (gamepad2.dpad_up){
                garraPoder=.5;
            }
            if (gamepad2.dpad_down){
                garraPoder=-.5;
            }

            // Update everything. Odometry. Etc.
            drive.update();
            garra.setPower(garraPoder);
            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
    private void SlidePOS(int slidePOS, double motorpower) {
        Elevador1.setTargetPosition(slidePOS);
        Elevador2.setTargetPosition(slidePOS);
        Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Elevador1.setPower(motorpower);
        Elevador2.setPower(motorpower);
        while (Elevador1.isBusy() || Elevador2.isBusy()) {
            telemetry.addData("Starting at",  "%7d :%7d",
                    Elevador1.getCurrentPosition(),
                    Elevador2.getCurrentPosition());

        }
    }
}
