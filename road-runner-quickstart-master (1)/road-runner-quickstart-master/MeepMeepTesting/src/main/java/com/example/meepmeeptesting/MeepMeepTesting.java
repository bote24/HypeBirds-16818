
package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 40, Math.toRadians(180), Math.toRadians(180), 15.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 62.35, Math.toRadians(270)))
                                .forward(61.9)
                                .turn(Math.toRadians(90))
                                .waitSeconds(2)
                                .strafeLeft(12.5)
                                .turn(Math.toRadians(180))
                                .forward(8)
                                .waitSeconds(2)
                                .back(8)
                                .turn(Math.toRadians(180))
                                .strafeRight(12.5)
                                .waitSeconds(2)
                                .strafeLeft(12.5)
                                .turn(Math.toRadians(180))
                                .forward(8)
                                .back(8)
                                .turn(Math.toRadians(180))
                                .strafeLeft(12)
                                .waitSeconds(2)
//                                estacionamiento 1
//                                .strafeLeft(12)
//                                .back(23)
                                //estacionamiento 2
                                //Dejarlo asi
                                //estacionamiento 3
                                .strafeLeft(12)
                                .forward(23)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}