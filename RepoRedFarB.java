package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Reposition, Red, Far, B start" )

public class RepoRedFarB extends AutoOp {
    // Start all the way toward the build side on the second tile from
    // the middle on the stone side.
    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        waitForStart();
        fingerUp();
        openClaw();
        forward(0.7); //0.45
        pause(0.5);
        //strafeRight(3.8);  //3.75
        right(1);
        forward(1.8);
        left(1);
        pause(0.5);
        //armUp();
       // pause(0.5);
        forward(0.475);
        pause(0.5);
        fingerDown();
        pause(2);
        backward(2.25);  //2
        //left(2);
        pause(2);
      //  left(1);
      //  pause(1);
       // armDown();
        right(.3);
        pause(1);
        fingerUp();
        strafeLeft(1.35);
        forward(2);
        strafeRight(1);
        backward(2);
        forward(.45);
        left(1.2);
        forward(1.7);
        /*initialize(hardwareMap, telemetry);
        waitForStart();
        fingerUp();
        forward(0.5);
        pause(0.5);
        strafeRight(2);
        pause(0.5);
        armUp();
        pause(0.5);
        forward(0.45);
        pause(0.5);
        fingerDown();
        pause(2);
        backward(2.25);
        pause(2);
        fingerUp();
        pause(1);
        armDown();
        strafeLeft(1.6);
        forward(1.5);
        left(1);
        forward(0.8);*/
    }
}