package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Reposition, Red, Near, S start" )

public class RepoRedNearS extends AutoOp {
    // Needs to change from far to near.
    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        waitForStart();
        fingerUp();
        openClaw();
        //step one
        forward(.15);
        right(1.1);
        forward(4.35);
        //Step 2
        left(.8);
        pause(0.5);
        forward(1.5);
        pause(0.5);
        fingerDown();
        pause(2);
        //Step 3
        backward(2.4);  //2
        pause(2);
        right(.3);
        pause(1);
        fingerUp();
        //Step 4
        strafeLeft(1.4);
        //Step 5
        forward(2);
        //Step 6
        strafeRight(.75);
        //Step 7
        backward(2);
        
        //Step 8
        forward(.45);
        
        //Step 9
        left(1.2);
        forward(0.85);
        pause(3);
        //step 10
     /*
        left(1);
        forward(1.0);
        */
        strafeLeft(1);
        //step 11
        //right(1);
        forward(.85);
    }
}