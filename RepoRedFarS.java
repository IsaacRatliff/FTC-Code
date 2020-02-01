package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Reposition, Red, Far, S start" )

public class RepoRedFarS extends AutoOp {
    // Start all the way toward the build side on the second tile from
    // the middle on the stone side.
    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        waitForStart();
        fingerUp();
        openClaw();
        forward(0.65);
        //pause(0.5);
        right(1);
        forward(3.2);
        left(1);
        //pause(0.5);
        forward(0.475);
        //pause(0.5);
        fingerDown();
        pause(1);
        backward(2.25);  
        fingerUp();
        strafeLeft(1.2);
        forward(2);
        strafeRight(1);
        backward(2);
        forward(.45);
        left(1.2);
        forward(1.7);
    }
}