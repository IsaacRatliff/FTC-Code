package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class BGRepoRedFarS extends AutoOp{
    
    @Override
    public void runOpMode(){
        initialize(hardwareMap, telemetry);
        waitForStart();
        fingerUp();
        openClaw();
        strafeLeft(0.1);
        forward(1.1); 
        pause(1);
        closeClaw();
        pause(1);
        backward(0.5);
        right(1);
        forward(4.2);
        left(1);
        pause(1);
        armUp();
        pause(1);
        forward(0.475);
        openClaw();
        pause(0.5);
        fingerDown();
        pause(2);
        backward(2.25);  
        pause(2);
        right(.3);
        pause(1);
        fingerUp();
        strafeLeft(1.35);
        pause(0.5);
        armUp();
        pause(0.5);
        forward(2);
        strafeRight(1);
        backward(2);
        forward(.45);
        left(1.2);
        forward(1.7);
        
    }
    

    // todo: write your code here
}