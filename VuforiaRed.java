package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Image Detection Red Side" )

public class VuforiaRed extends VuforiaOp{
    public void runOpMode(){
        initialize(hardwareMap, telemetry);
        initVision(hardwareMap);
        activateVision();
        setCameraPosition(0.0, 0, 0);
        waitForStart();
        //armUp(0.05);
        fingerUp();
        double x;
        openClaw();
        forward(0.9);
        x = toSkystone(4, -0.06, 100);
        //strafeLeft(0.3);
        forward(0.6);
        closeClaw();
        backward(0.3);
        strafeRight(-x);
        backward(0.4);
        strafeRight(2.2);
        openClaw();
        //move(0.3, -1);
        //strafeLeft(1.0);
        forward(0.3);
        left(1);
        //forward(0.7);
        tapeOut(1.0);
        
    }

    // todo: write your code here
}
