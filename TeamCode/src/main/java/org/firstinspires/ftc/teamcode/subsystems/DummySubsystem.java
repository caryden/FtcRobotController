package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

public class DummySubsystem extends SubsystemBase {
    double x = 0;

    public void periodic(){
        x +=2;
        x = x % 2;
    }

    public void doNothing(){
        x +=1;
        x-=1;
    }
}
