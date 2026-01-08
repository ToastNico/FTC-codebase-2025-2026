package org.firstinspires.ftc.teamcode.Crawler.Tasking;

import java.util.*;
import org.firstinspires.ftc.teamcode.Crawler.Navigation.Follower;
import org.firstinspires.ftc.teamcode.Crawler.Localization.Localizer;

public interface Task {
    void start();
    void update();
    boolean isFinished();
}

