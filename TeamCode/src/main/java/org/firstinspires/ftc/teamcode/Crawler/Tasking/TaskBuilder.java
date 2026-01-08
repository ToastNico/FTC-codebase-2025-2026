package org.firstinspires.ftc.teamcode.Crawler.Tasking;

import org.firstinspires.ftc.teamcode.Crawler.Localization.Localizer;
import org.firstinspires.ftc.teamcode.Crawler.Navigation.Follower;

import java.util.ArrayList;
import java.util.List;

public class TaskBuilder {
    private List<Task> list = new ArrayList<>();
    private Follower f;
    private Localizer l;

    public TaskBuilder(Follower f, Localizer l) {
        this.f = f;
        this.l = l;
    }

    // driveTo with default full speed (1.0)
    public TaskBuilder driveTo(double x, double y, double h) {
        return driveTo(x, y, h, 1.0);
    }

    public TaskBuilder driveTo(double x, double y, double h, double speed) {
        list.add(new Task() {
            @Override public void start() {}
            // We pass 'speed' into the follower update
            @Override public void update() { f.update(x, y, h, speed, l); }
            @Override public boolean isFinished() { return f.update(x, y, h, speed, l); }
        });
        return this;
    }

    public TaskBuilder runAction(Runnable r) {
        list.add(new Task() {
            @Override public void start() { r.run(); }
            @Override public void update() {}
            @Override public boolean isFinished() { return true; }
        });
        return this;
    }

    public TaskBuilder waypoint(double x, double y, double h, double radius) {
        return waypoint(x, y, h, radius, 1.0);
    }

    public TaskBuilder waypoint(double x, double y, double h, double radius, double speed) {
        list.add(new Task() {
            @Override public void start() {}
            @Override public void update() {
                f.update(x, y, h, speed, l);
            }
            @Override public boolean isFinished() {
                // Calculation for the Look-Ahead proximity
                double distance = Math.hypot(x - l.x, y - l.y);
                return distance < radius;
            }
        });
        return this;
    }

    public List<Task> build() { return list; }
}