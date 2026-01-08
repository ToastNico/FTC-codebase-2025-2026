package org.firstinspires.ftc.teamcode.Crawler.Tasking;

import java.util.List;

public class StateMachine {
    private List<Task> currentQueue;
    private int index = 0;
    private boolean running = false;
    private boolean taskStarted = false;

    public void start(List<Task> tasks) {
        this.currentQueue = tasks;
        this.index = 0;
        this.running = true;
        this.taskStarted = false;
    }

    public void update() {
        if (!running || currentQueue == null || index >= currentQueue.size()) {
            running = false;
            return;
        }

        Task task = currentQueue.get(index);

        if (!taskStarted) {
            task.start();
            taskStarted = true;
        }

        task.update();

        if (task.isFinished()) {
            index++;
            taskStarted = false;
        }
    }

    public boolean isBusy() {
        return running;
    }

    public void stop() {
        running = false;
        currentQueue = null;
    }
}