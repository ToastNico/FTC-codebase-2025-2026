package org.firstinspires.ftc.teamcode.AutoEninge;

public enum Team {
    RED(24),
    BLUE(20);

    private final int teamAprilTagID;

    private Team(int teamAprilTagID) {
        this.teamAprilTagID = teamAprilTagID;
    }

    public int getTeamAprilTagID() {
        return this.teamAprilTagID;
    }
}
