package lib.utils;

public class CanDeviceId {
    private final String busName;
    private final int canId;

    public CanDeviceId(String busName, int canId) {
        this.busName = busName;
        this.canId = canId;
    }

    public String getBusName() {
        return busName;
    }

    public int getCanId() {
        return canId;
    }
}
