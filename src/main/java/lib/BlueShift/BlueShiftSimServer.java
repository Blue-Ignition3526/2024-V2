package lib.BlueShift;

import java.net.InetSocketAddress;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * The Java class for interacting with the BlueShift simulation bridge
 * <br>
 * <br>
 * Add
 * <br>
 * <br>
 * <code>
 * `implementation 'org.java-websocket:Java-WebSocket:1.5.7'` 
 * </code>
 * <br>
 * <br>
 * to your build.gradle file in the dependencies section to use this class
 */
public class BlueShiftSimServer extends WebSocketServer {
    // Singleton getter
    private static BlueShiftSimServer instance = null;
    public static BlueShiftSimServer getInstance() {
        if (instance == null) instance = new BlueShiftSimServer();
        return instance;
    }

    public BlueShiftSimServer() {
        super(new InetSocketAddress("0.0.0.0", 5809));
    }

    // -------------------- WebSocket Handler Methods --------------------

    @Override
    public void onStart() {
        System.out.println("BlueShift - server running on " + this.getAddress().getHostString() + ":" + this.getAddress().getPort());
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        System.out.println("BlueShift - Error: ");
        ex.printStackTrace();
    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        System.out.println("BlueShift - New connection from " + conn.getRemoteSocketAddress().getAddress().getHostAddress());
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        System.out.println("BlueShift - Closed connection with " + conn.getRemoteSocketAddress().getAddress().getHostAddress());
    }

    @Override
    public void onMessage(WebSocket conn, String message) {
        // Do nothing, BlueShift does not receive messages
    }

    // -------------------- BlueShift Methods --------------------
    private void putOutputSerialized(String key, String serializedOutput) {
        // Sanitize key
        key = key.replace("=", "");

        // Broadcast value
        this.broadcast(key + "=" + serializedOutput);
    } 

    public void putOutput(String key, Pose2d pose) {
        this.putOutputSerialized(key, "Pose2d " + String.valueOf(pose.getX()) + " " + String.valueOf(pose.getY()) + " " + String.valueOf(pose.getRotation().getDegrees()));
    }

    public void putOutput(String key, SwerveModuleState[] states) {
        String serializedOutput = "SwerveModuleState[] ";
        for (var state : states) serializedOutput += String.valueOf(state.speedMetersPerSecond) + " " + String.valueOf(state.angle.getDegrees()) + ",";
        serializedOutput = serializedOutput.substring(0, serializedOutput.length() - 1);
        putOutputSerialized(key, serializedOutput);
    }
}
