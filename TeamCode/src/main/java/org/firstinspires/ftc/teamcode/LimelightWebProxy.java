package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Makes the Limelight reachable from a laptop on the robot's Wi-Fi.
 *
 * The Limelight sits on the Control Hub's private USB network, which the Wi-Fi side cannot
 * reach. The Limelight is 172.29.0.1 on that network; the address the Driver Station shows
 * (e.g. eth0:172.29.0.25) is the Control Hub's own end of the USB link, not the Limelight. While this OpMode runs, the Control Hub relays ports 5800-5809
 * to the Limelight, so http://192.168.43.1:5801 opens the Limelight web UI and
 * http://192.168.43.1:5807/results shows the raw results the FTC SDK reads.
 * Stop the OpMode to stop relaying.
 */
@TeleOp(name = "BIOBUZZ Limelight Web Proxy", group = "BIOBUZZ")
public class LimelightWebProxy extends LinearOpMode {
    // The Limelight 3A's own address on its USB network.
    public static String LIMELIGHT_IP = "172.29.0.1";
    public static int FIRST_PORT = 5800;
    public static int LAST_PORT = 5809;
    // Each connection uses two threads; cap them so a runaway can't crash the Robot Controller app.
    public static int MAX_OPEN_CONNECTIONS = 48;

    private final List<ServerSocket> servers = new ArrayList<>();
    private final List<Socket> sockets = new ArrayList<>();
    private final AtomicInteger connections = new AtomicInteger();
    private final AtomicInteger open = new AtomicInteger();
    private final AtomicInteger rejected = new AtomicInteger();
    private volatile boolean running = true;

    @Override
    public void runOpMode() {
        String info = "(no limelight in config)";
        try {
            info = hardwareMap.get(Limelight3A.class, "limelight").getConnectionInfo();
        } catch (RuntimeException ignored) {
            // only used for telemetry
        }
        String ip = LIMELIGHT_IP;
        if (isOwnAddress(ip)) {
            // Relaying to ourselves loops forever; refuse instead.
            telemetry.addData("ERROR", "%s is this Control Hub, not the Limelight. Fix LIMELIGHT_IP.", ip);
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addData("Limelight", "config %s -> relaying to %s", info, ip);
        telemetry.addLine("Press START, then on the laptop open http://192.168.43.1:5801");
        telemetry.update();
        waitForStart();

        List<String> failed = new ArrayList<>();
        for (int port = FIRST_PORT; port <= LAST_PORT; port++) {
            try {
                ServerSocket server = new ServerSocket();
                server.setReuseAddress(true);
                server.bind(new InetSocketAddress(port));
                server.setSoTimeout(500);
                servers.add(server);
                startAcceptor(server, ip, port);
            } catch (IOException e) {
                failed.add(port + " (" + e.getMessage() + ")");
            }
        }

        try {
            while (opModeIsActive()) {
                telemetry.addData("Relaying", "192.168.43.1:%d-%d -> %s", FIRST_PORT, LAST_PORT, ip);
                telemetry.addData("Web UI", "http://192.168.43.1:5801");
                telemetry.addData("SDK results", "http://192.168.43.1:5807/results");
                telemetry.addData("Connections (open / total / refused)", "%d / %d / %d",
                    open.get(), connections.get(), rejected.get());
                if (!failed.isEmpty()) telemetry.addData("Ports not opened", failed);
                telemetry.update();
                idle();
            }
        } finally {
            running = false;
            closeAll();
        }
    }

    /** True if the address belongs to one of this device's own network interfaces. */
    static boolean isOwnAddress(String ip) {
        try {
            InetAddress address = InetAddress.getByName(ip);
            return address.isLoopbackAddress() || address.isAnyLocalAddress()
                || NetworkInterface.getByInetAddress(address) != null;
        } catch (IOException e) {
            return false;
        }
    }

    private void startAcceptor(ServerSocket server, String ip, int port) {
        Thread t = new Thread(() -> {
            while (running) {
                try {
                    Socket client = server.accept();
                    if (open.get() >= MAX_OPEN_CONNECTIONS) {
                        rejected.incrementAndGet();
                        closeQuietly(client);
                        continue;
                    }
                    Socket upstream = new Socket();
                    try {
                        upstream.connect(new InetSocketAddress(ip, port), 2000);
                    } catch (IOException e) {
                        closeQuietly(client);
                        closeQuietly(upstream);
                        continue;
                    }
                    track(client);
                    track(upstream);
                    connections.incrementAndGet();
                    open.incrementAndGet();
                    AtomicInteger closedSides = new AtomicInteger();
                    pipe(client, upstream, closedSides);
                    pipe(upstream, client, closedSides);
                } catch (SocketTimeoutException e) {
                    // check running again
                } catch (IOException | RuntimeException | OutOfMemoryError e) {
                    if (!running) return;
                }
            }
        }, "ll-proxy-" + port);
        t.setDaemon(true);
        t.start();
    }

    private void pipe(Socket from, Socket to, AtomicInteger closedSides) {
        Thread t = new Thread(() -> {
            byte[] buffer = new byte[16 * 1024];
            try (InputStream in = from.getInputStream(); OutputStream out = to.getOutputStream()) {
                int n;
                while (running && (n = in.read(buffer)) != -1) {
                    out.write(buffer, 0, n);
                    out.flush();
                }
            } catch (IOException ignored) {
                // connection closed
            } finally {
                closeQuietly(from);
                closeQuietly(to);
                untrack(from);
                untrack(to);
                if (closedSides.incrementAndGet() == 2) open.decrementAndGet();
            }
        }, "ll-pipe");
        t.setDaemon(true);
        t.start();
    }

    private synchronized void track(Socket s) {
        sockets.add(s);
    }

    private synchronized void untrack(Socket s) {
        sockets.remove(s);
    }

    private synchronized void closeAll() {
        for (ServerSocket s : servers) {
            try {
                s.close();
            } catch (IOException ignored) {
                // closing anyway
            }
        }
        for (Socket s : sockets) closeQuietly(s);
        servers.clear();
        sockets.clear();
    }

    private static void closeQuietly(Socket s) {
        try {
            s.close();
        } catch (IOException ignored) {
            // closing anyway
        }
    }
}
