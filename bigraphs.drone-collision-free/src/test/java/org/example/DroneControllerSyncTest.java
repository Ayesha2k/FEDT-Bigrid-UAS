package org.example;

import com.sun.net.httpserver.HttpServer;
import org.junit.jupiter.api.Test;

import java.net.InetSocketAddress;
import java.net.http.HttpClient;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicReference;

/** No ROS, CDO or aircraft required. Also runnable directly with java -ea. */
class DroneControllerSyncTest {
    @Test
    void controllerCompletionGatesNavigation() throws Exception {
        AtomicReference<String> state = new AtomicReference<>("{\"phase\":\"taking_off\",\"connected\":true}");
        AtomicReference<String> task = new AtomicReference<>("{\"status\":\"running\"}");
        AtomicReference<Integer> statusCode = new AtomicReference<>(200);
        HttpServer server = HttpServer.create(new InetSocketAddress("127.0.0.1", 0), 0);
        server.createContext("/", exchange -> {
            String body = exchange.getRequestURI().getPath().startsWith("/tasks/") ? task.get() : state.get();
            byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
            exchange.sendResponseHeaders(statusCode.get(), bytes.length);
            try (var out = exchange.getResponseBody()) { out.write(bytes); }
        });
        server.start();
        try {
            int port = server.getAddress().getPort();
            DroneControllerSync sync = new DroneControllerSync(HttpClient.newHttpClient(), "http://127.0.0.1");
            sync.accepted(port, "{\"task_id\":\"takeoff-1\"}");
            check(!sync.ready(port), "Takeoff still running");
            state.set("{\"phase\":\"hovering\",\"connected\":true}");
            check(!sync.ready(port), "Hovering alone cannot complete a running task");
            task.set("{\"status\":\"completed\"}");
            check(sync.ready(port), "Settled takeoff is ready");
            sync.accepted(port, "{\"task_id\":\"nav-1\"}");
            state.set("{\"phase\":\"navigating\",\"connected\":true}");
            task.set("{\"status\":\"running\"}");
            check(!sync.ready(port), "Entering a grid cannot complete navigation");
            task.set("{\"status\":\"failed\",\"error\":\"Goal timeout\"}");
            check(!sync.ready(port), "Failed task must not trigger replanning");
            check(sync.reason(port).contains("Goal timeout"), "Expose task failure reason");
            task.set("{\"status\":\"cancelled\"}");
            check(!sync.ready(port), "Externally cancelled task requires explicit recovery");
            sync.accepted(port, "{\"message\":\"holding\"}");
            state.set("{\"phase\":\"hovering\",\"connected\":false}");
            check(!sync.ready(port), "Stale telemetry blocks movement");
            state.set("{\"phase\":\"fault\",\"error\":\"ROS unavailable\"}");
            check(!sync.ready(port), "Fault blocks movement");
            check(sync.reason(port).contains("ROS unavailable"), "Expose phase error");
            state.set("{\"state\":\"hovering\"}");
            check(sync.ready(port), "Legacy status without task API remains supported");
            statusCode.set(503);
            boolean failed = false;
            try { sync.ready(port); } catch (java.io.IOException e) { failed = e.getMessage().contains("503"); }
            check(failed, "Unavailable HTTP status is never ready");
        } finally { server.stop(0); }
    }

    @Test
    void initialWorldHeightIsNotTakeoff() {
        check(!DroneControllerSync.hasLiftedOff(false, 0.135, 0.135), "No takeoff command");
        check(!DroneControllerSync.hasLiftedOff(true, 0.135, 0.135), "No actual rise");
        check(DroneControllerSync.hasLiftedOff(true, 0.135, 0.30), "Actual rise after command");
        check(DroneControllerSync.hasLiftedOff(true, 0.0, 0.15), "Ground-level real drone");
    }

    private static void check(boolean value, String message) {
        if (!value) throw new AssertionError(message);
    }

    public static void main(String[] args) throws Exception {
        DroneControllerSyncTest test = new DroneControllerSyncTest();
        test.initialWorldHeightIsNotTakeoff();
        test.controllerCompletionGatesNavigation();
        System.out.println("PASS: controller synchronization and takeoff regression cases");
    }
}
