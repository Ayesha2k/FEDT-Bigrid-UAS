package org.example;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/** Per-aircraft completion tracking. HTTP acceptance and grid entry are not arrival. */
final class DroneControllerSync {
    private final HttpClient http;
    private final String baseUrl;
    private final ObjectMapper json = new ObjectMapper();
    private final Map<Integer, String> tasks = new ConcurrentHashMap<>();
    private final Map<Integer, String> reasons = new ConcurrentHashMap<>();

    DroneControllerSync(HttpClient http, String baseUrl) {
        this.http = http;
        this.baseUrl = baseUrl;
    }

    void accepted(int port, String body) throws IOException {
        JsonNode response = json.readTree(body);
        if (response == null || !response.isObject()) throw new IOException("Invalid controller response: " + body);
        String task = response.path("task_id").asText("");
        if (task.isEmpty()) tasks.remove(port); // Legacy response or an explicit hold.
        else tasks.put(port, task);
        reasons.remove(port);
    }

    boolean ready(int port) throws IOException, InterruptedException {
        JsonNode state = get(port, "/status");
        String phase = state.path("phase").asText(state.path("state").asText("unknown"));
        if (state.has("connected") && !state.path("connected").asBoolean()) {
            return blocked(port, "telemetry unavailable");
        }
        if ("fault".equals(phase) || "offline".equals(phase)) {
            return blocked(port, phase + ": " + state.path("error").asText(""));
        }
        String taskId = tasks.get(port);
        if (taskId != null) {
            JsonNode task = get(port, "/tasks/" + taskId);
            String status = task.path("status").asText("unknown");
            if (!"completed".equals(status)) {
                return blocked(port, "task " + taskId + " " + status + ": " + task.path("error").asText(""));
            }
        }
        if (!"hovering".equals(phase)) return blocked(port, "phase=" + phase);
        reasons.remove(port);
        return true;
    }

    String reason(int port) { return reasons.getOrDefault(port, "waiting for controller"); }

    static boolean hasLiftedOff(boolean commandSent, double startZ, double z) {
        return commandSent && Double.isFinite(startZ) && z > startZ + 0.10;
    }

    private boolean blocked(int port, String reason) {
        reasons.put(port, reason);
        return false;
    }

    private JsonNode get(int port, String path) throws IOException, InterruptedException {
        HttpRequest request = HttpRequest.newBuilder(URI.create(baseUrl + ":" + port + path))
                .timeout(Duration.ofSeconds(2)).GET().build();
        HttpResponse<String> response = http.send(request, HttpResponse.BodyHandlers.ofString());
        if (response.statusCode() != 200) {
            throw new IOException(path + " HTTP " + response.statusCode() + ": " + response.body());
        }
        JsonNode value = json.readTree(response.body());
        if (value == null || !value.isObject()) throw new IOException("Invalid " + path + ": " + response.body());
        return value;
    }
}
