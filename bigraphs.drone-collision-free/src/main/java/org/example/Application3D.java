package org.example;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Topic;
import org.bigraphs.framework.core.Bigraph;
import org.bigraphs.framework.core.BigraphFileModelManagement;
import org.bigraphs.framework.core.Control;
import org.bigraphs.framework.core.exceptions.IncompatibleSignatureException;
import org.bigraphs.framework.core.exceptions.InvalidConnectionException;
import org.bigraphs.framework.core.exceptions.builder.TypeNotExistsException;
import org.bigraphs.framework.core.exceptions.operations.IncompatibleInterfaceException;
import org.bigraphs.framework.core.impl.pure.PureBigraph;
import org.bigraphs.framework.core.impl.pure.PureBigraphBuilder;
import org.bigraphs.framework.core.impl.signature.DynamicSignature;
import org.bigraphs.framework.core.impl.signature.DynamicSignatureBuilder;
import org.bigraphs.framework.core.reactivesystem.BigraphMatch;
import org.bigraphs.framework.core.reactivesystem.InstantiationMap;
import org.bigraphs.framework.core.reactivesystem.ParametricReactionRule;
import org.bigraphs.framework.core.utils.BigraphUtil;
import org.bigraphs.framework.simulation.matching.AbstractBigraphMatcher;
import org.bigraphs.framework.simulation.matching.pure.PureReactiveSystem;
import org.bigraphs.model.bigraphBaseModel.impl.BigraphBaseModelPackageImpl;
import org.bigraphs.spring.data.cdo.CdoTemplate;
import org.eclipse.emf.cdo.common.id.CDOID;
import org.eclipse.emf.cdo.common.model.CDOPackageRegistry;
import org.eclipse.emf.cdo.util.CDOUtil;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EPackage;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.CommandLineRunner;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.context.annotation.Import;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import javax.json.JsonObject;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BinaryOperator;

import static org.bigraphs.framework.core.factory.BigraphFactory.*;

@SpringBootApplication
@Import(value = {CDOServerConfig.class})
public class Application3D implements CommandLineRunner {

    // Resource paths
    private static final String WORLD_RESOURCE_BASE = "src/test/resources/models/3Ddiagonaldirectional/";
    private static final String WORLD_SIGNATURE_MM = WORLD_RESOURCE_BASE + "mm_sig_loc.ecore";
    private static final String WORLD_SIGNATURE_INSTANCE = WORLD_RESOURCE_BASE + "sig_loc.xmi";

    // Timing constants (milliseconds)
    private static final long TAKEOFF_PROTECTION_MS = 5000;
    private static final long TAKEOFF_COOLDOWN_MS = 5000;
    private static final long WAIT_TIMEOUT_MS = 8000;

    private static final long MOVE_TIMEOUT_MS = 10000;
    private static final long MOVE_INTERVAL_MS = 2000;
    private static final long LANDING_DELAY_MS = 2000;

    private static double elapsedMs(long startNano, long endNano) {
        return (endNano - startNano) / 1_000_000.0;
    }

    /** Appends a sample to this drone's queue for the given phase store, creating it on first use. */
    private static void recordLatency(Map<String, Queue<Double>> store, String droneId, double ms) {
        store.computeIfAbsent(droneId, k -> new ConcurrentLinkedQueue<>()).add(ms);
    }

    /** Measure planning time without REST execution or intentional waits. */
    private void logPlanningLatency(String droneId, String action, long startNano, long endNano) {
        double ms = elapsedMs(startNano, endNano);
        recordLatency(planningLatenciesMsByDrone, droneId, ms);
        System.out.println("  ⏱ [Planning] " + droneId + " " + action + ": "
                + String.format(Locale.ROOT, "%.3f", ms) + " ms");
    }

    /** Measure only the Bigraph match used to authorize an action. */
    private void logMatchOnlyLatency(String droneId, String action, long startNano, long endNano) {
        double ms = elapsedMs(startNano, endNano);
        recordLatency(gateMatchLatenciesMsByDrone, droneId, ms);
        System.out.println("  ⏱ [Match] " + droneId + " " + action + ": "
                + String.format(Locale.ROOT, "%.3f", ms) + " ms");
    }

    /** Measure the HTTP request/response interval separately from execution completion. */
    private void logRestLatency(String droneId, String action, long startNano, long endNano) {
        double ms = elapsedMs(startNano, endNano);
        recordLatency(restLatenciesMsByDrone, droneId, ms);
        System.out.println("  ⏱ [REST] " + droneId + " " + action + ": "
                + String.format(Locale.ROOT, "%.3f", ms) + " ms");
    }

    // Altitude thresholds (meters)
    private static final double TAKEOFF_ALTITUDE = 0.1;
    private static final double LANDING_ALTITUDE = 0.1;

    // Battery thresholds (volts)

    // Voltage thresholds are in volts; preserve experimental values.
    private static final double BATTERY_EMPTY_THRESHOLD = 2.1; //3.1！！！！！！！！！
    private static final double BATTERY_LOW_THRESHOLD = 2.2; //3.2！！！！！！！！！

    // RSSI threshold
    private static final int RSSI_BAD_THRESHOLD = 85;//65！！！！！！！！！

    // Synthetic readings are classified using the thresholds above.
    private static final double INJECT_BATTERY_NORMAL_V = 4.0;   // >= BATTERY_LOW_THRESHOLD
    private static final double INJECT_BATTERY_LOW_V    = 3.15;  // [EMPTY, LOW) -> Low
    private static final double INJECT_BATTERY_EMPTY_V  = 3.0;  // < EMPTY     -> Empty
    private static final int    INJECT_RSSI_BAD    = 70;         // >= RSSI_BAD_THRESHOLD -> Bad
    private static final int    INJECT_RSSI_NORMAL = 50;         // <  RSSI_BAD_THRESHOLD -> Normal
    @Autowired
    protected CdoTemplate template;

    @Value("${bigrid.service.base-url:http://172.29.224.1:8080}")
    private String bigridServiceBaseUrl;

    @Value("${bigrid.service.rows:7}")
    private int bigridRows;

    @Value("${bigrid.service.cols:7}")
    private int bigridCols;

    @Value("${bigrid.service.layers:3}")
    private int bigridLayers;

    @Value("${bigrid.service.format:xml}")
    private String bigridFormat;

    @Value("${bigrid.service.origin.x:-1.5}")
    private double gridOriginX;

    @Value("${bigrid.service.origin.y:-1.5}")
    private double gridOriginY;

    @Value("${bigrid.service.origin.z:0.0}")
    private double gridOriginZ;

    @Value("${bigrid.service.step.x:0.5}")
    private double gridStepX;

    @Value("${bigrid.service.step.y:0.5}")
    private double gridStepY;

    @Value("${bigrid.service.layer.height:0.6}")
    private double layerHeight; //0.35 in original

    @Value("${drone.count:12}")
    private int configuredDroneCount;

    @Value("${ros.bridge.host:localhost}")
    private String rosBridgeHost;

    @Value("${ros.update.enabled:true}")
    private boolean rosUpdateEnabled;

    @Value("${ros.sim.mode:true}")
    private boolean rosSimMode;

    // Drone Control Configuration
    @Value("${drone.control.enabled:true}")
    private boolean droneControlEnabled;
    @Value("${drone.control.base-url:http://127.0.0.1}")
    private String droneControlBaseUrl;
    @Value("${drone.control.start-port:5000}")
    private int droneControlStartPort;

    @Value("${drone.targets:-1,-1.5;1,-1.5;1.5,-1;1.5,1;1,1.5;-1,1.5;-1.5,1;-1.5,-1;-1,-1;1,-1;1,1;-1,1}")
    private String droneTargetsConfig;

    @Value("${charging.station.grids:3}")
    private String chargingStationGrids;

    @Value("${obstacle.grids:18,67}")
    private String obstacleGridsConfig;

    @Value("${baseline.mode:false}")
    private boolean baselineMode;

    @Value("${fault.injection.enabled:true}")
    private boolean faultInjectionEnabled;
    @Value("${fault.injection.port:8090}")
    private int faultInjectionPort;

    @Value("${collective.ascent.enabled:true}")
    private boolean collectiveAscentEnabled;

    @Value("${collective.ascent.altitude:0.9}")
    private double collectiveAscentAltitude;

    @Value("${collective.ascent.takeoff-wait-timeout-ms:8000}")
    private long collectiveAscentTakeoffWaitMs;

    @Value("${collective.ascent.settle-ms:4000}")
    private long collectiveAscentSettleMs;

    @Value("${navigation.absolute-altitude.enabled:true}")
    private boolean absoluteAltitudeEnabled;

    @Value("${navigation.base-altitude:0.3}")
    private double navigationBaseAltitude;

    @Value("${takeoff.allow-out-of-grid:true}")
    private boolean allowOutOfGridPlacement;

    @Value("${collision.violation.debounce-frames:3}")
    private int violationDebounceFrames;

    private final HttpClient httpClient = HttpClient.newBuilder()
            .connectTimeout(Duration.ofSeconds(5))
            .build();

    private final Map<String, GridPoint> droneTargets = new HashMap<>();

    private List<GridPoint> chargingStations = new ArrayList<>();

    private final Set<Integer> obstacleGrids = ConcurrentHashMap.newKeySet();

    private final ObjectMapper objectMapper = new ObjectMapper();
    private DroneControllerSync controllerSync;
    private final Map<String, String> controllerWaitReasons = new ConcurrentHashMap<>();

    private final Map<String, DronePosition> dronePositions = new ConcurrentHashMap<>();

    private final Map<String, DroneStatus> droneStatuses = new ConcurrentHashMap<>();

    private final AtomicBoolean collisionRiskDetected = new AtomicBoolean(false);

    private final Map<Integer, GridReservation> gridReservations = new ConcurrentHashMap<>();

    private final Object cdoLock = new Object();

    private DynamicSignature combinedSignature;
    private DynamicSignature serviceWorldSignature;  

    private CDOID cdoIdWorld;
    private CDOID cdoIdDrone;
    private CDOID cdoIdComposed;

    private PureBigraph worldPart;
    private PureBigraph dronePart;
    private PureBigraph composite;

    private final AtomicBoolean firstUpdateCompleted = new AtomicBoolean(false);

    // Mission metrics (Baseline + Bigraph runs use the same ROS pose span for completion time)
    private final AtomicLong coOccupancyViolationCount = new AtomicLong(0);

    private final Map<Integer, Integer> coOccupancyStreak = new ConcurrentHashMap<>();

    private final Set<Integer> countedViolationGrids = ConcurrentHashMap.newKeySet();

    // Per-decision, per-drone latency samples (ms), disjoint phases: Planning (A* + diagonal check) ->
    // Match (FEDT runtime gate admissibility check only) -> REST (dispatch to drone control API).
    // Keyed by droneId so cf231/D0's samples are never pooled with cf232/D1's, etc.
    // Populated inside logPlanningLatency / logMatchOnlyLatency / logRestLatency respectively.
    private final Map<String, Queue<Double>> planningLatenciesMsByDrone = new ConcurrentHashMap<>();
    private final Map<String, Queue<Double>> gateMatchLatenciesMsByDrone = new ConcurrentHashMap<>();
    private final Map<String, Queue<Double>> restLatenciesMsByDrone = new ConcurrentHashMap<>();
    private volatile Long firstRosPoseTimeMs = null;
    private volatile Long lastRosPoseTimeMs = null;
    private volatile Long allDronesLandedAtMs = null;
    // AtomicBoolean (not plain volatile) so the normal completion path and the JVM shutdown-hook
    // path (which can race on Ctrl+C) can never both write the report.
    private final AtomicBoolean missionReportPrinted = new AtomicBoolean(false);

    public static void main(String[] args) {
        BigraphBaseModelPackageImpl.init();
        SpringApplication.run(Application3D.class, args);
    }

    @Override
    public void run(String... args) throws Exception {
        controllerSync = new DroneControllerSync(httpClient, droneControlBaseUrl);
        // Ctrl+C (or any JVM shutdown) before all drones land would otherwise silently discard
        // every collected metric (completion time, violations, per-decision latencies), since
        // they only ever lived in memory and were written out by maybePrintMissionReport().
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            if (!missionReportPrinted.get()) {
                System.out.println("\n⚠ JVM shutting down before mission completion — writing partial mission report...");
                writeMissionReport("interrupted");
            }
        }, "MissionReportShutdownHook"));

        TimeUnit.MILLISECONDS.sleep(2500);

        if (baselineMode) {
            System.out.println("\n========================================");
            System.out.println("  BASELINE MODE (A* without Bigraph step check)");
            System.out.println("========================================\n");
        }

        prepareDatabase();

        fetchWorldSignatureFromService();

        System.out.println("Building merged signature");
        sig();

        worldPart = fetchWorldModelFromService();

        registerMetaModelToCDO();

        System.out.println("\n========================================");
        System.out.println("Creating Drone Model and Composite Model");
        System.out.println("========================================");

        int currentSiteCount = worldPart.getSites().size();
        dronePart = droneModel(currentSiteCount);
        composite = composeWorldAndDrones(worldPart, dronePart);

        EPackage MM = createOrGetBigraphMetaModel(sig());
        worldPart = BigraphUtil.toBigraph(MM, template.insert(worldPart.getInstanceModel(), "/world"), sig());
        dronePart = BigraphUtil.toBigraph(MM, template.insert(dronePart.getInstanceModel(), "/drone"), sig());
        composite = BigraphUtil.toBigraph(MM, template.insert(composite.getInstanceModel(), "/composed"), sig());

        cdoIdWorld = CDOUtil.getCDOObject(worldPart.getInstanceModel()).cdoID();
        cdoIdDrone = CDOUtil.getCDOObject(dronePart.getInstanceModel()).cdoID();
        cdoIdComposed = CDOUtil.getCDOObject(composite.getInstanceModel()).cdoID();

        System.out.println("✓ World Model inserted into CDO (CDOID: " + cdoIdWorld + ")");
        System.out.println("✓ Drone Model inserted into CDO (CDOID: " + cdoIdDrone + ")");
        System.out.println("✓ Composite Model inserted into CDO (CDOID: " + cdoIdComposed + ")");

        initializeRosSubscriptions();

        startFaultInjectionServer();

        if (!baselineMode) {

            Thread modelUpdateThread = new Thread(this::continuousModelUpdate, "ModelUpdateThread");
            modelUpdateThread.setDaemon(false);
            modelUpdateThread.start();

            System.out.println("Waiting for first model update to complete...");
            while (!firstUpdateCompleted.get()) {
                TimeUnit.MILLISECONDS.sleep(100);
            }
            System.out.println("✓ First model update completed\n");
        } else {
            firstUpdateCompleted.set(true);
            System.out.println("✓ Baseline mode: skipping continuous Bigraph model update thread\n");
        }

        if (rosUpdateEnabled) {
            Thread statusMonitorThread = new Thread(this::continuousStatusMonitoring, "StatusMonitorThread");
            statusMonitorThread.setDaemon(false);
            statusMonitorThread.start();
            System.out.println("✓ Battery and communication status monitoring thread started");
        }

        if (rosUpdateEnabled) {
            System.out.println("Waiting for ROS2 status data to stabilize (3 seconds)...");
            TimeUnit.MILLISECONDS.sleep(3000);
            System.out.println("✓ ROS2 status data should be stable now\n");
        }

        parseDroneTargets();

        if (droneControlEnabled) {
            performTakeoffSequence();

            performCollectiveAscent();
        }

        System.out.println("\n========================================");
        System.out.println("Entering navigation control loop...");
        System.out.println("========================================\n");

        while (true) {
            TimeUnit.MILLISECONDS.sleep(1000);  

            if (!droneControlEnabled || !rosUpdateEnabled) {
                continue;
            }

            checkAndApplyTakeoffRules();

            if (baselineMode) {
                performBaselineNavigationControl();
            } else {
                performNavigationControl();
            }

            checkAndApplyLandingRules();

            maybePrintMissionReport();
        }
    }

    private void continuousModelUpdate() {
        try {
            int currentSiteCount = worldPart.getSites().size();
            int updateCount = 0;

            while (true) {
                updateCount++;

                PureBigraph latestWorld = fetchWorldModelFromService();
                int newSiteCount = latestWorld.getSites().size();

                boolean needUpdateDrone = (newSiteCount != currentSiteCount) ||
                        (rosUpdateEnabled && !dronePositions.isEmpty() && !collisionRiskDetected.get());

                synchronized (cdoLock) {
                    if (needUpdateDrone) {
                        if (newSiteCount != currentSiteCount) {
                            System.out.println("  [Update" + updateCount + "] Site count changed: " + currentSiteCount + " -> " + newSiteCount);
                            currentSiteCount = newSiteCount;
                        }

                        if (rosUpdateEnabled && !dronePositions.isEmpty() && !collisionRiskDetected.get()) {
                            dronePart = droneModelFromRosPositions(newSiteCount);
                        } else {
                            dronePart = droneModel(newSiteCount);
                        }

                        EObject insertedDrone = template.insert(dronePart.getInstanceModel(), "/drone");
                        dronePart = BigraphUtil.toBigraph(createOrGetBigraphMetaModel(sig()), insertedDrone, sig());
                        cdoIdDrone = CDOUtil.getCDOObject(dronePart.getInstanceModel()).cdoID();
                    }

                    EObject insertedWorld = template.insert(latestWorld.getInstanceModel(), "/world");
                    worldPart = BigraphUtil.toBigraph(createOrGetBigraphMetaModel(sig()), insertedWorld, sig());
                    cdoIdWorld = CDOUtil.getCDOObject(worldPart.getInstanceModel()).cdoID();

                    PureBigraph updatedComposite = composeWorldAndDrones(worldPart, dronePart);
                    EObject insertedComposite = template.insert(updatedComposite.getInstanceModel(), "/composed");
                    composite = BigraphUtil.toBigraph(createOrGetBigraphMetaModel(sig()), insertedComposite, sig());
                    cdoIdComposed = CDOUtil.getCDOObject(composite.getInstanceModel()).cdoID();
                }

                if (!firstUpdateCompleted.get()) {
                    firstUpdateCompleted.set(true);
                }

                TimeUnit.MILLISECONDS.sleep(500);
            }
        } catch (Exception e) {
            System.err.println("!! Model update thread exception: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private void prepareDatabase() throws Exception {
        System.out.println("Preparing CDO database...");

        String[] paths = {"/drone", "/world", "/composed"};
        for (String path : paths) {
            try {
                template.removeAll(path);
                System.out.println("  ✓ Cleared path: " + path);
            } catch (org.eclipse.emf.cdo.view.CDOViewSet.CDOViewSetException e) {

                System.out.println("  ⚠ Skipped dirty resource: " + path + " (will be overwritten)");
            } catch (Exception e) {

                System.out.println("  ! Error cleaning " + path + " (ignorable): " + e.getClass().getSimpleName());
            }
        }

        System.out.println("✓ CDO database ready");
    }

    private void registerMetaModelToCDO() throws Exception {

        DynamicSignature signature = sig();
        EPackage metaModel = createOrGetBigraphMetaModel(signature); 

        EPackage.Registry.INSTANCE.put(metaModel.getNsURI(), metaModel); 
        CDOPackageRegistry.INSTANCE.put(metaModel.getNsURI(), metaModel); 
        template.getCDOPackageRegistry().put(metaModel.getNsURI(), metaModel); 
    }

    private DynamicSignature sig() {
        if (combinedSignature == null) {
            combinedSignature = buildCombinedSignature();
        }
        return combinedSignature;
    }

    private DynamicSignature buildCombinedSignature() {
        try {
            System.out.println("Building merged signature (World + Drone)...");

            if (serviceWorldSignature == null) {
                throw new IllegalStateException("World signature not loaded yet!");
            }

            DynamicSignature droneSignature = createDroneSignature();
            DynamicSignature merged = BigraphUtil.mergeSignatures(serviceWorldSignature, droneSignature);

            System.out.println("✓ Merged signature built successfully");
            System.out.println("  World signature controls: " + serviceWorldSignature.getControls().size());
            System.out.println("  Drone signature controls: " + droneSignature.getControls().size());
            System.out.println("  Merged total controls: " + merged.getControls().size());

            return merged;
        } catch (Exception e) {
            System.err.println("Error building merged signature: " + e.getMessage());
            e.printStackTrace();
            throw new IllegalStateException("Unable to build combined signature", e);
        }
    }

    private DynamicSignature createDroneSignature() {
        DynamicSignatureBuilder builder = pureSignatureBuilder();
        builder.add("OccupiedBy", 0)
                .add("Bad", 0)
                .add("Drone", 1)
                .add("ID", 0)
                .add("Status", 0)
                .add("Battery", 0)
                .add("Normal", 0)
                .add("Communication", 0)
                .add("Low", 0)
                .add("Landed", 0)
                .add("flying", 0)
                .add("Empty", 0)
                .add("ObstacleOrWeather", 0);
        for (int i = 0; i <= 10; i++) {
            builder.add("D" + i, 0);
        }
        return builder.create();
    }

    private DynamicSignature fetchWorldSignatureFromService() throws Exception {
        if (serviceWorldSignature != null) {
            return serviceWorldSignature;
        }

        System.out.println("\n==================Local World Signature======================");

        List<EObject> sigObjects = BigraphFileModelManagement.Load.signatureInstanceModel(
                WORLD_SIGNATURE_MM, WORLD_SIGNATURE_INSTANCE);
        serviceWorldSignature = (DynamicSignature) createOrGetSignature(sigObjects.get(0));

        System.out.println("  Control count: " + serviceWorldSignature.getControls().size());
        System.out.println("  Control list: ");
        for (Control<?, ?> control : serviceWorldSignature.getControls()) {
            System.out.println("    - " + control.getNamedType().stringValue() +
                    " (arity: " + control.getArity().getValue() + ")");
        }
        System.out.println("========================================\n");

        return serviceWorldSignature;
    }

    private PureBigraph fetchWorldModelFromService() throws Exception {

        DynamicSignature combinedSig = sig();
        EPackage metaModel = createOrGetBigraphMetaModel(combinedSig);

        System.out.println("\nFetching 3D BiGrid instance model...");
        URI uri = URI.create(String.format(Locale.ROOT,
                "%s/generate/3d-diagonal-directional/bigrid?rows=%d&cols=%d&layers=%d&format=%s",
                bigridServiceBaseUrl, bigridRows, bigridCols, bigridLayers, bigridFormat));

        HttpRequest request = HttpRequest.newBuilder(uri)
                .timeout(Duration.ofSeconds(10))
                .header("Content-Type", "application/json")
                .POST(HttpRequest.BodyPublishers.ofString(buildGridRequestPayload(), StandardCharsets.UTF_8))
                .build();

        HttpResponse<String> response = httpClient.send(request, HttpResponse.BodyHandlers.ofString(StandardCharsets.UTF_8));

        if (response.statusCode() != 200) {
            throw new IllegalStateException("Failed to fetch BiGrid model, status=" + response.statusCode());
        }

        String xmlContent = extractXmlFromJsonResponse(response.body());

        try (ByteArrayInputStream inputStream = new ByteArrayInputStream(xmlContent.getBytes(StandardCharsets.UTF_8))) {

            List<EObject> worldObjects = BigraphFileModelManagement.Load.bigraphInstanceModel(metaModel, inputStream);

            PureBigraph bigraph = BigraphUtil.toBigraph(metaModel, worldObjects.get(0), combinedSig);

            //System.out.println("========================================\n");

            return bigraph;
        } catch (Exception e) {
            System.err.println("!!! Error loading World Model !!!");
            System.err.println("Error type: " + e.getClass().getName());
            System.err.println("Error message: " + e.getMessage());
            e.printStackTrace();
            throw e;
        }
    }

    private String extractXmlFromJsonResponse(String responseBody) throws Exception {
        String trimmedBody = responseBody.trim();

        try {

            if (trimmedBody.startsWith("{")) {
                JsonNode jsonNode = objectMapper.readTree(trimmedBody);

                if (jsonNode.has("content")) {
                    String content = jsonNode.get("content").asText();

                    String mimeType = jsonNode.has("mimeType") ? jsonNode.get("mimeType").asText() : "";

                    if (mimeType.contains("xml")) {

                        return content;
                    }

                    if (content.trim().startsWith("<?xml") || content.trim().startsWith("<")) {

                        return content;
                    }
                }

                throw new IllegalStateException("JSON response does not contain valid XML content");
            }

            if (trimmedBody.startsWith("<?xml") || trimmedBody.startsWith("<")) {
                System.out.println("Response appears to be pure XML");
                return trimmedBody;
            }

            throw new IllegalStateException("Response is neither valid JSON nor XML. First 100 characters: " +
                    trimmedBody.substring(0, Math.min(100, trimmedBody.length())));

        } catch (Exception e) {
            System.err.println("Error parsing response: " + e.getMessage());
            System.err.println("Response preview (first 500 chars): " +
                    trimmedBody.substring(0, Math.min(500, trimmedBody.length())));
            throw e;
        }
    }

    private String buildGridRequestPayload() {
        return String.format(Locale.ROOT,
                "{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"stepSizeX\":%.3f,\"stepSizeY\":%.3f,\"layerHeight\":%.3f}",
                gridOriginX, gridOriginY, gridOriginZ, gridStepX, gridStepY, layerHeight);
    }

    private PureBigraph droneModel(int siteCount) throws InvalidConnectionException, TypeNotExistsException {
        if (siteCount <= 0) {
            return pureBuilder(sig()).create();
        }

        List<Bigraph<DynamicSignature>> placements = new ArrayList<>();
        for (int i = 0; i < siteCount; i++) {

            if (obstacleGrids.contains(i)) {
                placements.add(buildObstacleCell());
            } else {
                placements.add(emptyOccupiedCell());
            }
        }

        int dronesToPlace = Math.min(configuredDroneCount, siteCount);
        for (int idx = 0; idx < dronesToPlace; idx++) {

            if (obstacleGrids.contains(idx)) {
                continue;
            }

            String droneId = "D" + idx;

            String droneStatus = "Landed";  
            DroneStatus status = droneStatuses.get(droneId);
            if (status != null) {
                if (status.landingRuleApplied) {
                    droneStatus = "Landed";  
                } else if (status.takeoffRuleApplied) {
                    droneStatus = "flying";  
                }
            }
            placements.set(idx, buildDrone(droneId, droneStatus, "OccupiedBy"));
        }

        Bigraph<DynamicSignature> result = placements.stream()
                .reduce(pureLinkings(sig()).identity_e(), accumulator::apply); 
        return (PureBigraph) result;
    }

    private PureBigraph buildDrone(String id, String status, String nodeType) throws InvalidConnectionException, TypeNotExistsException {
        PureBigraphBuilder<DynamicSignature> builder = pureBuilder(sig());
        String normalizedId = id.toLowerCase(Locale.ROOT);

        DroneStatus droneStatus = droneStatuses.get(id);
        String batteryLevel = (droneStatus != null) ? droneStatus.batteryLevel : "Normal";
        String communicationStatus = (droneStatus != null) ? droneStatus.communicationStatus : "Normal";

        return builder.root()
                .child(nodeType).down()
                .child("Drone", normalizedId).down()
                .child("ID").down().child(id).up()
                .child("Status").down().child(status).up()
                .child("Battery").down().child(batteryLevel).up()
                .child("Communication").down().child(communicationStatus).up()
                .create();
    }

    private PureBigraph emptyOccupiedCell() throws InvalidConnectionException, TypeNotExistsException {
        PureBigraphBuilder<DynamicSignature> builder = pureBuilder(sig());
        return builder.root()
                .child("OccupiedBy")
                .create();
    }

    private PureBigraph buildObstacleCell() throws InvalidConnectionException, TypeNotExistsException {
        PureBigraphBuilder<DynamicSignature> builder = pureBuilder(sig());
        return builder.root()
                .child("OccupiedBy").down()
                .child("ObstacleOrWeather")
                .create();
    }

    private PureBigraph composeWorldAndDrones(PureBigraph world, PureBigraph drones) throws IncompatibleSignatureException, IncompatibleInterfaceException {
        return ops(world).nesting(drones).getOuterBigraph();
    }

    private final BinaryOperator<Bigraph<DynamicSignature>> accumulator = (partial, element) -> {
        try {
            return ops(partial).parallelProduct(element).getOuterBigraph();
        } catch (IncompatibleSignatureException | IncompatibleInterfaceException e) {
            return pureLinkings(partial.getSignature()).identity_e();
        }
    };

    private ParametricReactionRule<PureBigraph> droneTakeOffRule(String id) throws Exception {
        PureBigraphBuilder<DynamicSignature> redexB = pureBuilder(sig());
        PureBigraphBuilder<DynamicSignature> reactumB = pureBuilder(sig());
        String normalizedId = id.toLowerCase(Locale.ROOT);
        redexB.root()
                .child("OccupiedBy").down()
                .site()
                .child("Drone", normalizedId).down()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("Landed").up()
                .child("Battery").down()
                .child("Normal").up()
                .child("Communication").down()
                .child("Normal").up()
        ;

        reactumB.root()
                .child("OccupiedBy").down()
                .site()
                .child("Drone", normalizedId).down()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("flying").up()
                .child("Battery").down()
                .child("Normal").up()
                .child("Communication").down()
                .child("Normal").up()
        ;
        InstantiationMap instantiationMap = InstantiationMap.create(1);
        instantiationMap.map(0, 0);
        return new ParametricReactionRule<>(redexB.create(), reactumB.create(), instantiationMap);
    }

    private ParametricReactionRule<PureBigraph> droneLandingRule(String id) throws Exception {
        PureBigraphBuilder<DynamicSignature> redexB = pureBuilder(sig());
        PureBigraphBuilder<DynamicSignature> reactumB = pureBuilder(sig());
        String normalizedId = id.toLowerCase(Locale.ROOT);
        redexB.root()
                .child("OccupiedBy").down()
                .site()
                .child("Drone", normalizedId).down()
                .site()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("flying");

        reactumB.root()
                .child("OccupiedBy").down()
                .site()
                .child("Drone", normalizedId).down()
                .site()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("Landed");
        InstantiationMap instantiationMap = InstantiationMap.create(2);
        instantiationMap.map(0, 0);
        instantiationMap.map(1, 1);
        return new ParametricReactionRule<>(redexB.create(), reactumB.create(), instantiationMap);
    }

    private ParametricReactionRule<PureBigraph> createBatteryRule(String id, String from, String to) throws Exception {
        PureBigraphBuilder<DynamicSignature> redexB = pureBuilder(sig());
        PureBigraphBuilder<DynamicSignature> reactumB = pureBuilder(sig());
        String normalizedId = id.toLowerCase(Locale.ROOT);

        redexB.root().child("OccupiedBy").down().site()
                .child("Drone", normalizedId).down().site()
                .child("ID").down().child(id).up()
                .child("Battery").down().child(from);
        reactumB.root().child("OccupiedBy").down().site()
                .child("Drone", normalizedId).down().site()
                .child("ID").down().child(id).up()
                .child("Battery").down().child(to);

        InstantiationMap map = InstantiationMap.create(2);
        map.map(0, 0);
        map.map(1, 1);
        return new ParametricReactionRule<>(redexB.create(), reactumB.create(), map);
    }

    private ParametricReactionRule<PureBigraph> droneBatteryNormalToLowRule(String id) throws Exception {
        return createBatteryRule(id, "Normal", "Low");
    }

    private ParametricReactionRule<PureBigraph> droneBatteryLowToNormalRule(String id) throws Exception {
        return createBatteryRule(id, "Low", "Normal");
    }

    private ParametricReactionRule<PureBigraph> droneBatteryEmptyToLowRule(String id) throws Exception {
        return createBatteryRule(id, "Empty", "Low");
    }

    private ParametricReactionRule<PureBigraph> droneBatteryLowToEmptyRule(String id) throws Exception {
        return createBatteryRule(id, "Low", "Empty");
    }

    private ParametricReactionRule<PureBigraph> createCommunicationRule(String id, String from, String to) throws Exception {
        PureBigraphBuilder<DynamicSignature> redexB = pureBuilder(sig());
        PureBigraphBuilder<DynamicSignature> reactumB = pureBuilder(sig());
        String normalizedId = id.toLowerCase(Locale.ROOT);

        redexB.root().child("OccupiedBy").down().site()
                .child("Drone", normalizedId).down().site()
                .child("ID").down().child(id).up()
                .child("Communication").down().child(from);
        reactumB.root().child("OccupiedBy").down().site()
                .child("Drone", normalizedId).down().site()
                .child("ID").down().child(id).up()
                .child("Communication").down().child(to);

        InstantiationMap map = InstantiationMap.create(2);
        map.map(0, 0);
        map.map(1, 1);
        return new ParametricReactionRule<>(redexB.create(), reactumB.create(), map);
    }

    private ParametricReactionRule<PureBigraph> droneCommunicationNormalToBadRule(String id) throws Exception {
        return createCommunicationRule(id, "Normal", "Bad");
    }

    private ParametricReactionRule<PureBigraph> droneCommunicationBadToNormalRule(String id) throws Exception {
        return createCommunicationRule(id, "Bad", "Normal");
    }

    /** Preserve unmatched sites while moving one drone between adjacent locales. */
    private ParametricReactionRule<PureBigraph> createDirectionalMoveRule(String id, String routeType) throws Exception {
        PureBigraphBuilder<DynamicSignature> redexB = pureBuilder(sig());
        PureBigraphBuilder<DynamicSignature> reactumB = pureBuilder(sig());
        String normalizedId = id.toLowerCase(Locale.ROOT);

        redexB.root()
                .child("Locale", "occupied1").down()
                .site()
                .child(routeType, "occupied2")
                .child("OccupiedBy").down()
                .child("Drone", normalizedId).down()
                .site()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("flying")
                .top()
                .child("Locale", "occupied2").down()
                .site()
                .child("OccupiedBy");

        reactumB.root()
                .child("Locale", "occupied1").down()
                .site()
                .child(routeType, "occupied2")
                .child("OccupiedBy")
                .top()
                .child("Locale", "occupied2").down()
                .site()
                .child("OccupiedBy").down()
                .child("Drone", normalizedId).down()
                .site()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("flying");

        InstantiationMap instantiationMap = InstantiationMap.create(3);
        instantiationMap.map(0, 0);
        instantiationMap.map(1, 1);
        instantiationMap.map(2, 2);
        return new ParametricReactionRule<>(redexB.create(), reactumB.create(), instantiationMap);
    }

    private boolean sendDroneControlRequest(String droneId, int port, String endpoint, String successMessage, String errorPrefix) {
        try {
            String url = droneControlBaseUrl + ":" + port + endpoint;
            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .timeout(Duration.ofSeconds(5))
                    .POST(HttpRequest.BodyPublishers.noBody())
                    .build();

            long restStart = System.nanoTime();
            HttpResponse<String> response = httpClient.send(request, HttpResponse.BodyHandlers.ofString());
            long restEnd = System.nanoTime();

            if (response.statusCode() == 200) {
                controllerSync.accepted(port, response.body());
                System.out.println("  ✓ " + droneId + " " + successMessage);
                logRestLatency(droneId, endpoint, restStart, restEnd);
                return true;
            } else {
                System.err.println("  !! " + droneId + " " + errorPrefix + " failed: HTTP " + response.statusCode() + ": " + response.body());
                return false;
            }
        } catch (Exception e) {
            System.err.println("  !! " + droneId + " " + errorPrefix + " failed: " + e.getMessage());
            return false;
        }
    }

    private boolean activateIdle(String droneId, int port) {
        return sendDroneControlRequest(droneId, port, "/activate_idle", "Idle state has been activated.", "activate idle");
    }

    private boolean beginTakeoff(String droneId, int port) {
        DroneStatus status = droneStatuses.get(droneId);
        if (status != null) status.takeoffStartZ = status.z;
        return sendDroneControlRequest(droneId, port, "/begin_takeoff", "Takeoff command has been transmitted.", "takeoff");
    }

    /** Polling is per drone; a busy drone never prevents others from advancing. */
    private boolean controllerReady(String droneId) {
        int port = droneControlStartPort + Integer.parseInt(droneId.substring(1));
        String reason;
        try {
            if (controllerSync.ready(port)) {
                controllerWaitReasons.remove(droneId);
                return true;
            }
            reason = controllerSync.reason(port);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return false;
        } catch (Exception e) {
            reason = e.getMessage();
        }
        String previous = controllerWaitReasons.put(droneId, reason);
        if (!Objects.equals(previous, reason)) {
            System.out.println("[Controller Wait] " + droneId + ": " + reason);
        }
        return false;
    }

    private boolean navigateTo(String droneId, int port, double x, double y, double z) {
        if (!controllerReady(droneId)) return false;
        try {
            String url = droneControlBaseUrl + ":" + port + "/navigate/" + x + "/" + y + "/" + z;
            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .timeout(Duration.ofSeconds(5))
                    .POST(HttpRequest.BodyPublishers.noBody())
                    .build();

            long restStart = System.nanoTime();
            HttpResponse<String> response = httpClient.send(request, HttpResponse.BodyHandlers.ofString());
            long restEnd = System.nanoTime();

            if (response.statusCode() == 200) {
                controllerSync.accepted(port, response.body());
                System.out.println("  ✓ " + droneId + " navigation command sent -> (" +
                        String.format("%.2f, %.2f, %.2f", x, y, z) + ")");
                logRestLatency(droneId, "navigate", restStart, restEnd);
                return true;
            } else {
                System.err.println("  !! " + droneId + " navigation failed: HTTP " + response.statusCode() + ": " + response.body());
                return false;
            }
        } catch (Exception e) {
            System.err.println("  !! " + droneId + " navigation failed: " + e.getMessage());
            return false;
        }
    }

    private boolean beginLanding(String droneId, int port) {
        return sendDroneControlRequest(droneId, port, "/begin_landing", "Landing command has been transmitted.", "landing");
    }

    private void performTakeoffSequence() throws Exception {
        if (!droneControlEnabled) {
            System.out.println("\n========================================");
            System.out.println("Drone control function not enabled");
            System.out.println("To enable, set: drone.control.enabled=true");
            System.out.println("========================================\n");
            return;
        }

        System.out.println("\n========================================");
        System.out.println(baselineMode
                ? "Starting takeoff sequence: direct REST control (no Bigraph match)"
                : "Starting takeoff sequence: rule matching and takeoff control");
        System.out.println("========================================");

        if (!baselineMode) {

            System.out.println("\n[Pre-Takeoff Check] Applying battery and communication rules...");
            for (int i = 0; i < configuredDroneCount; i++) {
                String droneId = "D" + i;
                DroneStatus status = droneStatuses.get(droneId);
                if (status != null) {
                    try {
                        System.out.println("  " + droneId + " battery voltage: " +
                                String.format("%.2f", status.batteryVoltage) + "V, level: " + status.batteryLevel);
                        checkAndApplyBatteryRule(droneId, status);
                        checkAndApplyCommunicationRule(droneId, status);
                    } catch (Exception e) {
                        System.err.println("  !! Error applying rules for " + droneId + ": " + e.getMessage());
                    }
                }
            }
            System.out.println("✓ Pre-takeoff status check completed\n");
        }

        AbstractBigraphMatcher<PureBigraph> matcher = baselineMode ? null
                : AbstractBigraphMatcher.create(PureBigraph.class);

        for (int i = 0; i < configuredDroneCount; i++) {
            String droneId = "D" + i;
            int cfNumber = 231 + i;
            int port = droneControlStartPort + i;

            try {
                System.out.println("\nProcessing " + droneId + " (cf" + cfNumber + ", port " + port + "):");

                DroneStatus status = droneStatuses.get(droneId);
                if (status != null) {
                    System.out.println("  Current status - Battery: " + status.batteryLevel +
                            " (" + String.format("%.2f", status.batteryVoltage) + "V), Comm: " + status.communicationStatus);
                }

                ParametricReactionRule<PureBigraph> takeoffRule = baselineMode ? null : droneTakeOffRule(droneId);

                boolean takeoffMatched = baselineMode;
                if (!baselineMode) {
                    long takeoffMatchStart = System.nanoTime();
                    Iterator<BigraphMatch<PureBigraph>> matchIterator = matcher.match(composite, takeoffRule).iterator();
                    takeoffMatched = matchIterator.hasNext();
                    long takeoffMatchEnd = System.nanoTime();
                    if (takeoffMatched) {
                        logMatchOnlyLatency(droneId, "takeoff authorization", takeoffMatchStart, takeoffMatchEnd);
                    }
                }

                if (takeoffMatched) {
                    if (!baselineMode) {
                        System.out.println("  ✓ Rule matched successfully - " + droneId + " can take off");
                    } else {
                        System.out.println("  ✓ Baseline takeoff - " + droneId + " (no Bigraph check)");
                    }

                    System.out.println("  → Sending takeoff commands...");

                    if (!activateIdle(droneId, port)) {
                        System.err.println("  !! Unable to activate idle state, skipping " + droneId);
                        continue;
                    }

                    TimeUnit.MILLISECONDS.sleep(500);  

                    if (!beginTakeoff(droneId, port)) {
                        System.err.println("  !! Unable to send takeoff command, skipping " + droneId);
                        continue;
                    }

                    if (status != null) {
                        status.takeoffTime = System.currentTimeMillis();
                        status.takeoffCommandSent = true;  
                    }

                    System.out.println("  ✓ " + droneId + " takeoff sequence initiated");
                } else {

                    String reason = "";
                    if (status != null) {
                        if (!"Normal".equals(status.batteryLevel)) {
                            reason = "Battery is " + status.batteryLevel + " (requires Normal)";
                        } else if (!"Normal".equals(status.communicationStatus)) {
                            reason = "Communication is " + status.communicationStatus + " (requires Normal)";
                        } else if (status.hasTakenOff || status.takeoffRuleApplied) {
                            reason = "already in flying status";
                        } else {
                            reason = "unknown (check Bigraph model)";
                        }
                    }
                    System.out.println("  ✗ Rule not matched - " + droneId + " cannot take off: " + reason);
                }
            } catch (Exception e) {
                System.err.println("  !! Error processing " + droneId + ": " + e.getMessage());
                e.printStackTrace();
            }
        }

        System.out.println("\n========================================");
        System.out.println("Takeoff sequence completed");
        System.out.println("========================================\n");
    }

    /** Wait for controller-confirmed takeoff before requesting a shared operating height. */
    private void performCollectiveAscent() {
        if (!collectiveAscentEnabled) {
            return;
        }
        if (!droneControlEnabled) {
            System.out.println("[Collective Ascent] Skipped: drone control not enabled");
            return;
        }
        if (!rosUpdateEnabled) {
            System.out.println("[Collective Ascent] Skipped: ROS updates disabled (no altitude feedback)");
            return;
        }

        System.out.println("\n========================================");
        System.out.println("[Collective Ascent] Raising all UAS to operating altitude "
                + String.format(Locale.ROOT, "%.2f", collectiveAscentAltitude) + "m");
        System.out.println("========================================");

        long waitStart = System.currentTimeMillis();
        while (true) {
            int taken = 0, active = 0;
            for (int i = 0; i < configuredDroneCount; i++) {
                if (obstacleGrids.contains(i)) {
                    continue; 
                }
                active++;
                DroneStatus st = droneStatuses.get("D" + i);
                if (st != null && st.hasTakenOff && controllerReady("D" + i)) {
                    taken++;
                }
            }
            if (active > 0 && taken >= active) {
                System.out.println("  ✓ All " + active + " UAS airborne, beginning collective ascent");
                break;
            }
            if (System.currentTimeMillis() - waitStart > collectiveAscentTakeoffWaitMs) {
                System.out.println("  ⚠ Timeout waiting for takeoff (" + taken + "/" + active
                        + " airborne); proceeding with those airborne");
                break;
            }
            try {
                TimeUnit.MILLISECONDS.sleep(200);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }

        Set<String> ascending = new HashSet<>();
        for (int i = 0; i < configuredDroneCount; i++) {
            String droneId = "D" + i;
            DroneStatus st = droneStatuses.get(droneId);
            if (st == null || !st.hasTakenOff || !controllerReady(droneId)) {
                continue;
            }
            DronePosition pos = dronePositions.get(droneId);
            if (pos == null) {
                System.out.println("  ⚠ " + droneId + " has no position yet, skipping ascent");
                continue;
            }
            int port = droneControlStartPort + i;
            System.out.println("  ↑ " + droneId + " ascending from "
                    + String.format(Locale.ROOT, "%.2f", st.z) + "m to "
                    + String.format(Locale.ROOT, "%.2f", collectiveAscentAltitude) + "m");
            if (navigateTo(droneId, port, pos.x, pos.y, collectiveAscentAltitude)) {
                ascending.add(droneId);
            }
        }

        // Simulator time can run slower than wall time; never infer completion from a fixed sleep.
        long deadline = System.currentTimeMillis() + Math.max(collectiveAscentTakeoffWaitMs, collectiveAscentSettleMs);
        while (!ascending.isEmpty() && System.currentTimeMillis() < deadline) {
            ascending.removeIf(id -> controllerReady(id)
                    && Math.abs(droneStatuses.get(id).z - collectiveAscentAltitude) <= 0.10);
            try {
                TimeUnit.MILLISECONDS.sleep(200);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
        if (ascending.isEmpty()) {
            System.out.println("[Collective Ascent] Controller-confirmed ascent complete.");
        } else {
            System.err.println("[Collective Ascent] Still pending: " + ascending
                    + "; navigation will wait for each controller independently.");
        }
        System.out.println("========================================\n");
    }

    /** Movement directions: 8 horizontal + 2 vertical */
    private enum MoveDirection {
        FORWARD, BACK, LEFT, RIGHT, FORWARD_LEFT, FORWARD_RIGHT, BACK_LEFT, BACK_RIGHT, UP, DOWN
    }

    private MoveDirection getMoveDirection(GridPoint current, GridPoint next) {
        double dx = next.x - current.x, dy = next.y - current.y, dz = next.z - current.z;
        boolean zChg = Math.abs(dz) > 0.01, xChg = Math.abs(dx) > 0.01, yChg = Math.abs(dy) > 0.01;

        if (zChg && !xChg && !yChg) return dz > 0 ? MoveDirection.UP : MoveDirection.DOWN;
        if (xChg && yChg) {
            if (dx > 0 && dy > 0) return MoveDirection.FORWARD_LEFT;
            if (dx > 0 && dy < 0) return MoveDirection.FORWARD_RIGHT;
            if (dx < 0 && dy > 0) return MoveDirection.BACK_LEFT;
            return MoveDirection.BACK_RIGHT;
        }
        if (xChg) return dx > 0 ? MoveDirection.FORWARD : MoveDirection.BACK;
        if (yChg) return dy > 0 ? MoveDirection.LEFT : MoveDirection.RIGHT;
        return MoveDirection.FORWARD;
    }

    private ParametricReactionRule<PureBigraph> getMoveRuleForDirection(String droneId, MoveDirection dir) throws Exception {
        String routeType = switch (dir) {
            case FORWARD -> "ForwardRoute";
            case BACK -> "BackRoute";
            case LEFT -> "LeftRoute";
            case RIGHT -> "RightRoute";
            case FORWARD_LEFT -> "ForwardLeftRoute";
            case FORWARD_RIGHT -> "ForwardRightRoute";
            case BACK_LEFT -> "BackLeftRoute";
            case BACK_RIGHT -> "BackRightRoute";
            case UP -> "UpRoute";
            case DOWN -> "DownRoute";
        };
        return createDirectionalMoveRule(droneId, routeType);
    }

    /** Absolute mode targets a layer height; relative mode offsets the measured altitude. */
    private double computeTargetZ(MoveDirection direction, DroneStatus status, GridPoint nextPoint) {
        boolean vertical = (direction == MoveDirection.UP || direction == MoveDirection.DOWN);

        if (absoluteAltitudeEnabled) {
            int totalPerLayer = bigridCols * bigridRows;
            int targetLayer = (totalPerLayer > 0) ? nextPoint.gridIndex / totalPerLayer : 0;
            double targetZ = navigationBaseAltitude + targetLayer * layerHeight;
            System.out.println("  → " + (vertical ? "Vertical " + direction : "Horizontal")
                    + " (absolute): layer " + targetLayer + " -> "
                    + String.format(Locale.ROOT, "%.2f", targetZ) + "m (from "
                    + String.format(Locale.ROOT, "%.2f", status.z) + "m)");
            return targetZ;
        }

        double targetZ;
        if (direction == MoveDirection.UP) {
            targetZ = status.z + layerHeight;
            System.out.println("  → Vertical UP: " + String.format("%.2f", status.z) + "m -> " + String.format("%.2f", targetZ) + "m");
        } else if (direction == MoveDirection.DOWN) {
            targetZ = status.z - layerHeight;
            System.out.println("  → Vertical DOWN: " + String.format("%.2f", status.z) + "m -> " + String.format("%.2f", targetZ) + "m");
        } else {
            targetZ = status.z;
            System.out.println("  → Horizontal move: keeping altitude " + String.format("%.2f", targetZ) + "m");
        }
        return targetZ;
    }

    /** Reserve/check the orthogonal sweep cells as well as the diagonal destination. */
    private boolean isDiagonalPathClear(String droneId, GridPoint current, GridPoint next, MoveDirection direction) {
        if (!(direction == MoveDirection.FORWARD_LEFT ||
                direction == MoveDirection.FORWARD_RIGHT ||
                direction == MoveDirection.BACK_LEFT ||
                direction == MoveDirection.BACK_RIGHT)) {
            return true;
        }

        int totalPerLayer = bigridCols * bigridRows;
        int layerIndex = current.gridIndex / totalPerLayer;
        int indexInLayer = current.gridIndex % totalPerLayer;
        int xIndex = indexInLayer / bigridRows;
        int yIndex = indexInLayer % bigridRows;

        List<Integer> cellsToCheck = new ArrayList<>(2);

        switch (direction) {
            case FORWARD_LEFT -> {
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex + 1, yIndex);     // FORWARD
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex,     yIndex + 1); // LEFT
            }
            case FORWARD_RIGHT -> {
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex + 1, yIndex);     // FORWARD
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex,     yIndex - 1); // RIGHT
            }
            case BACK_LEFT -> {
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex - 1, yIndex);     // BACK
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex,     yIndex + 1); // LEFT
            }
            case BACK_RIGHT -> {
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex - 1, yIndex);     // BACK
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex,     yIndex - 1); // RIGHT
            }
            default -> {
                return true;
            }
        }

        for (int idx : cellsToCheck) {

            if (idx == next.gridIndex) {
                continue;
            }

            for (Map.Entry<String, DronePosition> entry : dronePositions.entrySet()) {
                String otherId = entry.getKey();
                if (!otherId.equals(droneId)) {
                    DronePosition pos = entry.getValue();
                    if (pos.gridIndex == idx) {
                        DroneStatus otherStatus = droneStatuses.get(otherId);
                        GridPoint otherTarget = null;
                        if (otherStatus != null) {
                            if (otherStatus.isInEmergency && otherStatus.emergencyTarget != null) {
                                otherTarget = otherStatus.emergencyTarget;
                            } else {
                                otherTarget = droneTargets.get(otherId);
                            }
                        }

                        if (otherTarget != null && otherTarget.gridIndex == current.gridIndex) {
                            continue;
                        }

                        DroneStatus myStatus = droneStatuses.get(droneId);
                        if (otherStatus != null && otherStatus.isWaiting
                                && myStatus != null && myStatus.isWaiting) {
                            int myIdx = Integer.parseInt(droneId.substring(1));
                            int otherIdx = Integer.parseInt(otherId.substring(1));
                            if (myIdx < otherIdx) {
                                System.out.println("  [Deadlock Break] " + droneId +
                                        " priority over waiting " + otherId + " at Grid[" + idx + "]");
                                continue;
                            }
                        }

                        if (myStatus != null && myStatus.consecutiveWaitCycles >= 5) {
                            System.out.println("  [Forced Pass] " + droneId +
                                    " forced through Grid[" + idx + "] after " +
                                    myStatus.consecutiveWaitCycles + " wait cycles");
                            myStatus.consecutiveWaitCycles = 0;
                            continue;
                        }

                        System.out.println("  ⚠ " + droneId + " diagonal " + direction.name() +
                                " path blocked by another drone at Grid[" + idx + "], waiting/detour...");
                        return false;
                    }
                }
            }
        }

        return true;
    }

    private void addIfValidDiagonalCheckCell(List<Integer> cellsToCheck, int layerIndex, int xIndex, int yIndex) {
        if (xIndex >= 0 && xIndex < bigridCols && yIndex >= 0 && yIndex < bigridRows &&
                layerIndex >= 0 && layerIndex < bigridLayers) {
            int totalPerLayer = bigridCols * bigridRows;
            int idx = layerIndex * totalPerLayer + xIndex * bigridRows + yIndex;
            cellsToCheck.add(idx);
        }
    }

    /** Navigation control: dynamic A* + grid reservation + wait/detour */
    /** Plan the next move, apply the Bigraph gate, and retain reservations until arrival. */
    private void performNavigationControl() {
        if (!droneControlEnabled) {
            return;
        }

        long currentTime = System.currentTimeMillis();
        AbstractBigraphMatcher<PureBigraph> matcher = AbstractBigraphMatcher.create(PureBigraph.class);
        final long WAIT_TIMEOUT = 8000;  

        for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
            String droneId = entry.getKey();
            DroneStatus status = entry.getValue();

            if (!status.hasTakenOff || !status.takeoffRuleApplied) {
                continue;
            }
            if (!controllerReady(droneId)) continue;

            if (status.reachedDestination) {
                releaseAllGrids(droneId);
                continue;
            }

            try {

                boolean needsEmergency = ("Low".equals(status.batteryLevel) || "Empty".equals(status.batteryLevel)
                        || "Bad".equals(status.communicationStatus)) && !status.isInEmergency;
                if (needsEmergency) {

                    continue;
                }

                GridPoint target;
                if (status.isInEmergency && status.emergencyTarget != null) {
                    target = status.emergencyTarget;
                } else {
                    target = droneTargets.get(droneId);
                }

                if (target == null) {
                    System.err.println("  !! " + droneId + " no target configured");
                    continue;
                }

                DronePosition currentPos = dronePositions.get(droneId);
                if (currentPos == null || currentPos.gridIndex < 0) {
                    continue;
                }

                if (status.isMoving && status.movingToGrid != null) {

                    if (currentPos.gridIndex == status.movingToGrid) {

                        System.out.println("  ✓ " + droneId + " arrived at Grid[" + status.movingToGrid + "]");

                        releaseAllGrids(droneId);

                        tryReserveGrid(droneId, currentPos.gridIndex);
                        status.reservedGrid = currentPos.gridIndex;

                        status.isMoving = false;
                        status.movingToGrid = null;
                        status.avoidGrids.clear();
                        status.consecutiveWaitCycles = 0;
                    } else {

                        if (currentTime - status.lastMoveTime > MOVE_TIMEOUT_MS) {
                            System.out.println("  ⏱ " + droneId + " move timeout: expected Grid[" + status.movingToGrid + "], current Grid[" + currentPos.gridIndex + "], re-planning from current position");
                            releaseGrid(droneId, status.movingToGrid);
                            status.reservedGrid = null;
                            status.isMoving = false;
                            status.movingToGrid = null;

                        } else {

                            continue;
                        }
                    }
                }

                if (currentPos.gridIndex == target.gridIndex) {

                    if (status.isInEmergency) {
                        if ("EMPTY_BATTERY".equals(status.emergencyType) || "LOW_BATTERY_BAD_COMM".equals(status.emergencyType)) {
                            System.out.println("\n🚨 [EMERGENCY LANDING] " + droneId + " reached emergency landing point Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.emergencyLanded = true;

                        } else if ("LOW_BATTERY".equals(status.emergencyType)) {
                            System.out.println("\n🔋 [RECHARGE POINT] " + droneId + " reached recharge point Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.waitingForRecharge = true;

                        } else if ("BAD_COMM".equals(status.emergencyType)) {
                            System.out.println("\n📡 [HOME REACHED] " + droneId + " reached home Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.waitingForRecharge = true;

                        }
                    } else {
                        System.out.println("\n🎯 [Target Reached] " + droneId + " reached target Grid[" + target.gridIndex + "]");
                        status.reachedDestination = true;
                    }

                    status.isMoving = false;
                    status.movingToGrid = null;
                    releaseAllGrids(droneId);
                    continue;
                }

                if (currentTime - status.lastMoveTime < 2000) { 
                    continue;
                }

                GridPoint currentGridPoint = gridIndexToPoint(currentPos.gridIndex);

                long planStart = System.nanoTime();

                System.out.println("  [Path Planning] " + droneId + " current: Grid[" + currentPos.gridIndex +
                        "](" + String.format("%.1f,%.1f,%.1f", currentGridPoint.x, currentGridPoint.y, currentGridPoint.z) +
                        "), target: Grid[" + target.gridIndex + "](" + String.format("%.1f,%.1f", target.x, target.y) + ")");

                GridPoint nextPoint = planNextStep(droneId, currentGridPoint, target);

                if (nextPoint == null) {
                    logPlanningLatency(droneId, "navigation", planStart, System.nanoTime());
                    System.err.println("  !! " + droneId + " unable to plan path to target");
                    status.isWaiting = true;
                    continue;
                }

                int currentLayer = currentPos.gridIndex / (bigridCols * bigridRows);
                int nextLayer = nextPoint.gridIndex / (bigridCols * bigridRows);
                String layerInfo = (currentLayer != nextLayer) ?
                        " [Layer " + currentLayer + " → Layer " + nextLayer + "]" :
                        " [Layer " + currentLayer + "]";
                System.out.println("  [Next Step] Grid[" + nextPoint.gridIndex +
                        "](" + String.format("%.1f,%.1f,%.1f", nextPoint.x, nextPoint.y, nextPoint.z) + ")" + layerInfo);

                MoveDirection direction = getMoveDirection(currentGridPoint, nextPoint);
                String directionName = direction.name();
                String navAction = directionName + " move";

                if (!isDiagonalPathClear(droneId, currentGridPoint, nextPoint, direction)) {
                    logPlanningLatency(droneId, navAction, planStart, System.nanoTime());
                    handleWaitingAndDetour(droneId, status, currentTime, currentGridPoint, target, nextPoint);
                    continue;
                }

                long planEnd = System.nanoTime();
                logPlanningLatency(droneId, navAction, planStart, planEnd);

                long matchStart = System.nanoTime();
                ParametricReactionRule<PureBigraph> moveRule = getMoveRuleForDirection(droneId, direction);
                Iterator<BigraphMatch<PureBigraph>> matchIterator = matcher.match(composite, moveRule).iterator();
                boolean ruleMatched = matchIterator.hasNext();
                long matchEnd = System.nanoTime();
                logMatchOnlyLatency(droneId, navAction, matchStart, matchEnd);

                if (!ruleMatched) {
                    System.out.println("  ⚠ " + droneId + " " + directionName + " rule not matched");
                    handleWaitingAndDetour(droneId, status, currentTime, currentGridPoint, target, nextPoint);
                    continue;
                }

                System.out.println("  ✓ " + droneId + " " + directionName + " rule matched");

                boolean reservationSuccess = tryReserveGrid(droneId, nextPoint.gridIndex);
                if (!reservationSuccess) {
                    System.out.println("  ⚠ " + droneId + " Grid[" + nextPoint.gridIndex + "] reserved by " + getGridReservation(nextPoint.gridIndex));
                    handleWaitingAndDetour(droneId, status, currentTime, currentGridPoint, target, nextPoint);
                    continue;
                }

                if (status.isWaiting) {
                    long waitTime = currentTime - status.waitingStartTime;
                    System.out.println("  ✓ [Wait End] " + droneId + " successfully reserved Grid[" + nextPoint.gridIndex +
                            "] (waited " + (waitTime / 1000) + "s)");
                    status.isWaiting = false;
                    status.waitingForGrid = null;
                    status.avoidGrids.clear();
                    status.consecutiveWaitCycles = 0;
                }
                status.reservedGrid = nextPoint.gridIndex;

                System.out.println(droneId + " locked Grid[" + nextPoint.gridIndex + "], preparing to move");

                int droneIndex = Integer.parseInt(droneId.substring(1));
                int port = droneControlStartPort + droneIndex;

                System.out.println("[Navigation] " + droneId + " from Grid[" + currentPos.gridIndex +
                        "](" + String.format("%.1f,%.1f,%.1f", currentGridPoint.x, currentGridPoint.y, currentGridPoint.z) + ") " +
                        directionName + " moving to Grid[" + nextPoint.gridIndex +
                        "](" + String.format("%.1f,%.1f,%.1f", nextPoint.x, nextPoint.y, nextPoint.z) + ")");

                double targetZ = computeTargetZ(direction, status, nextPoint);

                if (navigateTo(droneId, port, nextPoint.x, nextPoint.y, targetZ)) {
                    status.lastMoveTime = System.currentTimeMillis();

                    status.isMoving = true;
                    status.movingToGrid = nextPoint.gridIndex;

                    if (currentPos.gridIndex != nextPoint.gridIndex) {
                        releaseGrid(droneId, currentPos.gridIndex);
                    }
                } else {
                    System.err.println(" !! Navigation command failed");

                    releaseGrid(droneId, nextPoint.gridIndex);
                    status.reservedGrid = null;
                    status.isMoving = false;
                    status.movingToGrid = null;
                }

            } catch (ClassCastException e) {

                System.err.println(" !! " + droneId + " nav: Bigraph model port structure incompatible (array vs list), skipping this cycle");
            } catch (Exception e) {
                System.err.println(" !! " + droneId + " nav error: " + e.getMessage());
            }
        }
    }

    /** Baseline navigation: A* + diagonal check + direct REST (no Bigraph match / grid reservation / wait) */
    /** Execute the baseline path without the Bigraph gate or reservation policy. */
    private void performBaselineNavigationControl() {
        if (!droneControlEnabled) {
            return;
        }

        long currentTime = System.currentTimeMillis();

        for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
            String droneId = entry.getKey();
            DroneStatus status = entry.getValue();

            if (!status.hasTakenOff || !status.takeoffRuleApplied) {
                continue;
            }
            if (!controllerReady(droneId)) continue;

            if (status.reachedDestination) {
                continue;
            }

            try {
                boolean needsEmergency = ("Low".equals(status.batteryLevel) || "Empty".equals(status.batteryLevel)
                        || "Bad".equals(status.communicationStatus)) && !status.isInEmergency;
                if (needsEmergency) {
                    continue;
                }

                GridPoint target;
                if (status.isInEmergency && status.emergencyTarget != null) {
                    target = status.emergencyTarget;
                } else {
                    target = droneTargets.get(droneId);
                }

                if (target == null) {
                    System.err.println("  !! " + droneId + " no target configured");
                    continue;
                }

                DronePosition currentPos = dronePositions.get(droneId);
                if (currentPos == null || currentPos.gridIndex < 0) {
                    continue;
                }

                if (status.isMoving && status.movingToGrid != null) {
                    if (currentPos.gridIndex == status.movingToGrid) {
                        System.out.println("  ✓ " + droneId + " arrived at Grid[" + status.movingToGrid + "]");
                        status.isMoving = false;
                        status.movingToGrid = null;
                        status.avoidGrids.clear();
                        status.consecutiveWaitCycles = 0;
                    } else if (currentTime - status.lastMoveTime > MOVE_TIMEOUT_MS) {
                        System.out.println("  ⏱ " + droneId + " move timeout: expected Grid[" + status.movingToGrid
                                + "], current Grid[" + currentPos.gridIndex + "], re-planning from current position");
                        status.isMoving = false;
                        status.movingToGrid = null;
                    } else {
                        continue;
                    }
                }

                if (currentPos.gridIndex == target.gridIndex) {
                    if (status.isInEmergency) {
                        if ("EMPTY_BATTERY".equals(status.emergencyType) || "LOW_BATTERY_BAD_COMM".equals(status.emergencyType)) {
                            System.out.println("\n🚨 [EMERGENCY LANDING] " + droneId + " reached emergency landing point Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.emergencyLanded = true;
                        } else if ("LOW_BATTERY".equals(status.emergencyType)) {
                            System.out.println("\n🔋 [RECHARGE POINT] " + droneId + " reached recharge point Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.waitingForRecharge = true;
                        } else if ("BAD_COMM".equals(status.emergencyType)) {
                            System.out.println("\n📡 [HOME REACHED] " + droneId + " reached home Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.waitingForRecharge = true;
                        }
                    } else {
                        System.out.println("\n🎯 [Target Reached] " + droneId + " reached target Grid[" + target.gridIndex + "]");
                        status.reachedDestination = true;
                    }
                    status.isMoving = false;
                    status.movingToGrid = null;
                    continue;
                }

                if (currentTime - status.lastMoveTime < MOVE_INTERVAL_MS) {
                    continue;
                }

                GridPoint currentGridPoint = gridIndexToPoint(currentPos.gridIndex);
                long planStart = System.nanoTime();

                System.out.println("  [Path Planning] " + droneId + " current: Grid[" + currentPos.gridIndex +
                        "](" + String.format("%.1f,%.1f,%.1f", currentGridPoint.x, currentGridPoint.y, currentGridPoint.z) +
                        "), target: Grid[" + target.gridIndex + "](" + String.format("%.1f,%.1f", target.x, target.y) + ")");

                GridPoint nextPoint = planNextStep(droneId, currentGridPoint, target);

                if (nextPoint == null) {
                    logPlanningLatency(droneId, "navigation", planStart, System.nanoTime());
                    System.err.println("  !! " + droneId + " unable to plan path to target");
                    continue;
                }

                int currentLayer = currentPos.gridIndex / (bigridCols * bigridRows);
                int nextLayer = nextPoint.gridIndex / (bigridCols * bigridRows);
                String layerInfo = (currentLayer != nextLayer) ?
                        " [Layer " + currentLayer + " → Layer " + nextLayer + "]" :
                        " [Layer " + currentLayer + "]";
                System.out.println("  [Next Step] Grid[" + nextPoint.gridIndex +
                        "](" + String.format("%.1f,%.1f,%.1f", nextPoint.x, nextPoint.y, nextPoint.z) + ")" + layerInfo);

                MoveDirection direction = getMoveDirection(currentGridPoint, nextPoint);
                String directionName = direction.name();
                String navAction = directionName + " move";

                logPlanningLatency(droneId, navAction, planStart, System.nanoTime());
                System.out.println("  → [Baseline] " + droneId + " executing " + directionName + " without Bigraph check");

                int droneIndex = Integer.parseInt(droneId.substring(1));
                int port = droneControlStartPort + droneIndex;

                System.out.println("[Navigation] " + droneId + " from Grid[" + currentPos.gridIndex +
                        "](" + String.format("%.1f,%.1f,%.1f", currentGridPoint.x, currentGridPoint.y, currentGridPoint.z) + ") " +
                        directionName + " moving to Grid[" + nextPoint.gridIndex +
                        "](" + String.format("%.1f,%.1f,%.1f", nextPoint.x, nextPoint.y, nextPoint.z) + ")");

                double targetZ = computeTargetZ(direction, status, nextPoint);

                if (navigateTo(droneId, port, nextPoint.x, nextPoint.y, targetZ)) {
                    status.lastMoveTime = System.currentTimeMillis();
                    status.isMoving = true;
                    status.movingToGrid = nextPoint.gridIndex;
                } else {
                    System.err.println(" !! Navigation command failed");
                    status.isMoving = false;
                    status.movingToGrid = null;
                }

            } catch (Exception e) {
                System.err.println(" !! " + droneId + " nav error: " + e.getMessage());
            }
        }
    }

    private void handleWaitingAndDetour(String droneId, DroneStatus status, long currentTime,
                                        GridPoint currentPoint, GridPoint target, GridPoint blockedPoint) {
        if (!status.isWaiting) {
            status.isWaiting = true;
            status.waitingStartTime = currentTime;
            status.waitingForGrid = blockedPoint.gridIndex;
        }
        status.consecutiveWaitCycles++;

        if (currentTime - status.waitingStartTime > WAIT_TIMEOUT_MS) {
            Set<Integer> obstacles = getOccupiedGrids(droneId);
            obstacles.add(blockedPoint.gridIndex);
            List<GridPoint> detour = planPath(currentPoint, target, obstacles);

            if (detour.size() > 1 && detour.get(1).gridIndex != blockedPoint.gridIndex) {
                System.out.println("  [Detour] " + droneId + " \u2192 Grid[" + detour.get(1).gridIndex + "]");
                status.isWaiting = false;
                status.waitingForGrid = null;
                status.consecutiveWaitCycles = 0;

                status.avoidGrids.add(blockedPoint.gridIndex);
                status.avoidGridsSetTime = currentTime;
            } else {

                status.avoidGrids.add(blockedPoint.gridIndex);
                status.avoidGridsSetTime = currentTime;
                status.waitingStartTime = currentTime;
            }
        }
    }

    /** Persist the rewritten composite before subsequent decisions consume it. */
    private boolean applyRuleAndPersist(String droneId, ParametricReactionRule<PureBigraph> rule, String logPrefix) {
        return applyRuleAndPersist(droneId, rule, logPrefix, logPrefix);
    }

    private boolean applyRuleAndPersist(String droneId, ParametricReactionRule<PureBigraph> rule, String logPrefix, String matchAction) {
        synchronized (cdoLock) {
            try {
                PureBigraph currentComposite = composite;
                AbstractBigraphMatcher<PureBigraph> matcher = AbstractBigraphMatcher.create(PureBigraph.class);
                long matchStart = System.nanoTime();
                Iterator<BigraphMatch<PureBigraph>> matchIterator = matcher.match(currentComposite, rule).iterator();

                if (matchIterator.hasNext()) {
                    long matchEnd = System.nanoTime();
                    logMatchOnlyLatency(droneId, matchAction, matchStart, matchEnd);
                    BigraphMatch<PureBigraph> match = matchIterator.next();
                    PureBigraph resultComposite = new PureReactiveSystem().buildParametricReaction(
                            currentComposite, match, rule);

                    EObject insertedComposite = template.insert(resultComposite.getInstanceModel(), "/composed");
                    composite = BigraphUtil.toBigraph(createOrGetBigraphMetaModel(sig()), insertedComposite, sig());
                    cdoIdComposed = CDOUtil.getCDOObject(composite.getInstanceModel()).cdoID();

                    System.out.println("  ✓ Rule applied, " + droneId + " " + logPrefix);
                    System.out.println("  ✓ Updated model persisted (CDOID: " + cdoIdComposed + ")");
                    return true;
                } else {
                    System.out.println("  ○ Rule not matched, " + droneId + " may already be in target status");
                    return false;
                }
            } catch (ClassCastException e) {

                System.err.println("  !! Bigraph model port structure incompatible (array vs list), skipping rule: " + droneId + " " + logPrefix);
                return false;
            } catch (Exception e) {
                System.err.println("  !! CDO operation failed: " + e.getMessage());
                e.printStackTrace();
                return false;
            }
        }
    }

    /** Apply the model transition only after telemetry reports an actual ascent. */
    private boolean checkAndApplyTakeoffRules() {
        boolean anyRuleApplied = false;

        for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
            String droneId = entry.getKey();
            DroneStatus status = entry.getValue();

            if (status.hasTakenOff && !status.takeoffRuleApplied) {
                if (baselineMode) {
                    System.out.println("\n [Baseline] " + droneId + " took off (altitude: " +
                            String.format("%.3f", status.z) + "m), marking flying (no Bigraph rule apply)");
                    status.status = "flying";
                    status.takeoffRuleApplied = true;
                    anyRuleApplied = true;
                    continue;
                }
                try {
                    System.out.println("\n [Rule Apply] " + droneId + " took off (altitude: " +
                            String.format("%.3f", status.z) + "m), applying takeoff rule...");

                    ParametricReactionRule<PureBigraph> takeoffRule = droneTakeOffRule(droneId);
                    if (applyRuleAndPersist(droneId, takeoffRule, "status updated to flying", "apply takeoff rule")) {
                        status.status = "flying"; 
                        status.takeoffRuleApplied = true;
                        anyRuleApplied = true;
                    } else {

                        String reason = "";
                        if (!"Normal".equals(status.batteryLevel)) {
                            reason = "Battery is " + status.batteryLevel + " (rule requires Normal)";
                        } else if (!"Normal".equals(status.communicationStatus)) {
                            reason = "Communication is " + status.communicationStatus + " (rule requires Normal)";
                        } else {
                            reason = "Bigraph model may not be updated yet";
                        }
                        System.out.println("  ⚠ Takeoff rule not matched: " + reason);
                        System.out.println("  → Physical state is flying, updating memory status...");

                        status.status = "flying";//? todo: test if this is needed
                        status.takeoffRuleApplied = true;
                    }
                } catch (Exception e) {
                    System.err.println("  !! Failed to apply takeoff rule: " + e.getMessage());
                    e.printStackTrace();
                }
            }
        }

        return anyRuleApplied;
    }

    private boolean checkAndApplyLandingRules() {
        boolean anyRuleApplied = false;
        long currentTime = System.currentTimeMillis();

        for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
            String droneId = entry.getKey();
            DroneStatus status = entry.getValue();

            if (status.reachedDestination && !status.landingCommandSent) {
                try {
                    System.out.println("\n[Target Reached] " + droneId + " reached target, beginning landing immediately...");

                    int droneIndex = Integer.parseInt(droneId.substring(1));
                    int port = droneControlStartPort + droneIndex;

                    if (beginLanding(droneId, port)) {
                        status.landingCommandSent = true;
                        status.landingCommandTime = currentTime;
                        System.out.println("  ✓ " + droneId + " landing command sent");
                    } else {
                        System.err.println(" !! Unable to send landing command");
                    }
                } catch (Exception e) {
                    System.err.println(" !! Error sending landing command: " + e.getMessage());
                    e.printStackTrace();
                }
                continue; 
            }

            if (status.landingCommandSent && !status.landingRuleApplied) {
                long timeSinceLandingCommand = currentTime - status.landingCommandTime;

                if (timeSinceLandingCommand >= 2000 && status.hasLanded) { 
                    try {
                        System.out.println("\n [Rule Apply] " + droneId + " landed (altitude: " +
                                String.format("%.3f", status.z) + "m), applying landing rule...");

                        ParametricReactionRule<PureBigraph> landingRule = droneLandingRule(droneId);
                        if (applyRuleAndPersist(droneId, landingRule, "status updated to Landed", "apply landing rule")) {
                            status.status = "Landed";
                            status.landingRuleApplied = true;
                            anyRuleApplied = true;
                        } else {
                            status.landingRuleApplied = true;
                        }
                    } catch (Exception e) {
                        System.err.println("  !! Failed to apply landing rule: " + e.getMessage());
                        e.printStackTrace();
                    }
                }
            }
        }

        return anyRuleApplied;
    }

    //                         curl "http://localhost:8090/inject/comm?drone=D2&state=clear"

    private void startFaultInjectionServer() {
        if (!faultInjectionEnabled) {
            System.out.println("Fault-injection HTTP server disabled (fault.injection.enabled=false)");
            return;
        }
        try {
            HttpServer server = HttpServer.create(new InetSocketAddress(faultInjectionPort), 0);
            server.createContext("/inject/battery", ex -> handleInject(ex, true));
            server.createContext("/inject/comm", ex -> handleInject(ex, false));
            server.createContext("/inject/status", this::handleInjectStatus);
            server.setExecutor(null); 
            server.start();
            System.out.println("\n========================================");
            System.out.println("✓ Fault-injection HTTP server listening on port " + faultInjectionPort);
            System.out.println("  battery: curl \"http://localhost:" + faultInjectionPort + "/inject/battery?drone=D1&level=Low|Empty|Normal|clear\"");
            System.out.println("  comm:    curl \"http://localhost:" + faultInjectionPort + "/inject/comm?drone=D2&state=Bad|Normal|clear\"");
            System.out.println("  status:  curl \"http://localhost:" + faultInjectionPort + "/inject/status\"");
            System.out.println("========================================\n");
        } catch (Exception e) {
            System.err.println("!! Failed to start fault-injection server on port " + faultInjectionPort + ": " + e.getMessage());
        }
    }

    private void handleInject(HttpExchange ex, boolean battery) throws IOException {
        Map<String, String> q = parseQuery(ex.getRequestURI().getRawQuery());
        String drone = q.get("drone");
        String level = battery ? firstNonNull(q.get("level"), q.get("value"))
                : firstNonNull(q.get("state"), q.get("value"));

        if (drone == null || level == null) {
            respond(ex, 400, "missing parameter. need drone= and level=/state=/value=\n");
            return;
        }
        DroneStatus st = droneStatuses.get(drone);
        if (st == null) {
            respond(ex, 404, "unknown drone '" + drone + "'. available: " + droneStatuses.keySet() + "\n");
            return;
        }

        String msg = battery ? applyBatteryInjection(st, level) : applyCommInjection(st, level);
        if (msg == null) {
            respond(ex, 400, "invalid value '" + level + "'\n");
            return;
        }
        System.out.println("🧪 [INJECT] " + drone + " " + msg);
        respond(ex, 200, "OK: " + drone + " " + msg + "\n");
    }

    private String applyBatteryInjection(DroneStatus st, String level) {
        Double v;
        switch (level.toLowerCase(Locale.ROOT)) {
            case "normal": v = INJECT_BATTERY_NORMAL_V; break;
            case "low":    v = INJECT_BATTERY_LOW_V;    break;
            case "empty":  v = INJECT_BATTERY_EMPTY_V;  break;
            case "clear": case "off": case "none":
                st.injectedBatteryVoltage = null;
                return "battery injection cleared (back to real ROS value)";
            default:
                try { v = Double.parseDouble(level); } catch (NumberFormatException e) { return null; }
        }
        st.injectedBatteryVoltage = v;
        st.batteryVoltage = v; 
        return "battery injected -> " + String.format(Locale.ROOT, "%.2fV", v);
    }

    private String applyCommInjection(DroneStatus st, String state) {
        Integer r;
        switch (state.toLowerCase(Locale.ROOT)) {
            case "normal": r = INJECT_RSSI_NORMAL; break;
            case "bad":    r = INJECT_RSSI_BAD;    break;
            case "clear": case "off": case "none":
                st.injectedRssi = null;
                return "comm injection cleared (back to real ROS value)";
            default:
                try { r = Integer.parseInt(state); } catch (NumberFormatException e) { return null; }
        }
        st.injectedRssi = r;
        st.rssi = r; 
        return "comm injected -> RSSI " + r;
    }

    private void handleInjectStatus(HttpExchange ex) throws IOException {
        StringBuilder sb = new StringBuilder();
        for (Map.Entry<String, DroneStatus> e : droneStatuses.entrySet()) {
            DroneStatus s = e.getValue();
            sb.append(e.getKey())
                    .append(": battery=").append(s.batteryLevel)
                    .append(String.format(Locale.ROOT, "(%.2fV)", s.batteryVoltage))
                    .append(s.injectedBatteryVoltage != null ? "[INJECTED]" : "")
                    .append(", comm=").append(s.communicationStatus)
                    .append("(RSSI ").append(s.rssi).append(")")
                    .append(s.injectedRssi != null ? "[INJECTED]" : "")
                    .append("\n");
        }
        respond(ex, 200, sb.toString());
    }

    private static Map<String, String> parseQuery(String raw) {
        Map<String, String> m = new HashMap<>();
        if (raw == null || raw.isEmpty()) return m;
        for (String pair : raw.split("&")) {
            int i = pair.indexOf('=');
            if (i > 0) m.put(pair.substring(0, i), pair.substring(i + 1));
        }
        return m;
    }

    private static String firstNonNull(String a, String b) { return a != null ? a : b; }

    private static void respond(HttpExchange ex, int code, String body) throws IOException {
        byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
        ex.getResponseHeaders().set("Content-Type", "text/plain; charset=utf-8");
        ex.sendResponseHeaders(code, bytes.length);
        try (OutputStream os = ex.getResponseBody()) {
            os.write(bytes);
        }
    }

    private void continuousStatusMonitoring() {
        try {
            System.out.println("\n========================================");
            System.out.println("Battery and communication status monitoring thread started");
            System.out.println("========================================\n");

            while (true) {
                TimeUnit.MILLISECONDS.sleep(1000);  

                if (!rosUpdateEnabled) {
                    continue;
                }

                for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
                    String droneId = entry.getKey();
                    DroneStatus status = entry.getValue();

                    try {

                        checkAndApplyBatteryRule(droneId, status);

                        checkAndApplyCommunicationRule(droneId, status);

                        checkAndTriggerEmergencyResponse(droneId, status);

                        checkEmergencyRecovery(droneId, status);
                    } catch (Exception e) {
                        System.err.println("  !! Error processing " + droneId + " status monitoring: " + e.getMessage());
                        e.printStackTrace();
                    }
                }
            }
        } catch (Exception e) {
            System.err.println("!! Status monitoring thread exception: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private void checkAndApplyBatteryRule(String droneId, DroneStatus status) {
        long timeSinceTakeoff = System.currentTimeMillis() - status.takeoffTime;
        boolean inTakeoffProtection = (timeSinceTakeoff < TAKEOFF_PROTECTION_MS) &&
                status.takeoffCommandSent && !status.takeoffRuleApplied;

        boolean changed = true;
        while (changed) {
            changed = false;
            if (status.batteryVoltage < BATTERY_EMPTY_THRESHOLD && "Low".equals(status.batteryLevel)) {
                applyBatteryTransition(droneId, status, "Low", "Empty", true);
                changed = true;
            } else if (status.batteryVoltage < BATTERY_LOW_THRESHOLD && "Normal".equals(status.batteryLevel) && !inTakeoffProtection) {
                applyBatteryTransition(droneId, status, "Normal", "Low", true);
                changed = true;
            } else if (status.batteryVoltage >= BATTERY_LOW_THRESHOLD && "Low".equals(status.batteryLevel)) {
                if (!applyBatteryTransition(droneId, status, "Low", "Normal", false)) {
                    status.bigraphBatteryNeedsSync = true;
                } else {
                    status.bigraphBatteryNeedsSync = false;
                }
                changed = true;
            } else if (status.batteryVoltage >= BATTERY_EMPTY_THRESHOLD && status.batteryVoltage < BATTERY_LOW_THRESHOLD && "Empty".equals(status.batteryLevel)) {
                applyBatteryTransition(droneId, status, "Empty", "Low", false);
                changed = true;
            }
        }
    }

    private boolean applyBatteryTransition(String droneId, DroneStatus status, String from, String to, boolean setApplied) {
        try {
            System.out.println("[Battery] " + droneId + ": " + from + " → " + to + " (" + String.format("%.2fV", status.batteryVoltage) + ")");
            boolean success = applyRuleAndPersist(droneId, createBatteryRule(droneId, from, to), "battery → " + to,
                    "battery " + from + "→" + to);
            status.batteryLevel = to;
            status.batteryRuleApplied = setApplied;
            return success;
        } catch (Exception e) {
            System.err.println("  !! Battery rule failed: " + e.getMessage());
            status.batteryLevel = to;
            return false;
        }
    }

    private void checkAndApplyCommunicationRule(String droneId, DroneStatus status) {
        try {
            if (status.rssi >= RSSI_BAD_THRESHOLD && !"Bad".equals(status.communicationStatus)) {
                applyCommunicationTransition(droneId, status, "Normal", "Bad");
            } else if (status.rssi < RSSI_BAD_THRESHOLD && "Bad".equals(status.communicationStatus)) {
                applyCommunicationTransition(droneId, status, "Bad", "Normal");
            }
        } catch (Exception e) {
            System.err.println("  !! Communication rule failed: " + e.getMessage());
        }
    }

    private void applyCommunicationTransition(String droneId, DroneStatus status, String from, String to) throws Exception {
        System.out.println("[Comm] " + droneId + ": " + from + " → " + to + " (RSSI: " + status.rssi + ")");
        applyRuleAndPersist(droneId, createCommunicationRule(droneId, from, to), "comm → " + to, "comm " + from + "→" + to);
        status.communicationStatus = to;
        status.communicationRuleApplied = true;
    }

    /** Allow an existing emergency to escalate to local landing when the battery is empty. */
    private void checkAndTriggerEmergencyResponse(String droneId, DroneStatus status) {
        if (!status.hasTakenOff) return;

        boolean lowBattery = "Low".equals(status.batteryLevel);
        boolean emptyBattery = "Empty".equals(status.batteryLevel);
        boolean badComm = "Bad".equals(status.communicationStatus);

        if (status.isInEmergency) {
            boolean alreadyLanding = "EMPTY_BATTERY".equals(status.emergencyType)
                    || "LOW_BATTERY_BAD_COMM".equals(status.emergencyType);
            if (emptyBattery && !alreadyLanding && !status.emergencyLanded) {
                System.out.println("\n🚨 [ESCALATE] " + droneId + " battery Empty during "
                        + status.emergencyType + " → switching to immediate local landing");
                triggerEmergencyLanding(droneId, status, "EMPTY_BATTERY");
            }
            return;
        }

        long timeSinceTakeoff = System.currentTimeMillis() - status.takeoffTime;
        boolean inTakeoffProtection = (timeSinceTakeoff < TAKEOFF_PROTECTION_MS) &&
                status.takeoffCommandSent && !status.takeoffRuleApplied;

        if (emptyBattery) {
            triggerEmergencyLanding(droneId, status, "EMPTY_BATTERY");
        } else if (lowBattery && badComm) {
            triggerEmergencyLanding(droneId, status, "LOW_BATTERY_BAD_COMM");
        } else if (lowBattery && !badComm && !inTakeoffProtection) {
            triggerReturnForRecharge(droneId, status);
        } else if (badComm && !lowBattery && !emptyBattery && !inTakeoffProtection) {
            triggerReturnToHome(droneId, status);
        }
    }

    private void triggerEmergencyLanding(String droneId, DroneStatus status, String emergencyType) {
        System.out.println("\n🚨 [EMERGENCY] " + droneId + " " + emergencyType);

        DronePosition currentPos = dronePositions.get(droneId);
        if (currentPos == null) return;

        int emergencyGridIndex = coordinateToGridIndex(currentPos.x, currentPos.y, gridOriginZ);
        if (emergencyGridIndex < 0) return;

        GridPoint emergencyLandingPoint = gridIndexToPoint(emergencyGridIndex);
        if (emergencyLandingPoint == null) return;

        if (!setEmergencyState(droneId, status, emergencyType, emergencyLandingPoint)) return;
        System.out.println("  → Emergency target: " + emergencyLandingPoint);
    }

    private void triggerReturnForRecharge(String droneId, DroneStatus status) {
        System.out.println("\n⚠️ [LOW_BATTERY] " + droneId + " (" + String.format("%.2fV", status.batteryVoltage) + ")");

        DronePosition currentPos = dronePositions.get(droneId);
        if (currentPos == null) return;

        GridPoint currentPoint = gridIndexToPoint(currentPos.gridIndex);
        if (currentPoint == null) return;

        GridPoint startPoint = (status.startGridIndex != null) ? gridIndexToPoint(status.startGridIndex) : null;

        if (!setEmergencyState(droneId, status, "LOW_BATTERY", null)) return;

        // Find nearest available charging station
        GridPoint selectedStation = findNearestAvailableChargingStation(droneId, currentPoint);
        double distToHome = (startPoint != null) ? heuristic(currentPoint, startPoint) : Double.MAX_VALUE;

        if (selectedStation != null && heuristic(currentPoint, selectedStation) <= distToHome) {

            status.emergencyTarget = selectedStation;
            System.out.println("  → Charging Station: " + selectedStation);
        } else if (startPoint != null) {

            status.emergencyTarget = startPoint;
            if (selectedStation == null) {
                String reason = chargingStations.isEmpty()
                        ? "No charging station configured"
                        : "Charging station(s) occupied/reserved/targeted by others";
                System.out.println("  → " + reason + ", returning to start point: " + startPoint);
            } else {
                System.out.println("  → Start point is closer, returning to: " + startPoint);
            }
        } else {

            System.err.println("  !! No start point available, triggering emergency landing");
            triggerEmergencyLanding(droneId, status, "LOW_BATTERY");
            return;
        }
    }

    private void triggerReturnToHome(String droneId, DroneStatus status) {
        System.out.println("\n📡 [BAD_COMM] " + droneId + " (RSSI: " + status.rssi + ")");

        GridPoint startPoint = (status.startGridIndex != null) ? gridIndexToPoint(status.startGridIndex) : null;
        if (startPoint == null) return;

        if (!setEmergencyState(droneId, status, "BAD_COMM", startPoint)) return;
        System.out.println("  → Start Point: " + startPoint);
    }

    /** Cancel the old motion before releasing its reservations and choosing a new target. */
    private boolean setEmergencyState(String droneId, DroneStatus status, String type, GridPoint target) {
        int port = droneControlStartPort + Integer.parseInt(droneId.substring(1));
        if (!sendDroneControlRequest(droneId, port, "/stop_controller", "Holding before emergency reroute.", "emergency hold")) {
            // Keep the old reservation and retry through the existing status monitor.
            return false;
        }
        status.isInEmergency = true;
        status.emergencyType = type;
        status.isMoving = false;
        status.movingToGrid = null;
        status.isWaiting = false;
        status.waitingForGrid = null;
        releaseAllGrids(droneId);
        if (status.originalTarget == null) status.originalTarget = droneTargets.get(droneId);
        if (target != null) status.emergencyTarget = target;
        return true;
    }

    private GridPoint findNearestAvailableChargingStation(String droneId, GridPoint currentPoint) {
        GridPoint nearest = null;
        double minDist = Double.MAX_VALUE;

        for (GridPoint station : chargingStations) {
            boolean occupied = dronePositions.entrySet().stream()
                    .anyMatch(e -> !e.getKey().equals(droneId) && e.getValue().gridIndex == station.gridIndex);
            String reservedBy = getGridReservation(station.gridIndex);
            boolean reserved = (reservedBy != null && !reservedBy.equals(droneId));
            boolean targeted = droneStatuses.entrySet().stream()
                    .anyMatch(e -> !e.getKey().equals(droneId) && e.getValue().isInEmergency &&
                            "LOW_BATTERY".equals(e.getValue().emergencyType) &&
                            e.getValue().emergencyTarget != null &&
                            e.getValue().emergencyTarget.gridIndex == station.gridIndex);

            if (!occupied && !reserved && !targeted) {
                double dist = heuristic(currentPoint, station);
                if (dist < minDist) {
                    minDist = dist;
                    nearest = station;
                }
            }
        }
        return nearest;
    }

    private void checkEmergencyRecovery(String droneId, DroneStatus status) {
        if ("BAD_COMM".equals(status.emergencyType) && "Normal".equals(status.communicationStatus)) {
            System.out.println("📡 [INFO] " + droneId + " Comm recovered, continuing home");
        }
    }

    private Ros sharedRos;

    private synchronized boolean ensureRosConnected(String host) {
        if (sharedRos != null) {
            return true;
        }
        try {
            Ros ros = new Ros(host);
            ros.connect();
            sharedRos = ros;
            System.out.println("✓ Connected to rosbridge at " + host + ":9090 (shared connection)");
            return true;
        } catch (Exception e) {
            System.err.println("!! rosbridge connect error at " + host + ":9090 "
                    + "(is rosbridge_server / the sim position publisher running?): " + e.getMessage());
            return false;
        }
    }

    private void subscribeRosTopic(String host, String topic, String type, RosMessageHandler handler) {
        try {
            if (!ensureRosConnected(host)) {
                System.err.println("Failed to subscribe (no rosbridge connection) to topic: " + topic);
                return;
            }
            Topic rosTopic = new Topic(sharedRos, topic, type);
            rosTopic.subscribe(message -> {
                try {
                    handler.handle(message);
                } catch (Exception e) {
                    System.err.println("Error processing ROS2 message (topic: " + topic + "): " + e.getMessage());
                    e.printStackTrace();
                }
            });
            System.out.println("✓ Subscribed to ROS2 topic: " + topic);
        } catch (Exception e) {
            System.err.println("Failed to subscribe to ROS2 topic (topic: " + topic + "): " + e.getMessage());
            e.printStackTrace();
        }
    }

    private void initializeRosSubscriptions() {
        if (!rosUpdateEnabled) {
            System.out.println("\n========================================");
            System.out.println("ROS2 update function not enabled");
            System.out.println("To enable, set: ros.update.enabled=true");
            System.out.println("========================================\n");
            return;
        }

        System.out.println("\n========================================");
        System.out.println("Initializing ROS2 subscriptions");
        System.out.println("========================================");
        System.out.println("ROS Bridge Host: " + rosBridgeHost);
        System.out.println("Drone Count: " + configuredDroneCount);
        System.out.println("ROS Sim Mode: " + rosSimMode);

        for (int i = 0; i < configuredDroneCount; i++) {
            String droneId = "D" + i;

            dronePositions.put(droneId, new DronePosition(droneId, 0, 0, 0, i));

            droneStatuses.put(droneId, new DroneStatus(droneId));
        }
        System.out.println("✓ Initialized default positions and status for " + configuredDroneCount + " drones");

        if (rosSimMode) {

            subscribeRosTopic(rosBridgeHost, "/cf_positions_path", "nav_msgs/Path", message -> {
                try {
                    JsonObject jsonObject = message.toJsonObject();
                    if (!jsonObject.containsKey("poses")) {
                        return;
                    }
                    javax.json.JsonArray poses = jsonObject.getJsonArray("poses");
                    if (poses == null || poses.isEmpty()) {
                        return;
                    }
                    for (int i = 0; i < poses.size(); i++) {
                        JsonObject poseStamped = poses.getJsonObject(i);
                        JsonObject header = poseStamped.getJsonObject("header");
                        if (header == null || !header.containsKey("frame_id")) {
                            continue;
                        }
                        String frameId = header.getString("frame_id");

                        int cfNumber;
                        try {
                            if (!frameId.startsWith("cf")) {
                                continue;
                            }
                            cfNumber = Integer.parseInt(frameId.substring(2));
                        } catch (NumberFormatException e) {
                            continue;
                        }
                        int droneIndex = cfNumber - 231;
                        if (droneIndex < 0 || droneIndex >= configuredDroneCount) {
                            continue;
                        }
                        String droneId = "D" + droneIndex;
                        JsonObject pose = poseStamped.getJsonObject("pose");
                        if (pose == null || !pose.containsKey("position")) {
                            continue;
                        }
                        JsonObject position = pose.getJsonObject("position");
                        double x = position.getJsonNumber("x").doubleValue();
                        double y = position.getJsonNumber("y").doubleValue();
                        double z = position.getJsonNumber("z").doubleValue();
                        recordRosPoseSample();
                        int gridIndex = coordinateToGridIndex(x, y, z);
                        DroneStatus status = droneStatuses.get(droneId);
                        if (status != null) {
                            status.z = z;
                            final long TAKEOFF_COOLDOWN = 5000;
                            long timeSinceTakeoff = System.currentTimeMillis() - status.takeoffTime;
                            if (!status.hasTakenOff && DroneControllerSync.hasLiftedOff(
                                    status.takeoffCommandSent, status.takeoffStartZ, z)) {
                                status.hasTakenOff = true;
                                status.takeoffTime = System.currentTimeMillis();
                                System.out.println("[Takeoff Detection] " + droneId + " took off! Altitude: " + String.format("%.3f", z) + "m");
                            }
                            if (z <= 0.1 && status.landingCommandSent && !status.hasLanded && timeSinceTakeoff > TAKEOFF_COOLDOWN) {
                                status.hasLanded = true;
                                System.out.println("[Landing Detection] " + droneId + " landed! Altitude: " + String.format("%.3f", z) + "m");
                            }
                        }
                        DronePosition oldPos = dronePositions.get(droneId);
                        DronePosition newPos = new DronePosition(droneId, x, y, z, gridIndex);
                        boolean shouldUpdate = (oldPos == null || oldPos.gridIndex != gridIndex);
                        dronePositions.put(droneId, newPos);
                        if (shouldUpdate) {
                            checkCollisionRisk();
                            System.out.println("[ROS2] " + droneId + " (" + frameId + " sim): " +
                                    String.format("(%.2f, %.2f, %.2f)", x, y, z) + " -> Grid[" + gridIndex + "]" +
                                    (oldPos != null ? " (From Grid[" + oldPos.gridIndex + "])" : " (Initial Position)"));
                        }
                    }
                } catch (Exception e) {
                    System.err.println("  ⚠ Error parsing /cf_positions_path message: " + e.getMessage());
                }
            });
        } else {

            for (int i = 0; i < configuredDroneCount; i++) {
                int droneIndex = i;
                String droneId = "D" + i;
                int cfNumber = 231 + i;
                String topic = "/cf" + cfNumber + "/pose";

                subscribeRosTopic(rosBridgeHost, topic, "geometry_msgs/PoseStamped", message -> {
                    JsonObject jsonObject = message.toJsonObject();

                    JsonObject pose = jsonObject.getJsonObject("pose");
                    JsonObject position = pose.getJsonObject("position");
                    double x = position.getJsonNumber("x").doubleValue();
                    double y = position.getJsonNumber("y").doubleValue();
                    double z = position.getJsonNumber("z").doubleValue();

                    recordRosPoseSample();
                    int gridIndex = coordinateToGridIndex(x, y, z);

                    DroneStatus status = droneStatuses.get(droneId);
                    if (status != null) {
                        status.z = z;

                        final long TAKEOFF_COOLDOWN = 5000;
                        long timeSinceTakeoff = System.currentTimeMillis() - status.takeoffTime;

                        if (!status.hasTakenOff && DroneControllerSync.hasLiftedOff(
                                status.takeoffCommandSent, status.takeoffStartZ, z)) {
                            status.hasTakenOff = true;
                            status.takeoffTime = System.currentTimeMillis();
                            System.out.println("[Takeoff Detection] " + droneId + " took off! Altitude: " + String.format("%.3f", z) + "m");
                        }

                        if (z <= 0.1 && status.landingCommandSent && !status.hasLanded && timeSinceTakeoff > TAKEOFF_COOLDOWN) {
                            status.hasLanded = true;
                            System.out.println("[Landing Detection] " + droneId + " landed! Altitude: " + String.format("%.3f", z) + "m");
                        }
                    }

                    DronePosition oldPos = dronePositions.get(droneId);
                    DronePosition newPos = new DronePosition(droneId, x, y, z, gridIndex);

                    boolean shouldUpdate = false;
                    if (oldPos == null || oldPos.gridIndex != gridIndex) {
                        shouldUpdate = true;
                    }

                    dronePositions.put(droneId, newPos);
                    if (shouldUpdate) {
                        checkCollisionRisk();
                        System.out.println("[ROS2] " + droneId + " (cf" + cfNumber + "): " +
                                String.format("(%.2f, %.2f, %.2f)", x, y, z) + " -> Grid[" + gridIndex + "]" +
                                (oldPos != null ? " (From Grid[" + oldPos.gridIndex + "])" : " (Initial Position)"));
                    }
                });
            }
        }

        for (int i = 0; i < configuredDroneCount; i++) {
            String droneId = "D" + i;
            int cfNumber = 231 + i;
            String statusTopic = "/cf" + cfNumber + "/status";
            subscribeRosTopic(rosBridgeHost, statusTopic, "crazyflie_interfaces/msg/Status", message -> {
                try {
                    JsonObject jsonObject = message.toJsonObject();

                    if (jsonObject.containsKey("battery_voltage") && jsonObject.containsKey("rssi")) {
                        double batteryVoltage = jsonObject.getJsonNumber("battery_voltage").doubleValue();
                        int rssi = jsonObject.getJsonNumber("rssi").intValue();

                        DroneStatus status = droneStatuses.get(droneId);
                        if (status != null) {

                            status.batteryVoltage = (status.injectedBatteryVoltage != null)
                                    ? status.injectedBatteryVoltage : batteryVoltage;
                            status.rssi = (status.injectedRssi != null)
                                    ? status.injectedRssi : rssi;
                        }
                    }
                } catch (Exception e) {
                    System.err.println("  ⚠ Error parsing " + droneId + " status message: " + e.getMessage());
                }
            });
        }

        System.out.println("========================================\n");
    }

    /** Round XY to grid centers, floor Z by layer height, and flatten in layer/X/Y order. */
    private int coordinateToGridIndex(double x, double y, double z) {

        final double EPSILON = 1e-6;

        if (Math.abs(x) < EPSILON) x = 0.0;
        if (Math.abs(y) < EPSILON) y = 0.0;
        if (Math.abs(z) < EPSILON) z = 0.0;

        double relX = x - gridOriginX;
        double relY = y - gridOriginY;
        double relZ = z - gridOriginZ;

        int xIndex = (int) Math.round(relX / gridStepX);
        int yIndex = (int) Math.round(relY / gridStepY);

        // 0 <= z < 0.3 -> layer 0
        // 0.3 <= z < 0.6 -> layer 1
        // 0.6 <= z < 0.9 -> layer 2
        int layerIndex = (int) Math.floor(relZ / layerHeight);

        if (xIndex < 0 || xIndex >= bigridCols ||
                yIndex < 0 || yIndex >= bigridRows ||
                layerIndex < 0 || layerIndex >= bigridLayers) {
            return -1; 
        }

        int index = layerIndex * (bigridCols * bigridRows) + xIndex * bigridRows + yIndex;

        return index;
    }

    // /**

    //  */
    // @Deprecated

    private void parseDroneTargets() {
        System.out.println("\n========================================");
        System.out.println("Parsing drone target points, charging station, and obstacles");
        System.out.println("========================================");

        obstacleGrids.clear();
        if (obstacleGridsConfig != null && !obstacleGridsConfig.trim().isEmpty()) {
            String[] grids = obstacleGridsConfig.split(",");
            System.out.println("  Obstacles:");
            for (String gridStr : grids) {
                try {
                    int gridIndex = Integer.parseInt(gridStr.trim());
                    GridPoint obstacle = gridIndexToPoint(gridIndex);
                    if (obstacle != null) {
                        obstacleGrids.add(gridIndex);
                        System.out.println("    - Grid[" + gridIndex + "] (" +
                                String.format("%.1f, %.1f, %.1f", obstacle.x, obstacle.y, obstacle.z) + ")");
                    } else {
                        System.err.println("    !! Invalid obstacle grid: " + gridIndex);
                    }
                } catch (NumberFormatException e) {
                    System.err.println("    !! Invalid obstacle grid format: " + gridStr);
                }
            }
            if (obstacleGrids.isEmpty()) {
                System.out.println("    (No obstacles configured)");
            }
        } else {
            System.out.println("  (No obstacles configured)");
        }

        chargingStations.clear();
        if (chargingStationGrids != null && !chargingStationGrids.trim().isEmpty()) {
            String[] grids = chargingStationGrids.split(",");
            System.out.println("  Charging Stations:");
            for (String gridStr : grids) {
                try {
                    int gridIndex = Integer.parseInt(gridStr.trim());
                    GridPoint station = gridIndexToPoint(gridIndex);
                    if (station != null) {
                        chargingStations.add(station);
                        System.out.println("    - Grid[" + gridIndex + "] (" +
                                String.format("%.1f, %.1f, %.1f", station.x, station.y, station.z) + ")");
                    } else {
                        System.err.println("    !! Invalid charging station grid: " + gridIndex);
                    }
                } catch (NumberFormatException e) {
                    System.err.println("    !! Invalid charging station grid format: " + gridStr);
                }
            }
            if (chargingStations.isEmpty()) {
                System.err.println("  !! No valid charging stations configured");
            }
        } else {
            System.err.println("  !! No charging stations configured");
        }

        String[] targets = droneTargetsConfig.split(";");
        for (int i = 0; i < targets.length && i < configuredDroneCount; i++) {
            String[] coords = targets[i].trim().split(",");
            if (coords.length == 2) {
                try {
                    double x = Double.parseDouble(coords[0].trim());
                    double y = Double.parseDouble(coords[1].trim());

                    double targetZ = gridOriginZ;  
                    int gridIndex = coordinateToGridIndex(x, y, targetZ);

                    String droneId = "D" + i;

                    droneTargets.put(droneId, new GridPoint(x, y, targetZ, gridIndex));
                    System.out.println("  " + droneId + " target: (" + x + ", " + y + ") -> Grid[" + gridIndex + "] (Layer 1)");

                    DroneStatus status = droneStatuses.get(droneId);
                    DronePosition currentPos = dronePositions.get(droneId);
                    if (status != null) {

                        if (currentPos != null && currentPos.gridIndex >= 0) {
                            status.startGridIndex = currentPos.gridIndex;
                            System.out.println("  " + droneId + " start position: Grid[" + currentPos.gridIndex + "] (from ROS2)");
                        } else {

                            status.startGridIndex = i;
                            System.out.println("  " + droneId + " start position: Grid[" + i + "] (default)");
                        }
                        status.originalTarget = droneTargets.get(droneId);
                    }
                } catch (NumberFormatException e) {
                    System.err.println("  !! Failed to parse target point: " + targets[i]);
                }
            }
        }
        System.out.println("========================================\n");
    }

    private GridPoint gridIndexToPoint(int gridIndex) {
        int totalGridsPerLayer = bigridCols * bigridRows;
        if (gridIndex < 0 || gridIndex >= totalGridsPerLayer * bigridLayers) {
            return null;
        }

        int layerIndex = gridIndex / totalGridsPerLayer;
        int indexInLayer = gridIndex % totalGridsPerLayer;

        int xIndex = indexInLayer / bigridRows;
        int yIndex = indexInLayer % bigridRows;

        double x = gridOriginX + xIndex * gridStepX;
        double y = gridOriginY + yIndex * gridStepY;
        double z = gridOriginZ + layerIndex * layerHeight;

        return new GridPoint(x, y, z, gridIndex);
    }

    private List<GridPoint> getNeighbors(int gridIndex) {
        List<GridPoint> neighbors = new ArrayList<>();

        int totalGridsPerLayer = bigridCols * bigridRows;
        int layerIndex = gridIndex / totalGridsPerLayer;
        int indexInLayer = gridIndex % totalGridsPerLayer;

        int xIndex = indexInLayer / bigridRows;
        int yIndex = indexInLayer % bigridRows;

        int[][] horizontalDirections = {
                {-1, 0, 0},  
                {1, 0, 0},   
                {0, -1, 0},  
                {0, 1, 0},   
                {-1, -1, 0}, 
                {-1, 1, 0},  
                {1, -1, 0},  
                {1, 1, 0}    
        };

        for (int[] dir : horizontalDirections) {
            int newX = xIndex + dir[0];
            int newY = yIndex + dir[1];
            int newLayer = layerIndex + dir[2];

            if (newX >= 0 && newX < bigridCols &&
                    newY >= 0 && newY < bigridRows &&
                    newLayer >= 0 && newLayer < bigridLayers) {
                int newIndex = newLayer * totalGridsPerLayer + newX * bigridRows + newY;
                GridPoint point = gridIndexToPoint(newIndex);
                if (point != null) {
                    neighbors.add(point);
                }
            }
        }

        if (layerIndex + 1 < bigridLayers) {
            int upIndex = (layerIndex + 1) * totalGridsPerLayer + xIndex * bigridRows + yIndex;
            GridPoint upPoint = gridIndexToPoint(upIndex);
            if (upPoint != null) {
                neighbors.add(upPoint);
            }
        }

        if (layerIndex - 1 >= 0) {
            int downIndex = (layerIndex - 1) * totalGridsPerLayer + xIndex * bigridRows + yIndex;
            GridPoint downPoint = gridIndexToPoint(downIndex);
            if (downPoint != null) {
                neighbors.add(downPoint);
            }
        }

        return neighbors;
    }

    private double heuristic(GridPoint a, GridPoint b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2) + Math.pow(a.z - b.z, 2));
    }

    private GridPoint planNextStep(String droneId, GridPoint start, GridPoint goal) {
        if (start.gridIndex == goal.gridIndex) {
            return null;  
        }

        Set<Integer> occupiedGrids = getOccupiedGrids(droneId);

        DroneStatus planStatus = droneStatuses.get(droneId);
        if (planStatus != null && !planStatus.avoidGrids.isEmpty()) {
            if (System.currentTimeMillis() - planStatus.avoidGridsSetTime < 30000) {
                occupiedGrids.addAll(planStatus.avoidGrids);
            } else {
                planStatus.avoidGrids.clear();
            }
        }

        List<GridPoint> fullPath = planPath(start, goal, occupiedGrids);

        if (!fullPath.isEmpty()) {
            StringBuilder sb = new StringBuilder();
            sb.append("  [Full Path] ").append(droneId).append(": ");
            for (int i = 0; i < fullPath.size(); i++) {
                GridPoint p = fullPath.get(i);
                sb.append("Grid[").append(p.gridIndex).append("]")
                        .append("(")
                        .append(String.format(Locale.ROOT, "%.1f,%.1f,%.1f", p.x, p.y, p.z))
                        .append(")");
                if (i != fullPath.size() - 1) {
                    sb.append(" -> ");
                }
            }
            System.out.println(sb.toString());
        }

        if (fullPath.isEmpty() || fullPath.size() < 2) {
            return null;  
        }

        return fullPath.get(1);
    }

    /** A* uses the current occupancy and reservation snapshot. */
    private List<GridPoint> planPath(GridPoint start, GridPoint goal, Set<Integer> occupiedGrids) {
        PriorityQueue<AStarNode> openSet = new PriorityQueue<>();
        Set<Integer> closedSet = new HashSet<>();
        Map<Integer, Double> gScores = new HashMap<>();

        openSet.add(new AStarNode(start, null, 0, heuristic(start, goal)));
        gScores.put(start.gridIndex, 0.0);

        while (!openSet.isEmpty()) {
            AStarNode current = openSet.poll();

            if (current.point.gridIndex == goal.gridIndex) {
                return reconstructPath(current);
            }

            closedSet.add(current.point.gridIndex);

            for (GridPoint neighbor : getNeighbors(current.point.gridIndex)) {

                if (closedSet.contains(neighbor.gridIndex)) {
                    continue;
                }

                if (occupiedGrids.contains(neighbor.gridIndex) && neighbor.gridIndex != goal.gridIndex) {
                    continue;
                }

                boolean isDiagonalXY = Math.abs(neighbor.x - current.point.x) > 0.5 &&
                        Math.abs(neighbor.y - current.point.y) > 0.5;
                boolean isVertical = Math.abs(neighbor.z - current.point.z) > 0.01;

                double moveCost;
                if (isDiagonalXY && !isVertical) {
                    moveCost = Math.sqrt(2);  
                } else if (isVertical && !isDiagonalXY) {
                    moveCost = 1.5;  
                } else {
                    moveCost = 1.0;  
                }

                double tentativeGScore = current.gCost + moveCost;

                if (!gScores.containsKey(neighbor.gridIndex) || tentativeGScore < gScores.get(neighbor.gridIndex)) {
                    gScores.put(neighbor.gridIndex, tentativeGScore);
                    double hScore = heuristic(neighbor, goal);
                    openSet.add(new AStarNode(neighbor, current, tentativeGScore, hScore));
                }
            }
        }

        return new ArrayList<>();
    }

    private List<GridPoint> reconstructPath(AStarNode node) {
        List<GridPoint> path = new ArrayList<>();
        AStarNode current = node;
        while (current != null) {
            path.add(0, current.point);  
            current = current.parent;
        }
        return path;
    }

    private Set<Integer> getOccupiedGrids(String excludeDroneId) {
        Set<Integer> occupied = new HashSet<>();

        for (Map.Entry<String, DronePosition> entry : dronePositions.entrySet()) {
            if (!entry.getKey().equals(excludeDroneId)) {
                DronePosition pos = entry.getValue();
                if (pos.gridIndex >= 0) {
                    occupied.add(pos.gridIndex);
                }
            }
        }

        occupied.addAll(obstacleGrids);

        for (Map.Entry<Integer, GridReservation> entry : gridReservations.entrySet()) {
            GridReservation reservation = entry.getValue();
            if (!reservation.droneId.equals(excludeDroneId)) {
                occupied.add(entry.getKey());
            }
        }

        return occupied;
    }

    /** Atomically reject reservations owned or occupied by another drone. */
    private boolean tryReserveGrid(String droneId, int gridIndex) {
        GridReservation existingReservation = gridReservations.get(gridIndex);

        if (existingReservation != null) {

            if (existingReservation.droneId.equals(droneId)) {
                return true;
            }

            return false;
        }

        GridReservation newReservation = new GridReservation(droneId, System.currentTimeMillis());
        GridReservation previous = gridReservations.putIfAbsent(gridIndex, newReservation);

        if (previous == null) {
            return true;
        }

        return previous.droneId.equals(droneId);
    }

    private void releaseGrid(String droneId, int gridIndex) {
        GridReservation reservation = gridReservations.get(gridIndex);
        if (reservation != null && reservation.droneId.equals(droneId)) {
            gridReservations.remove(gridIndex);
        }
    }

    private void releaseAllGrids(String droneId) {
        gridReservations.entrySet().removeIf(entry -> entry.getValue().droneId.equals(droneId));
    }

    private String getGridReservation(int gridIndex) {
        GridReservation reservation = gridReservations.get(gridIndex);
        return (reservation != null) ? reservation.droneId : null;
    }

    /** Debounce persistent same-cell occupancy independently of path authorization. */
    private void checkCollisionRisk() {
        Map<Integer, List<String>> gridOccupancy = new HashMap<>();

        for (DronePosition pos : dronePositions.values()) {
            if (pos.gridIndex >= 0) {
                gridOccupancy.computeIfAbsent(pos.gridIndex, k -> new ArrayList<>())
                        .add(pos.droneId);
            }
        }

        Set<Integer> currentViolatingGrids = new HashSet<>();
        boolean riskDetected = false;
        Map<Integer, List<String>> violatingOccupants = new HashMap<>();
        for (Map.Entry<Integer, List<String>> entry : gridOccupancy.entrySet()) {
            if (entry.getValue().size() > 1) {
                riskDetected = true;
                currentViolatingGrids.add(entry.getKey());
                violatingOccupants.put(entry.getKey(), entry.getValue());
                if (!collisionRiskDetected.get()) {
                    System.err.println("\n⚠⚠⚠ Collision Risk Warning (potential, pre-debounce) ⚠⚠⚠");
                    System.err.println("Multiple drones in same grid:");
                }
                System.err.println("  Grid[" + entry.getKey() + "]: " +
                        String.join(", ", entry.getValue()));
            }
        }

        for (Integer gridIndex : currentViolatingGrids) {
            long airborneOccupants = violatingOccupants.get(gridIndex).stream()
                    .filter(id -> {
                        DroneStatus s = droneStatuses.get(id);
                        return s != null && s.hasTakenOff;
                    })
                    .count();
            if (airborneOccupants < 2) {

                coOccupancyStreak.remove(gridIndex);
                continue;
            }
            int streak = coOccupancyStreak.merge(gridIndex, 1, Integer::sum);
            if (streak >= violationDebounceFrames && !countedViolationGrids.contains(gridIndex)) {
                long total = coOccupancyViolationCount.incrementAndGet();
                countedViolationGrids.add(gridIndex);
                System.err.println("  ✗ CONFIRMED co-occupancy violation at Grid[" + gridIndex + "]: "
                        + String.join(", ", violatingOccupants.get(gridIndex))
                        + " (sustained " + streak + " frames ≥ " + violationDebounceFrames
                        + "; total confirmed = " + total + ")");
            }
        }

        coOccupancyStreak.keySet().retainAll(currentViolatingGrids);
        countedViolationGrids.retainAll(currentViolatingGrids);

        if (riskDetected && !collisionRiskDetected.get()) {
            System.err.println("Temporarily not merging models to avoid conflict");
            System.err.println("⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠\n");
        }

        collisionRiskDetected.set(riskDetected);
    }

    private void recordRosPoseSample() {
        long now = System.currentTimeMillis();
        if (firstRosPoseTimeMs == null) {
            firstRosPoseTimeMs = now;
        }
        lastRosPoseTimeMs = now;
    }

    /** Gate: only calls writeMissionReport("complete") once every configured drone has landed and stayed landed for LANDING_DELAY_MS. */
    private void maybePrintMissionReport() {
        if (missionReportPrinted.get()) {
            return;
        }

        for (int i = 0; i < configuredDroneCount; i++) {
            DroneStatus status = droneStatuses.get("D" + i);
            if (status == null || !status.hasLanded) {
                allDronesLandedAtMs = null;
                return;
            }
        }

        long now = System.currentTimeMillis();
        if (allDronesLandedAtMs == null) {
            allDronesLandedAtMs = now;
            return;
        }
        if (now - allDronesLandedAtMs < LANDING_DELAY_MS) {
            return;
        }

        writeMissionReport("complete");
    }

    /**
     * Builds and writes the mission report (completion time, violations, per-decision latencies)
     * from whatever metrics have been collected so far. Guarded by missionReportPrinted so it runs
     * at most once, whether triggered by normal completion or by the JVM shutdown hook (Ctrl+C).
     * @param runStatus "complete" (all drones landed) or "interrupted" (JVM exiting early)
     */
    /** Keep JSON field names stable for downstream analysis. */
    private void writeMissionReport(String runStatus) {
        if (!missionReportPrinted.compareAndSet(false, true)) {
            return;
        }

        double completionTimeSec = 0.0;
        if (firstRosPoseTimeMs != null && lastRosPoseTimeMs != null) {
            completionTimeSec = (lastRosPoseTimeMs - firstRosPoseTimeMs) / 1000.0;
        }

        String modeLabel = baselineMode ? "BASELINE" : "BIGRAPH";

        // Each stats block has the shape {"overall": {...pooled...}, "by_drone": {"D0": {...}, "D1": {...}, ...}}
        // so per-drone samples (e.g. cf231/D0 vs cf232/D1) are never pooled together at the source.
        Map<String, Object> planningStats = summarizeByDrone(planningLatenciesMsByDrone);
        Map<String, Object> gateStats = summarizeByDrone(gateMatchLatenciesMsByDrone);
        Map<String, Object> restStats = summarizeByDrone(restLatenciesMsByDrone);

        System.out.println("\n========== MISSION REPORT (" + modeLabel + ", " + runStatus + ") ==========");
        System.out.println("Co-occupancy violations: " + coOccupancyViolationCount.get());
        System.out.println("Completion time: " + String.format(Locale.ROOT, "%.2f", completionTimeSec) + " s");
        if (firstRosPoseTimeMs != null && lastRosPoseTimeMs != null) {
            System.out.println("  (ROS pose span: t0=" + firstRosPoseTimeMs + " t1=" + lastRosPoseTimeMs + ")");
        }
        System.out.println("Per-decision latency (ms), pooled across all drones:");
        System.out.println("  Planning : " + formatLatencySummary(overallOf(planningStats)));
        System.out.println("  Gate     : " + formatLatencySummary(overallOf(gateStats)));
        System.out.println("  REST     : " + formatLatencySummary(overallOf(restStats)));
        System.out.println("Per-drone gate (Match) latency (ms):");
        for (Map.Entry<String, Object> e : byDroneOf(gateStats).entrySet()) {
            @SuppressWarnings("unchecked")
            Map<String, Object> droneStats = (Map<String, Object>) e.getValue();
            System.out.println("  " + e.getKey() + " : " + formatLatencySummary(droneStats));
        }
        System.out.println("====================================================\n");

        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss"));
        String reportFileName = "mission-report-" + modeLabel.toLowerCase(Locale.ROOT) + "-" + runStatus + "-" + timestamp + ".json";
        try {
            Map<String, Object> report = new LinkedHashMap<>();
            report.put("run_status", runStatus);
            report.put("generated_at", timestamp);
            report.put("baseline_mode", baselineMode);
            report.put("co_occupancy_violations", coOccupancyViolationCount.get());
            report.put("completion_time_sec", completionTimeSec);
            report.put("first_ros_pose_ms", firstRosPoseTimeMs);
            report.put("last_ros_pose_ms", lastRosPoseTimeMs);
            report.put("planning_latency_ms", planningStats);
            report.put("gate_match_latency_ms", gateStats);
            report.put("rest_latency_ms", restStats);
            objectMapper.writerWithDefaultPrettyPrinter()
                    .writeValue(new java.io.File(reportFileName), report);
            System.out.println("✓ Mission report written to " + reportFileName);
        } catch (Exception e) {
            System.err.println("!! Failed to write " + reportFileName + ": " + e.getMessage());
        }
    }

    /** Computes count/mean/p50/p95/max (ms) from a collection of per-decision latency samples. */
    private static Map<String, Object> summarizeLatenciesMs(Collection<Double> samples) {
        Map<String, Object> stats = new LinkedHashMap<>();
        if (samples.isEmpty()) {
            stats.put("count", 0);
            stats.put("mean_ms", null);
            stats.put("p50_ms", null);
            stats.put("p95_ms", null);
            stats.put("max_ms", null);
            return stats;
        }
        List<Double> sorted = new ArrayList<>(samples);
        Collections.sort(sorted);
        double mean = sorted.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        stats.put("count", sorted.size());
        stats.put("mean_ms", mean);
        stats.put("p50_ms", percentile(sorted, 0.50));
        stats.put("p95_ms", percentile(sorted, 0.95));
        stats.put("max_ms", sorted.get(sorted.size() - 1));
        return stats;
    }

    /** Nearest-rank percentile over an ascending-sorted list. */
    private static double percentile(List<Double> sortedAscending, double p) {
        int idx = (int) Math.ceil(p * sortedAscending.size()) - 1;
        idx = Math.max(0, Math.min(sortedAscending.size() - 1, idx));
        return sortedAscending.get(idx);
    }

    private static String formatLatencySummary(Map<String, Object> stats) {
        int count = (int) stats.get("count");
        if (count == 0) {
            return "n=0 (no samples)";
        }
        return String.format(Locale.ROOT, "n=%d  mean=%.3f  p50=%.3f  p95=%.3f  max=%.3f",
                count, (double) stats.get("mean_ms"), (double) stats.get("p50_ms"),
                (double) stats.get("p95_ms"), (double) stats.get("max_ms"));
    }

    /**
     * Builds {"overall": pooled-summary, "by_drone": {droneId: summary, ...}} from a per-drone
     * sample store, so each drone's latency distribution stays separately inspectable (never
     * silently merged with another drone's), while still exposing a pooled figure for convenience.
     */
    private static Map<String, Object> summarizeByDrone(Map<String, Queue<Double>> byDrone) {
        List<Double> pooled = new ArrayList<>();
        for (Queue<Double> samples : byDrone.values()) {
            pooled.addAll(samples);
        }

        List<String> droneIds = new ArrayList<>(byDrone.keySet());
        droneIds.sort(Comparator.comparingInt(Application3D::droneIndexOf));

        Map<String, Object> perDrone = new LinkedHashMap<>();
        for (String droneId : droneIds) {
            perDrone.put(droneId, summarizeLatenciesMs(byDrone.get(droneId)));
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("overall", summarizeLatenciesMs(pooled));
        result.put("by_drone", perDrone);
        return result;
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> overallOf(Map<String, Object> byDroneStats) {
        return (Map<String, Object>) byDroneStats.get("overall");
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> byDroneOf(Map<String, Object> byDroneStats) {
        return (Map<String, Object>) byDroneStats.get("by_drone");
    }

    /** Extracts the numeric index from a droneId like "D3" -> 3, for stable console/JSON ordering. */
    private static int droneIndexOf(String droneId) {
        try {
            return Integer.parseInt(droneId.substring(1));
        } catch (NumberFormatException | IndexOutOfBoundsException e) {
            return Integer.MAX_VALUE;
        }
    }

    /** Use the latest telemetry; only pre-takeoff out-of-grid positions use the fallback cell. */
    private PureBigraph droneModelFromRosPositions(int siteCount) throws InvalidConnectionException, TypeNotExistsException, IncompatibleSignatureException, IncompatibleInterfaceException {
        if (!rosUpdateEnabled || dronePositions.isEmpty()) {

            return droneModel(siteCount);
        }

        if (collisionRiskDetected.get()) {
            System.err.println("⚠ Collision risk detected, using previous drone model");
            return dronePart; 
        }

        List<Bigraph<DynamicSignature>> placements = new ArrayList<>();
        for (int i = 0; i < siteCount; i++) {

            if (obstacleGrids.contains(i)) {
                placements.add(buildObstacleCell());
            } else {
                placements.add(emptyOccupiedCell());
            }
        }

        Set<Integer> placedCells = new HashSet<>();
        for (DronePosition pos : dronePositions.values()) {
            int placeIndex = pos.gridIndex;

            if ((placeIndex < 0 || placeIndex >= siteCount) && allowOutOfGridPlacement) {
                DroneStatus st = droneStatuses.get(pos.droneId);
                boolean notTakenOff = (st == null) || !st.hasTakenOff;
                if (notTakenOff) {
                    int fallback = fallbackGridIndex(pos.droneId, siteCount);
                    if (fallback >= 0 && !placedCells.contains(fallback) && !obstacleGrids.contains(fallback)) {
                        System.out.println("  ⚠ " + pos.droneId + " out-of-grid (grid=" + pos.gridIndex
                                + ", sensor drift, pre-takeoff), placing at fallback Grid[" + fallback + "] to allow takeoff");
                        placeIndex = fallback;
                    }
                }
            }

            if (placeIndex >= 0 && placeIndex < siteCount) {

                if (obstacleGrids.contains(placeIndex)) {
                    continue;
                }

                String droneStatus = "Landed";  
                DroneStatus status = droneStatuses.get(pos.droneId);
                if (status != null) {
                    if (status.landingRuleApplied) {
                        droneStatus = "Landed";  
                    } else if (status.takeoffRuleApplied) {
                        droneStatus = "flying";  
                    }
                }

                placements.set(placeIndex,
                        buildDrone(pos.droneId, droneStatus, "OccupiedBy"));
                placedCells.add(placeIndex);
            }
        }

        Bigraph<DynamicSignature> result = placements.stream()
                .reduce(pureLinkings(sig()).identity_e(), accumulator::apply);
        return (PureBigraph) result;
    }

    private int fallbackGridIndex(String droneId, int siteCount) {
        DroneStatus status = droneStatuses.get(droneId);
        if (status != null && status.startGridIndex != null
                && status.startGridIndex >= 0 && status.startGridIndex < siteCount) {
            return status.startGridIndex;
        }
        try {
            int idx = Integer.parseInt(droneId.substring(1));
            if (idx >= 0 && idx < siteCount) {
                return idx;
            }
        } catch (NumberFormatException ignored) {

        }
        return -1;
    }

    private static class DronePosition {
        final String droneId;
        final double x;
        final double y;
        final double z;
        final int gridIndex;

        DronePosition(String droneId, double x, double y, double z, int gridIndex) {
            this.droneId = droneId;
            this.x = x;
            this.y = y;
            this.z = z;
            this.gridIndex = gridIndex;
        }
    }

    private static class DroneStatus {
        final String droneId;
        // Flight state
        volatile double z;
        volatile double takeoffStartZ = Double.NaN;
        volatile String status = "Landed";
        volatile boolean hasTakenOff, takeoffRuleApplied, takeoffCommandSent;
        volatile long takeoffTime;
        volatile boolean landingCommandSent, hasLanded, landingRuleApplied;
        volatile long landingCommandTime;
        // Navigation state
        volatile boolean reachedDestination, isWaiting, isMoving;
        volatile long lastMoveTime, waitingStartTime;
        volatile Integer waitingForGrid, reservedGrid, movingToGrid;
        // Status monitoring
        volatile double batteryVoltage = 4.2;
        volatile int rssi;
        volatile String batteryLevel = "Normal", communicationStatus = "Normal";

        volatile Double injectedBatteryVoltage = null;
        volatile Integer injectedRssi = null;
        volatile boolean batteryRuleApplied, communicationRuleApplied, bigraphBatteryNeedsSync;
        // Emergency
        volatile boolean isInEmergency, waitingForRecharge, emergencyLanded;
        volatile String emergencyType;
        volatile GridPoint originalTarget, emergencyTarget;
        volatile Integer startGridIndex;
        // Deadlock avoidance: grids to temporarily avoid during path planning
        final Set<Integer> avoidGrids = ConcurrentHashMap.newKeySet();
        volatile long avoidGridsSetTime;
        volatile int consecutiveWaitCycles;

        DroneStatus(String droneId) { this.droneId = droneId; }
    }

    private static class GridPoint {
        final double x;
        final double y;
        final double z;
        final int gridIndex;

        GridPoint(double x, double y, double z, int gridIndex) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.gridIndex = gridIndex;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            GridPoint that = (GridPoint) o;
            return gridIndex == that.gridIndex;
        }

        @Override
        public int hashCode() {
            return Integer.hashCode(gridIndex);
        }

        @Override
        public String toString() {
            return String.format("Grid[%d](%.1f,%.1f,%.1f)", gridIndex, x, y, z);
        }
    }

    private static class AStarNode implements Comparable<AStarNode> {
        final GridPoint point;
        final AStarNode parent;
        final double gCost;  
        final double hCost;  
        final double fCost;  // gCost + hCost

        AStarNode(GridPoint point, AStarNode parent, double gCost, double hCost) {
            this.point = point;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
            this.fCost = gCost + hCost;
        }

        @Override
        public int compareTo(AStarNode other) {
            return Double.compare(this.fCost, other.fCost);
        }
    }

    private static class GridReservation {
        final String droneId;           
        final long reservationTime;     

        GridReservation(String droneId, long reservationTime) {
            this.droneId = droneId;
            this.reservationTime = reservationTime;
        }
    }
}
