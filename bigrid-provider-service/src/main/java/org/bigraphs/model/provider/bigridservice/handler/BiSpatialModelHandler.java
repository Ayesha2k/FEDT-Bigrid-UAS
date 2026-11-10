package org.bigraphs.model.provider.bigridservice.handler;

import lombok.Synchronized;
import org.bigraphs.framework.core.*;
import org.bigraphs.framework.core.impl.pure.PureBigraph;
import org.bigraphs.framework.core.impl.signature.DynamicSignature;
import org.bigraphs.framework.core.utils.BigraphUtil;
import org.bigraphs.framework.core.utils.emf.EMFUtils;
import org.bigraphs.model.provider.base.BLocationModelData;
import org.bigraphs.model.provider.bigridservice.data.request.ConvexShapeRequest;
import org.bigraphs.model.provider.bigridservice.data.request.GridSpecRequest;
import org.bigraphs.model.provider.bigridservice.data.request.GridSpec3DRequest;
import org.bigraphs.model.provider.bigridservice.data.ResponseData_GenerateGrid;
import org.bigraphs.model.provider.bigridservice.data.request.InterpolationRequest;
import org.bigraphs.model.provider.bigridservice.util.CdoTemplateProvider;
import org.bigraphs.model.provider.bigridservice.util.EcoreXmiUtil;
import org.bigraphs.model.provider.bigridservice.util.SignatureFromXmi;
import org.bigraphs.model.provider.spatial.bigrid.DiagonalDirectionalBiGridProvider;
import org.bigraphs.model.provider.spatial.bigrid.ThreeDimensionalBiGridProvider;
import org.bigraphs.model.provider.spatial.signature.DiagonalDirectionalBiSpaceSignatureProvider;
import org.bigraphs.model.provider.spatial.signature.ThreeDimensionalBiSpaceSignatureProvider;
import org.bigraphs.model.provider.spatial.bigrid.*;
import org.bigraphs.model.provider.spatial.signature.BiSpaceSignatureProvider;
import org.bigraphs.model.provider.spatial.signature.DirectionalBiSpaceSignatureProvider;
import org.bigraphs.spring.data.cdo.CdoTemplate;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.springframework.http.MediaType;
import org.springframework.stereotype.Component;
import org.springframework.web.reactive.function.server.ServerRequest;
import org.springframework.web.reactive.function.server.ServerResponse;
import org.swarmwalker.messages.BiGrid;
import reactor.core.publisher.Mono;

import java.awt.geom.Point2D;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.*;
import java.util.stream.Collectors;

import static org.bigraphs.framework.core.factory.BigraphFactory.*;

/**
 * @author Dominik Grzelak
 */
@Component
public class BiSpatialModelHandler extends ServiceHandlerSupport {

    protected final CdoTemplateProvider cdoTemplateProvider = new CdoTemplateProvider();

    // Bi-spatial Metamodel

    public Mono<ServerResponse> getOrCreateBigraphMetaModel(ServerRequest request) {
        String format = request.queryParam("format").orElse("xml");

        try {
            // Step 1: Get signature
            DynamicSignature signature = BiSpaceSignatureProvider.getInstance().getSignature();

            // Step 2: Format and respond
            if ("xml".equalsIgnoreCase(format)) {
                ByteArrayOutputStream xmlStream = new ByteArrayOutputStream();
                BigraphFileModelManagement.Store.exportAsMetaModel(pureBuilder(signature).create(), xmlStream);
                String xmlOutput = xmlStream.toString();

                return ServerResponse.ok()
                        .contentType(MediaType.APPLICATION_XML)
                        .bodyValue(xmlOutput);
            } else {
                // ToDo
                String jsonLike = "ToDo: metaModel.toString()";
                return ServerResponse.ok()
                        .contentType(MediaType.APPLICATION_JSON)
                        .bodyValue(jsonLike);
            }
        } catch (Exception e) {
            return Mono.error(new RuntimeException("Failed to create/get bigraph meta-model", e));
        }
    }

    // Bi-spatial Bi-grid

    public Mono<ServerResponse> createUniformBigrid(ServerRequest request) {
        Mono<GridSpecRequest> gridSpecMono = request.bodyToMono(GridSpecRequest.class);
        String format = request.queryParam("format").orElse("xml");
        int rows = parseQueryParamAsInt(request, "rows", 3);
        int cols = parseQueryParamAsInt(request, "cols", 3);

        return gridSpecMono.flatMap(gridSpec -> {
            float resFactor;
            if (gridSpec.stepSizeX == gridSpec.stepSizeY) {
                resFactor = gridSpec.stepSizeX;
            } else {
                // Diagonal (i.e., the longest "side")
                resFactor = (float) Math.sqrt(gridSpec.stepSizeX * gridSpec.stepSizeX + gridSpec.stepSizeY * gridSpec.stepSizeY);
            }
            return getServerResponseMono(format, rows, cols, resFactor, gridSpec);
        });
    }

    public Mono<ServerResponse> createUniformBigridDefault(ServerRequest request) {
        String format = request.queryParam("format").orElse("xml");
        int rows = parseQueryParamAsInt(request, "rows", 3);
        int cols = parseQueryParamAsInt(request, "cols", 3);
        float resFactor = 1f;
        GridSpecRequest gridSpec = new GridSpecRequest(0, 0, resFactor, resFactor);
        return getServerResponseMono(format, rows, cols, resFactor, gridSpec);
    }

    private Mono<ServerResponse> getServerResponseMono(String format, int rows, int cols, float resFactor, GridSpecRequest gridSpec) {
        BLocationModelData bLMD = BLocationModelDataFactory.createGrid(rows, cols,
                gridSpec.x, gridSpec.y, gridSpec.stepSizeX, gridSpec.stepSizeY);

        ResponseData_GenerateGrid response = new ResponseData_GenerateGrid();
        response.setCols(cols);
        response.setRows(rows);
        response.setResolutionFactor(resFactor);

        try {
            if ("xml".equalsIgnoreCase(format)) {
                response.setMimeType(MediaType.APPLICATION_XML.toString());
                BiGridProvider provider = new BiGridProvider(bLMD);
                PureBigraph bigrid = provider.getBigraph();
                ByteArrayOutputStream textStream = new ByteArrayOutputStream();
                BigraphFileModelManagement.Store.exportAsInstanceModel(bigrid, textStream);
                response.setContent(textStream.toString());
            } else if ("json".equalsIgnoreCase(format)) {
                response.setMimeType(MediaType.APPLICATION_JSON.toString());
                String json = BLocationModelDataFactory.toJson(bLMD);
                response.setContent(json);
            } else if ("protobuf".equalsIgnoreCase(format)) {
                response.setMimeType(MediaType.parseMediaType("application/x-protobuf").toString());
                BiGrid gridMessage = BLocationToBiGridConverter.convert(bLMD);
                // (!) Important: Encoding Protobuf payload to Base64
                // Because Protobuf is binary and putting it directly in a JSON string field will corrupt the data
                byte[] protoBytes = gridMessage.toByteArray();
                String base64 = Base64.getEncoder().encodeToString(protoBytes);
                response.setContent(base64);
            }
        } catch (Exception e) {
            return Mono.error(new RuntimeException(e));
        }

        return ServerResponse.ok()
                .contentType(MediaType.APPLICATION_JSON)
                .body(Mono.just(response), ResponseData_GenerateGrid.class);
    }

    // Directional Bi-spatial Bi-grid (with LeftRoute, RightRoute, ForwardRoute, BackRoute)

    /**
     * Creates a directional bigrid with POST request containing grid specifications.
     * Each locale has directional routes (LeftRoute, RightRoute, ForwardRoute, BackRoute)
     * based on its position in the grid.
     */
    public Mono<ServerResponse> createDirectionalBigrid(ServerRequest request) {
        Mono<GridSpecRequest> gridSpecMono = request.bodyToMono(GridSpecRequest.class);
        String format = request.queryParam("format").orElse("xml");
        int rows = parseQueryParamAsInt(request, "rows", 3);
        int cols = parseQueryParamAsInt(request, "cols", 3);

        return gridSpecMono.flatMap(gridSpec -> {
            float resFactor;
            if (gridSpec.stepSizeX == gridSpec.stepSizeY) {
                resFactor = gridSpec.stepSizeX;
            } else {
                resFactor = (float) Math.sqrt(gridSpec.stepSizeX * gridSpec.stepSizeX + gridSpec.stepSizeY * gridSpec.stepSizeY);
            }
            return getDirectionalServerResponseMono(format, rows, cols, resFactor, gridSpec);
        });
    }

    /**
     * Creates a directional bigrid with default parameters using GET request.
     * Default: 3x3 grid with step size 1.0, origin at (0,0).
     */
    public Mono<ServerResponse> createDirectionalBigridDefault(ServerRequest request) {
        String format = request.queryParam("format").orElse("xml");
        int rows = parseQueryParamAsInt(request, "rows", 3);
        int cols = parseQueryParamAsInt(request, "cols", 3);
        float resFactor = 1f;
        GridSpecRequest gridSpec = new GridSpecRequest(0, 0, resFactor, resFactor);
        return getDirectionalServerResponseMono(format, rows, cols, resFactor, gridSpec);
    }

    /**
     * Get the directional bigraph metamodel.
     */
    public Mono<ServerResponse> getOrCreateDirectionalBigraphMetaModel(ServerRequest request) {
        String format = request.queryParam("format").orElse("xml");

        try {
            DynamicSignature signature = DirectionalBiSpaceSignatureProvider.getInstance().getSignature();

            if ("xml".equalsIgnoreCase(format)) {
                ByteArrayOutputStream xmlStream = new ByteArrayOutputStream();
                BigraphFileModelManagement.Store.exportAsMetaModel(pureBuilder(signature).create(), xmlStream);
                String xmlOutput = xmlStream.toString();

                return ServerResponse.ok()
                        .contentType(MediaType.APPLICATION_XML)
                        .bodyValue(xmlOutput);
            } else {
                String jsonLike = "ToDo: metaModel.toString()";
                return ServerResponse.ok()
                        .contentType(MediaType.APPLICATION_JSON)
                        .bodyValue(jsonLike);
            }
        } catch (Exception e) {
            return Mono.error(new RuntimeException("Failed to create/get directional bigraph meta-model", e));
        }
    }

    private Mono<ServerResponse> getDirectionalServerResponseMono(String format, int rows, int cols, float resFactor, GridSpecRequest gridSpec) {
        BLocationModelData bLMD = BLocationModelDataFactory.createGrid(rows, cols,
                gridSpec.x, gridSpec.y, gridSpec.stepSizeX, gridSpec.stepSizeY);

        ResponseData_GenerateGrid response = new ResponseData_GenerateGrid();
        response.setCols(cols);
        response.setRows(rows);
        response.setResolutionFactor(resFactor);

        try {
            if ("xml".equalsIgnoreCase(format)) {
                response.setMimeType(MediaType.APPLICATION_XML.toString());
                DirectionalBiGridProvider provider = new DirectionalBiGridProvider(bLMD, rows, cols);
                PureBigraph bigrid = provider.getBigraph();
                ByteArrayOutputStream textStream = new ByteArrayOutputStream();
                BigraphFileModelManagement.Store.exportAsInstanceModel(bigrid, textStream);
                response.setContent(textStream.toString());
            } else if ("json".equalsIgnoreCase(format)) {
                response.setMimeType(MediaType.APPLICATION_JSON.toString());
                String json = BLocationModelDataFactory.toJson(bLMD);
                response.setContent(json);
            } else if ("protobuf".equalsIgnoreCase(format)) {
                // Note: Protobuf format for directional bigrid is not yet supported
                // as the BiGrid message schema would need to be updated to include directional routes
                response.setMimeType(MediaType.parseMediaType("application/x-protobuf").toString());
                response.setContent("Protobuf format not yet supported for directional bigrid. Please use xml or json format.");
            }
        } catch (Exception e) {
            return Mono.error(new RuntimeException(e));
        }

        return ServerResponse.ok()
                .contentType(MediaType.APPLICATION_JSON)
                .body(Mono.just(response), ResponseData_GenerateGrid.class);
    }

    // Diagonal Directional Bi-Spatial Bigrid (with 8-directional routes)

    /**
     * Creates a diagonal directional bigrid with POST request containing grid specifications.
     * Each locale has 8-directional routes (cardinal + diagonal directions) based on its position in the grid.
     */
    public Mono<ServerResponse> createDiagonalDirectionalBigrid(ServerRequest request) {
        Mono<GridSpecRequest> gridSpecMono = request.bodyToMono(GridSpecRequest.class);
        String format = request.queryParam("format").orElse("xml");
        int rows = parseQueryParamAsInt(request, "rows", 3);
        int cols = parseQueryParamAsInt(request, "cols", 3);

        return gridSpecMono.flatMap(gridSpec -> {
            float resFactor;
            if (gridSpec.stepSizeX == gridSpec.stepSizeY) {
                resFactor = gridSpec.stepSizeX;
            } else {
                resFactor = (float) Math.sqrt(gridSpec.stepSizeX * gridSpec.stepSizeX + gridSpec.stepSizeY * gridSpec.stepSizeY);
            }
            return getDiagonalDirectionalServerResponseMono(format, rows, cols, resFactor, gridSpec);
        });
    }

    /**
     * Creates a diagonal directional bigrid with default parameters using GET request.
     * Default: 3x3 grid with step size 1.0, origin at (0,0).
     */
    public Mono<ServerResponse> createDiagonalDirectionalBigridDefault(ServerRequest request) {
        String format = request.queryParam("format").orElse("xml");
        int rows = parseQueryParamAsInt(request, "rows", 3);
        int cols = parseQueryParamAsInt(request, "cols", 3);
        float resFactor = 1f;
        GridSpecRequest gridSpec = new GridSpecRequest(0, 0, resFactor, resFactor);
        return getDiagonalDirectionalServerResponseMono(format, rows, cols, resFactor, gridSpec);
    }

    /**
     * Get the diagonal directional bigraph metamodel.
     */
    public Mono<ServerResponse> getOrCreateDiagonalDirectionalBigraphMetaModel(ServerRequest request) {
        String format = request.queryParam("format").orElse("xml");

        try {
            DynamicSignature signature = DiagonalDirectionalBiSpaceSignatureProvider.getInstance().getSignature();

            if ("xml".equalsIgnoreCase(format)) {
                ByteArrayOutputStream xmlStream = new ByteArrayOutputStream();
                BigraphFileModelManagement.Store.exportAsMetaModel(pureBuilder(signature).create(), xmlStream);
                String xmlOutput = xmlStream.toString();

                return ServerResponse.ok()
                        .contentType(MediaType.APPLICATION_XML)
                        .bodyValue(xmlOutput);
            } else {
                String jsonLike = "ToDo: metaModel.toString()";
                return ServerResponse.ok()
                        .contentType(MediaType.APPLICATION_JSON)
                        .bodyValue(jsonLike);
            }
        } catch (Exception e) {
            return Mono.error(new RuntimeException("Failed to create/get diagonal directional bigraph meta-model", e));
        }
    }

    private Mono<ServerResponse> getDiagonalDirectionalServerResponseMono(String format, int rows, int cols, float resFactor, GridSpecRequest gridSpec) {
        BLocationModelData bLMD = BLocationModelDataFactory.createGrid(rows, cols,
                gridSpec.x, gridSpec.y, gridSpec.stepSizeX, gridSpec.stepSizeY);

        ResponseData_GenerateGrid response = new ResponseData_GenerateGrid();
        response.setCols(cols);
        response.setRows(rows);
        response.setResolutionFactor(resFactor);

        try {
            if ("xml".equalsIgnoreCase(format)) {
                response.setMimeType(MediaType.APPLICATION_XML.toString());
                DiagonalDirectionalBiGridProvider provider = new DiagonalDirectionalBiGridProvider(bLMD, rows, cols);
                PureBigraph bigrid = provider.getBigraph();
                ByteArrayOutputStream textStream = new ByteArrayOutputStream();
                BigraphFileModelManagement.Store.exportAsInstanceModel(bigrid, textStream);
                response.setContent(textStream.toString());
            } else if ("json".equalsIgnoreCase(format)) {
                response.setMimeType(MediaType.APPLICATION_JSON.toString());
                String json = BLocationModelDataFactory.toJson(bLMD);
                response.setContent(json);
            } else if ("protobuf".equalsIgnoreCase(format)) {
                // Note: Protobuf format for diagonal directional bigrid is not yet supported
                response.setMimeType(MediaType.parseMediaType("application/x-protobuf").toString());
                response.setContent("Protobuf format not yet supported for diagonal directional bigrid. Please use xml or json format.");
            }
        } catch (Exception e) {
            return Mono.error(new RuntimeException(e));
        }

        return ServerResponse.ok()
                .contentType(MediaType.APPLICATION_JSON)
                .body(Mono.just(response), ResponseData_GenerateGrid.class);
    }

    // Three-Dimensional Bi-Spatial Bigrid (with 10-directional routes including vertical)

    /**
     * Creates a 3D bigrid with POST request containing 3D grid specifications.
     * Multiple layers of 2D grids connected vertically with UpRoute and DownRoute.
     */
    public Mono<ServerResponse> createThreeDimensionalBigrid(ServerRequest request) {
        Mono<GridSpec3DRequest> gridSpecMono = request.bodyToMono(GridSpec3DRequest.class);
        String format = request.queryParam("format").orElse("xml");
        int rows = parseQueryParamAsInt(request, "rows", 3);
        int cols = parseQueryParamAsInt(request, "cols", 3);
        int layers = parseQueryParamAsInt(request, "layers", 3);

        return gridSpecMono.flatMap(gridSpec -> {
            float resFactor;
            if (gridSpec.stepSizeX == gridSpec.stepSizeY) {
                resFactor = gridSpec.stepSizeX;
            } else {
                resFactor = (float) Math.sqrt(gridSpec.stepSizeX * gridSpec.stepSizeX + gridSpec.stepSizeY * gridSpec.stepSizeY);
            }
            return getThreeDimensionalServerResponseMono(format, rows, cols, layers, resFactor, gridSpec);
        });
    }

    /**
     * Creates a 3D bigrid with default parameters using GET request.
     * Default: 3x3x3 grid with step size 1.0, layer height 1.0, origin at (0,0,0).
     */
    public Mono<ServerResponse> createThreeDimensionalBigridDefault(ServerRequest request) {
        String format = request.queryParam("format").orElse("xml");
        int rows = parseQueryParamAsInt(request, "rows", 3);
        int cols = parseQueryParamAsInt(request, "cols", 3);
        int layers = parseQueryParamAsInt(request, "layers", 3);
        float resFactor = 1f;
        GridSpec3DRequest gridSpec = new GridSpec3DRequest(0, 0, 0, resFactor, resFactor, resFactor);
        return getThreeDimensionalServerResponseMono(format, rows, cols, layers, resFactor, gridSpec);
    }

    /**
     * Get the 3D bigraph metamodel.
     */
    public Mono<ServerResponse> getOrCreateThreeDimensionalBigraphMetaModel(ServerRequest request) {
        String format = request.queryParam("format").orElse("xml");

        try {
            DynamicSignature signature = ThreeDimensionalBiSpaceSignatureProvider.getInstance().getSignature();

            if ("xml".equalsIgnoreCase(format)) {
                ByteArrayOutputStream xmlStream = new ByteArrayOutputStream();
                BigraphFileModelManagement.Store.exportAsMetaModel(pureBuilder(signature).create(), xmlStream);
                String xmlOutput = xmlStream.toString();

                return ServerResponse.ok()
                        .contentType(MediaType.APPLICATION_XML)
                        .bodyValue(xmlOutput);
            } else {
                String jsonLike = "ToDo: metaModel.toString()";
                return ServerResponse.ok()
                        .contentType(MediaType.APPLICATION_JSON)
                        .bodyValue(jsonLike);
            }
        } catch (Exception e) {
            return Mono.error(new RuntimeException("Failed to create/get 3D bigraph meta-model", e));
        }
    }

    private Mono<ServerResponse> getThreeDimensionalServerResponseMono(String format, int rows, int cols, int layers, float resFactor, GridSpec3DRequest gridSpec) {
        BLocationModelData bLMD = BLocationModelDataFactory.createGrid(rows, cols,
                gridSpec.x, gridSpec.y, gridSpec.stepSizeX, gridSpec.stepSizeY);

        ResponseData_GenerateGrid response = new ResponseData_GenerateGrid();
        response.setCols(cols);
        response.setRows(rows);
        response.setResolutionFactor(resFactor);

        try {
            if ("xml".equalsIgnoreCase(format)) {
                response.setMimeType(MediaType.APPLICATION_XML.toString());
                ThreeDimensionalBiGridProvider provider = new ThreeDimensionalBiGridProvider(
                        bLMD, rows, cols, layers,
                        gridSpec.x, gridSpec.y, gridSpec.z,
                        gridSpec.stepSizeX, gridSpec.stepSizeY, gridSpec.layerHeight);
                PureBigraph bigrid = provider.getBigraph();
                ByteArrayOutputStream textStream = new ByteArrayOutputStream();
                BigraphFileModelManagement.Store.exportAsInstanceModel(bigrid, textStream);
                response.setContent(textStream.toString());
            } else if ("json".equalsIgnoreCase(format)) {
                response.setMimeType(MediaType.APPLICATION_JSON.toString());
                String json = BLocationModelDataFactory.toJson(bLMD);
                response.setContent(json);
            } else if ("protobuf".equalsIgnoreCase(format)) {
                // Note: Protobuf format for 3D bigrid is not yet supported
                response.setMimeType(MediaType.parseMediaType("application/x-protobuf").toString());
                response.setContent("Protobuf format not yet supported for 3D bigrid. Please use xml or json format.");
            }
        } catch (Exception e) {
            return Mono.error(new RuntimeException(e));
        }

        return ServerResponse.ok()
                .contentType(MediaType.APPLICATION_JSON)
                .body(Mono.just(response), ResponseData_GenerateGrid.class);
    }

    // Bi-spatial Structures as Convex Shapes

    public Mono<ServerResponse> createConvexShape(ServerRequest request) {
        String format = request.queryParam("format").orElse("xml");

        Mono<ConvexShapeRequest> bodyMono = request.bodyToMono(ConvexShapeRequest.class);

        return bodyMono.flatMap(convexRequest -> {
            ResponseData_GenerateGrid response = new ResponseData_GenerateGrid();
            try {
                List<Point2D.Float> pointList = convexRequest.getPoints();
                float stepSize = convexRequest.getStepSize();
                PureBigraph bigraph = ConvexShapeBuilder.generateSingleRoot(
                        pointList, stepSize, 0.0f, BiGridElementFactory.create()
                );

                if ("xml".equalsIgnoreCase(format)) {
                    response.setMimeType(MediaType.APPLICATION_XML.toString());

                    ByteArrayOutputStream textStream = new ByteArrayOutputStream();
                    BigraphFileModelManagement.Store.exportAsInstanceModel(bigraph, textStream);
                    response.setContent(textStream.toString());
                } else if ("json".equalsIgnoreCase(format)) {
                    response.setMimeType(MediaType.APPLICATION_JSON.toString());

                    // ToDo
                    String json = "ToDo: BLocationModelDataFactory.toJson(bigraph);";
                    response.setContent(json);
                } else if ("protobuf".equalsIgnoreCase(format)) {
                    response.setMimeType(MediaType.parseMediaType("application/x-protobuf").toString());

                    BiGrid gridMessage = BLocationToBiGridConverter.convertPureSpatialBigraph(bigraph, stepSize, stepSize);
                    byte[] protoBytes = gridMessage.toByteArray();
                    String base64 = Base64.getEncoder().encodeToString(protoBytes);
                    response.setContent(base64);
                }
            } catch (Exception e) {
                return Mono.error(new RuntimeException("Failed to generate convex bigraph", e));
            }
            return ServerResponse.ok()
                    .contentType(MediaType.APPLICATION_JSON)
                    .body(Mono.just(response), ResponseData_GenerateGrid.class);
        });
    }

    // Bi-spatial Structures by Interpolation

    public Mono<ServerResponse> createInterpolatedBigraph(ServerRequest request) {
        Mono<InterpolationRequest> inputMono = request.bodyToMono(InterpolationRequest.class);
        String format = request.queryParam("format").orElse("xml");

        return inputMono.flatMap(input -> {
            ResponseData_GenerateGrid response = new ResponseData_GenerateGrid();
            try {
                List<Point2D.Float> pointList = input.points.stream()
                        .map(p -> new Point2D.Float(p.x, p.y))
                        .collect(Collectors.toList());

                PureBigraph bigraph = LinearInterpolationBuilder.generate(
                        pointList, input.stepSizeX, input.stepSizeY
                );

                if ("xml".equalsIgnoreCase(format)) {
                    response.setMimeType(MediaType.APPLICATION_XML.toString());

                    ByteArrayOutputStream textStream = new ByteArrayOutputStream();
                    BigraphFileModelManagement.Store.exportAsInstanceModel(bigraph, textStream);
                    response.setContent(textStream.toString());

                } else if ("json".equalsIgnoreCase(format)) {
                    response.setMimeType(MediaType.APPLICATION_JSON.toString());

                    // ToDo
                    String json = "ToDo: BLocationModelDataFactory.toJson(bigraph);";
                    response.setContent(json);

                } else if ("protobuf".equalsIgnoreCase(format)) {
                    response.setMimeType(MediaType.parseMediaType("application/x-protobuf").toString());

                    BiGrid gridMessage = BLocationToBiGridConverter.convertPureSpatialBigraph(bigraph, input.stepSizeX, input.stepSizeY);
                    byte[] protoBytes = gridMessage.toByteArray();
                    String base64 = Base64.getEncoder().encodeToString(protoBytes);
                    response.setContent(base64);
                }
            } catch (Exception e) {
                return Mono.error(new RuntimeException("Failed to interpolate bigraph", e));
            }
            return ServerResponse.ok()
                    .contentType(MediaType.APPLICATION_JSON)
                    .body(Mono.just(response), ResponseData_GenerateGrid.class);
        });
    }

    @Synchronized
    public Mono<ServerResponse> fetchFromCDO(ServerRequest request) {
        String address = request.queryParam("address").orElse("cdo://127.0.0.1:2036/repo1"); //TODO: update CDO template
        String repoPath = request.queryParam("repopath").orElse("/topo");
        String stepSizeX = request.queryParam("stepSizeX").orElse("1.0");
        String stepSizeY = request.queryParam("stepSizeY").orElse("1.0");
        String resFactorArg = request.queryParam("scaleFactor").orElse("-1.0");
        String format = request.queryParam("format").orElse("xml");
        ResponseData_GenerateGrid response = new ResponseData_GenerateGrid();
        float resFactor = Float.parseFloat(resFactorArg);
        if (resFactor <= 0)
            resFactor = (float) Math.sqrt(Float.parseFloat(stepSizeX) * Float.parseFloat(stepSizeX) + Float.parseFloat(stepSizeY) * Float.parseFloat(stepSizeY));
        response.setResolutionFactor(resFactor);
        response.setRows(-1);
        response.setCols(-1);

        var key = new CdoTemplateProvider.Key(address, repoPath);
        CdoTemplate cdoTemplate = cdoTemplateProvider.get(key);
        List<EObject> all = cdoTemplate.findAll(EObject.class, repoPath);
        EObject eObject = all.getLast();

        EObject copy = EcoreUtil.copy(EMFUtils.getRootContainer(eObject));
        try {
            String xmiString = EcoreXmiUtil.toXmiString(copy);
            DynamicSignature signatureFromXmi = SignatureFromXmi.createSignatureFromXmi(xmiString);
            PureBigraph bigraphLoaded = BigraphUtil.toBigraph(copy.eClass().getEPackage(), copy, signatureFromXmi);

            if ("xml".equalsIgnoreCase(format)) {
                response.setMimeType(MediaType.APPLICATION_XML.toString());

                ByteArrayOutputStream textStream = new ByteArrayOutputStream();
                BigraphFileModelManagement.Store.exportAsInstanceModel(bigraphLoaded, textStream);
                response.setContent(textStream.toString());
                try {
                    textStream.close();
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            } else if ("json".equalsIgnoreCase(format)) {
                response.setMimeType(MediaType.APPLICATION_JSON.toString());

                // ToDo
                String json = "ToDo: BLocationModelDataFactory.toJson(bigraph);";
                response.setContent(json);
            } else if ("protobuf".equalsIgnoreCase(format)) {
                response.setMimeType(MediaType.parseMediaType("application/x-protobuf").toString());

                BiGrid gridMessage = BLocationToBiGridConverter.convertPureSpatialBigraph(bigraphLoaded, Float.parseFloat(stepSizeX), Float.parseFloat(stepSizeY));
                byte[] protoBytes = gridMessage.toByteArray();
                String base64 = Base64.getEncoder().encodeToString(protoBytes);
                response.setContent(base64);
            }

            return ServerResponse.ok()
                    .contentType(MediaType.APPLICATION_JSON)
                    .body(Mono.just(response), ResponseData_GenerateGrid.class);
        } catch (Exception e) {
            return ServerResponse.badRequest().bodyValue(e.toString());
        }
    }
}
