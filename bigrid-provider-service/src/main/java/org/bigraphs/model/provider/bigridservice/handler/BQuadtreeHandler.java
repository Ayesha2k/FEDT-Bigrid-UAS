package org.bigraphs.model.provider.bigridservice.handler;

import org.bigraphs.framework.core.BigraphFileModelManagement;
import org.bigraphs.framework.core.impl.pure.PureBigraph;
import org.bigraphs.model.provider.base.BLocationModelData;
import org.bigraphs.model.provider.bigridservice.data.request.PointDataWithBoundaryRequest;
import org.bigraphs.model.provider.bigridservice.data.ResponseData_GenerateQuadtree;
import org.bigraphs.model.provider.spatial.bigrid.BLocationModelDataFactory;
import org.bigraphs.model.provider.spatial.bigrid.BiGridProvider;
import org.bigraphs.model.provider.spatial.quadtree.QuadItem;
import org.bigraphs.model.provider.spatial.quadtree.impl.QuadtreeConvert;
import org.bigraphs.model.provider.spatial.quadtree.impl.QuadtreeImpl;
import org.springframework.http.MediaType;
import org.springframework.stereotype.Component;
import org.springframework.web.reactive.function.server.ServerRequest;
import org.springframework.web.reactive.function.server.ServerResponse;
import org.swarmwalker.messages.BiGrid;
import reactor.core.publisher.Mono;

import java.awt.geom.Point2D;
import java.io.ByteArrayOutputStream;
import java.util.ArrayList;
import java.util.Base64;
import java.util.List;
import java.util.Map;

/**
 * @author Dominik Grzelak
 */
@Component
public class BQuadtreeHandler extends ServiceHandlerSupport {

    public Mono<ServerResponse> createBQuadtree(ServerRequest request) {
        String format = request.queryParam("format").orElse("xml");
        final int[] maxTreeDepth = {parseQueryParamAsInt(request, "maxTreeDepth", -1)};
        int maxPointsPerLeaf = parseQueryParamAsInt(request, "maxPointsPerLeaf", 1);
        double marginPoint = parseQueryParamAsDouble(request, "marginPoint", 1.0);
        if (marginPoint <= 0) {
            return ServerResponse.badRequest()
                    .contentType(MediaType.APPLICATION_JSON)
                    .body(Mono.just(Map.of("error", "marginPoint must be > 0")), Map.class);
        }

        Mono<PointDataWithBoundaryRequest> pointDataMono = request.bodyToMono(PointDataWithBoundaryRequest.class);

        return pointDataMono.flatMap(pointBoundaryProduct -> {
            QuadtreeImpl.Boundary boundary = pointBoundaryProduct.getBoundary();
            if (maxTreeDepth[0] == -1) {
                maxTreeDepth[0] = QuadtreeImpl.getMaxTreeDepthFrom(boundary, marginPoint); // choose the limiting axis; never negative
            }

            List<Point2D.Double> pointsOmitted = new ArrayList<>();
            List<Point2D.Double> pointsAdded = new ArrayList<>();
            List<Point2D.Double> pointData = pointBoundaryProduct.getPointData().getPoints();
            QuadtreeImpl quadtree = new QuadtreeImpl(boundary, maxPointsPerLeaf, maxTreeDepth[0]);
            quadtree.setProximityDistance((float) marginPoint);
//            quadtree.addListener(new QuadtreeListener() {
//                @Override

//                    pointsOmitted.add((Point2D.Double) p0);

            if (pointData == null) {
                return ServerResponse.badRequest()
                        .contentType(MediaType.APPLICATION_JSON)
                        .body(Mono.just(Map.of("error", "PointsData is null!")), Map.class);
            }
            pointData.forEach(pt -> {
                if (!quadtree.insert(QuadItem.create(pt))) {
                    pointsOmitted.add(pt);
                } else {
                    pointsAdded.add(pt);
                }
            });

//            int pointCount = pointData.size();
//            int pointCountOmitted = pointsOmitted.size();
            QuadtreeConvert converter = new QuadtreeConvert();
            BLocationModelData bLMD = converter.createBLocationModelDataFromQuadtree(quadtree);

            ResponseData_GenerateQuadtree responseData = new ResponseData_GenerateQuadtree();
            responseData.setPointsAdded(pointsAdded);
            responseData.setPointsOmitted(pointsOmitted);
            try {
                if ("xml".equalsIgnoreCase(format)) {
                    responseData.setMimeType(MediaType.APPLICATION_XML.toString());

                    BiGridProvider provider = new BiGridProvider(bLMD);
                    PureBigraph bigrid = provider.getBigraph();
                    ByteArrayOutputStream textStream = new ByteArrayOutputStream();
                    BigraphFileModelManagement.Store.exportAsInstanceModel(bigrid, textStream);
                    responseData.setContent(textStream.toString());
                } else if ("json".equalsIgnoreCase(format)) {
                    responseData.setMimeType(MediaType.APPLICATION_JSON.toString());

                    String json = BLocationModelDataFactory.toJson(bLMD);
                    responseData.setContent(json);
                } else if ("protobuf".equalsIgnoreCase(format)) {
                    responseData.setMimeType(MediaType.parseMediaType("application/x-protobuf").toString());

                    BiGrid gridMessage = BLocationToBiGridConverter.convert(bLMD);
                    byte[] protoBytes = gridMessage.toByteArray();
                    String base64 = Base64.getEncoder().encodeToString(protoBytes);
                    responseData.setContent(base64);
                }
            } catch (Exception e) {
                return Mono.error(new RuntimeException(e));
            }

            return ServerResponse.ok()
                    .contentType(MediaType.APPLICATION_JSON)
                    .body(Mono.just(responseData), ResponseData_GenerateQuadtree.class);
        });
    }
}
