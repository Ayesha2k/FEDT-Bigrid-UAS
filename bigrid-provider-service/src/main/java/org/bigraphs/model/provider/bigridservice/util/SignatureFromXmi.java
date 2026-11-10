package org.bigraphs.model.provider.bigridservice.util;

import org.bigraphs.framework.core.ControlStatus;
import org.w3c.dom.*;
import javax.xml.XMLConstants;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.bigraphs.framework.core.impl.signature.DynamicSignature;
import org.bigraphs.framework.core.impl.signature.DynamicSignatureBuilder;

import java.io.ByteArrayInputStream;
import java.nio.charset.StandardCharsets;
import java.util.LinkedHashMap;
import java.util.Map;

import static org.bigraphs.framework.core.factory.BigraphFactory.pureSignatureBuilder;

public class SignatureFromXmi {

    /**
     * Build a DefaultDynamicSignature by parsing the XMI/XML string directly.
     * Heuristics:
     *  - each <bChild ... xsi:type="ns:Control"> is a control occurrence
     *  - control name = local part of xsi:type (after ':', or whole if no ':')
     *  - arity(control) = max number of immediate <bPorts> children across occurrences
     *  - status(control) = ATOMIC if no immediate <bChild> children in any occurrence, else ACTIVE
     */
    public static DynamicSignature createSignatureFromXmi(String xmi) throws Exception {
        Document doc = parseXml(xmi);

        // Stats per control
        Map<String, Integer> maxArity = new LinkedHashMap<>();
        Map<String, Boolean> hasImmediateChildren = new LinkedHashMap<>();

        // Walk all elements; whenever we see a <bChild>, treat it as a node occurrence
        traverse(doc.getDocumentElement(), (Element el) -> {
            if (!"bChild".equals(localName(el))) return;

            String control = controlNameFromXsiType(el);
            if (control == null || control.isEmpty()) return;

            int ports = countImmediateChildren(el, "bPorts");
            boolean hasKids = countImmediateChildren(el, "bChild") > 0;

            maxArity.merge(control, ports, Math::max);
            hasImmediateChildren.merge(control, hasKids, (a, b) -> a || b);
        });

        DynamicSignatureBuilder sb = pureSignatureBuilder();
        for (var e : maxArity.entrySet()) {
            String ctrl = e.getKey();
            int arity = e.getValue();
            boolean kids = hasImmediateChildren.getOrDefault(ctrl, false);
            sb.add(ctrl, arity, kids ? ControlStatus.ACTIVE : ControlStatus.ATOMIC);
        }
        return sb.create();
    }

    private static Document parseXml(String xml) throws Exception {
        DocumentBuilderFactory f = DocumentBuilderFactory.newInstance();
        f.setNamespaceAware(true);
        // Secure processing / XXE hardening
        f.setFeature(XMLConstants.FEATURE_SECURE_PROCESSING, true);
        f.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
        f.setFeature("http://xml.org/sax/features/external-general-entities", false);
        f.setFeature("http://xml.org/sax/features/external-parameter-entities", false);
        f.setExpandEntityReferences(false);
        f.setXIncludeAware(false);

        DocumentBuilder b = f.newDocumentBuilder();
        try (ByteArrayInputStream in = new ByteArrayInputStream(xml.getBytes(StandardCharsets.UTF_8))) {
            return b.parse(in);
        }
    }

    private interface ElementVisitor { void visit(Element el); }

    private static void traverse(Node node, ElementVisitor v) {
        if (node == null) return;
        if (node.getNodeType() == Node.ELEMENT_NODE) {
            Element el = (Element) node;
            v.visit(el);
            NodeList children = el.getChildNodes();
            for (int i = 0; i < children.getLength(); i++) {
                traverse(children.item(i), v);
            }
        } else {
            NodeList kids = node.getChildNodes();
            for (int i = 0; i < kids.getLength(); i++) {
                traverse(kids.item(i), v);
            }
        }
    }

    private static String localName(Element el) {
        String ln = el.getLocalName();
        return (ln != null ? ln : el.getNodeName());
    }

    private static String controlNameFromXsiType(Element el) {
        // Try namespace-aware lookup first
        String xsi = "http://www.w3.org/2001/XMLSchema-instance";
        String typeVal = el.getAttributeNS(xsi, "type");
        if (typeVal == null || typeVal.isEmpty()) {
            // Fallback: non-NS attribute access (in case parser didn’t bind it)
            typeVal = el.getAttribute("xsi:type");
        }
        if (typeVal == null || typeVal.isEmpty()) return null;

        int idx = typeVal.indexOf(':');
        return (idx >= 0 ? typeVal.substring(idx + 1) : typeVal).trim();
    }

    private static int countImmediateChildren(Element parent, String wantedLocalName) {
        int count = 0;
        NodeList kids = parent.getChildNodes();
        for (int i = 0; i < kids.getLength(); i++) {
            Node n = kids.item(i);
            if (n.getNodeType() != Node.ELEMENT_NODE) continue;
            Element e = (Element) n;
            if (wantedLocalName.equals(localName(e))) count++;
        }
        return count;
    }
}
