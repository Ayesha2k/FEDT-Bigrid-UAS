package org.bigraphs.model.provider.bigridservice.util;

import org.bigraphs.spring.data.cdo.CdoServerConnectionString;
import org.bigraphs.spring.data.cdo.CdoTemplate;
import org.bigraphs.spring.data.cdo.SimpleCdoDbFactory;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.function.Function;

public final class CdoTemplateProvider {

  public record Key(String address, String repoPath) {}

  private final ConcurrentMap<Key, CdoTemplate> cache = new ConcurrentHashMap<>();

  public CdoTemplate get(Key key) {
    return cache.computeIfAbsent(key, this::createTemplate);
  }

  private CdoTemplate createTemplate(Key key) {
    var connStr = new CdoServerConnectionString(key.address() + key.repoPath());
    var dbFactory = new SimpleCdoDbFactory(connStr);
    return new CdoTemplate(dbFactory);
  }

  public void remove(Key key) {
    CdoTemplate template = cache.remove(key);
    closeQuietly(template);
  }

  public void closeAll() {
    cache.forEach((k, v) -> closeQuietly(v));
    cache.clear();
  }

  private static void closeQuietly(Object o) {

  }
}
