package org.example;

import org.bigraphs.spring.data.cdo.CdoServerConnectionString;
import org.bigraphs.spring.data.cdo.CdoTemplate;
import org.bigraphs.spring.data.cdo.SimpleCdoDbFactory;
import org.bigraphs.spring.data.cdo.repository.config.EnableCdoRepositories;
import org.springframework.boot.autoconfigure.AutoConfigureOrder;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.scheduling.annotation.EnableAsync;

/**
 * @author Dominik Grzelak
 */
@Configuration
@AutoConfigureOrder(2)
@EnableCdoRepositories()
@EnableAsync
public class CDOServerConfig {

    @Bean
    public CdoTemplate cdoTemplate() {
        return new CdoTemplate(new SimpleCdoDbFactory(new CdoServerConnectionString("cdo://localhost:2036/repo1")));
    }
}
