FROM tumgis/3dcitydb-web-map:alpine-v2.0.0

WORKDIR /var/www/mindcloud

COPY --chown=node:node . /var/www/mindcloud

RUN mkdir -p /var/www/mindcloud/asset/gate-paths \
    && chown -R node:node /var/www/mindcloud/asset/gate-paths

EXPOSE 8000

CMD ["node", "/var/www/mindcloud/scripts/server.js"]
