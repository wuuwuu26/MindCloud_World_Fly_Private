FROM tumgis/3dcitydb-web-map:alpine-v2.0.0

WORKDIR /var/www/google-tiles-flight

COPY --chown=node:node . /var/www/google-tiles-flight

RUN mkdir -p /var/www/google-tiles-flight/asset/gate-paths \
    && chown -R node:node /var/www/google-tiles-flight/asset/gate-paths

EXPOSE 8000

CMD ["node", "/var/www/google-tiles-flight/scripts/server.js"]
