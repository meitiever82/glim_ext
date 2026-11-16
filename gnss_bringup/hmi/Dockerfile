# Dockerfile
FROM python:3.11-slim AS rtkbuild
RUN apt-get update && apt-get install -y --no-install-recommends git build-essential \
    && rm -rf /var/lib/apt/lists/*
# Pinned to a specific demo5 commit (not just the branch tip) for
# reproducible builds: the demo5 branch moves, and matrix builds should not
# silently pick up a newer rtkrcv than what was validated in this repo.
# Sha captured via: git ls-remote https://github.com/rtklibexplorer/RTKLIB.git demo5 | cut -f1
# Re-pin deliberately (and re-run the integration checklist) to move it forward.
ARG RTKLIB_DEMO5_SHA=75a2e56275485b21a67bd35bc94bbeb8936e1a74
RUN git clone https://github.com/rtklibexplorer/RTKLIB.git /rtklib \
    && git -C /rtklib checkout "$RTKLIB_DEMO5_SHA" \
    && make -C /rtklib/app/consapp/rtkrcv/gcc -j"$(nproc)"

FROM python:3.11-slim
WORKDIR /app
COPY pyproject.toml README.md ./
COPY src ./src
RUN pip install --no-cache-dir .
COPY web ./web
COPY config.yaml.example ./config.yaml.example
COPY --from=rtkbuild /rtklib/app/consapp/rtkrcv/gcc/rtkrcv /usr/local/bin/rtkrcv
CMD ["python", "-m", "gnss_hmi.main", "/data/config.yaml"]
