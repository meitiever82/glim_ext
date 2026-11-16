# gnss-hmi 容器镜像。
# 注意:镜像里没有 ROS(rclpy / /opt/ros),因此只覆盖 TCP/CAN 自采集模式;
# ros.enabled: true 的只读模式请用 systemd(deploy/gnss-hmi.service)。
FROM python:3.11-slim AS rtkbuild
RUN apt-get update && apt-get install -y --no-install-recommends git build-essential \
    && rm -rf /var/lib/apt/lists/*
# 版本固定为 RTKLIB-EX v2.5.1(rtklibexplorer 维护,原 demo5 分支):rtkrcv_node 依赖
# 该版本的 -nc 选项,换成 Takasu 原版 2.4.3 会崩溃循环(见 gnss_bringup README
# 「安装 RTKLIB-EX 2.5.1」)。
# 只编 app/consapp/rtkrcv,不产生 librtklib.so,所以运行镜像里不需要 ldconfig。
# 钉 tag 的同时校验 commit sha:tag 若被上游移动,构建直接失败,而不是静默换版本。
# 刷新 sha:git ls-remote https://github.com/rtklibexplorer/RTKLIB.git refs/tags/v2.5.1
ARG RTKLIB_TAG=v2.5.1
ARG RTKLIB_SHA=62d4677ed8425a4e2748c6d390b500d1afb493fc
RUN git clone --depth 1 --branch "$RTKLIB_TAG" https://github.com/rtklibexplorer/RTKLIB.git /rtklib \
    && test "$(git -C /rtklib rev-parse HEAD)" = "$RTKLIB_SHA" \
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
