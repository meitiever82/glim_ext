# Foxglove 旁路（可选，T18）

Foxglove 只作为 ROS 原生调试旁路，不替代只读 HMI。

```bash
source /opt/ros/humble/setup.bash
ros2 run foxglove_bridge foxglove_bridge \
  --ros-args -p port:=8765 -p address:=0.0.0.0
```

- 浏览器打开 `https://app.foxglove.dev`，连接 `ws://<车端IP>:8765`。
- 推荐面板：Raw Messages（`/rtkrcv_node/rtk_fix`、`/gnss_cgi610/rtk_fix`）、
  Plot（quality/sats/ratio）、Diagnostics（`/gnss/diagnostics`）。
- `foxglove_bridge` 是独立进程，HMI 进程不受影响；两者都只读或只订阅，
  不参与解算。
- 记得把 `8765` 纳入现场网络白名单（与 HMI 的 8080 同级别）。
