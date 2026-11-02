# P1 验收与演练记录（部署 / 底图 / GeoTIFF / 远程访问）

日期：2026-11-03

## 1. 部署与自检（T10）

```bash
cd gnss_bringup/hmi
./deploy/deploy_and_start_gnss_hmi.sh user@vehicle
```

- rsync → `/opt/gnss-hmi`，远端 venv 安装，安装 `deploy/gnss-hmi.service`，
  `systemctl restart gnss-hmi`，最后 `curl /api/status` 自检；失败自动打印
  `journalctl -u gnss-hmi -n 50`。
- [ ] 换一台车/重装后一条命令可用。
- [ ] `systemctl restart gnss-hmi` 后 5 s 内页面恢复。
- [ ] `Restart=always`、`KillMode=control-group`：HMI 进程被 kill 后 systemd
      会连同其子进程一起重启。
- [ ] `run.log` 轮转与 `status.json` 快照按现场日志策略配置。

## 2. 底图自动维护（T11）

```bash
# 手动执行一次（systemd timer 每周自动跑）
sudo systemctl start gnss-hmi-tiles.service

# 或直接：
python3 -m gnss_hmi.tiles.manager \
  --root /data/gnsslog/hmi/tiles --bbox W,S,E,N --zooms 14-18
```

- 每次构建写入 `versions/<name>-<ts>.mbtiles`，SHA-256 校验后原子替换
  `current.mbtiles`，只保留最近 2 版；失败不替换现网缓存。
- [ ] 源不可用：仍服务上一版，页面不阻塞。
- [ ] 校验失败：启动时自动退到最新完好版本，再退网格。
- [ ] 归属标注与可用源按现场条款确认。

## 3. 矿区 GeoTIFF 导入（T12）

```bash
python3 -m gnss_hmi.tiles.geotiff site.tif --root /data/gnsslog/hmi/tiles \
  --epsg 3857 --name mine-ortho
```

- 仅接受显式 `EPSG:4326` / `EPSG:3857`；GCJ-02 / BD-09 没有权威 EPSG，
  因此会被显式白名单拒绝。
- 导入后原子切换，页面底图版本名随之更新。
- [ ] 控制点叠加偏差 ≤ 0.3 m；压路场景 ≤ 0.5–1 m。

## 4. 远程访问与 UI（T13）

- `web.host=0.0.0.0` + VPN/反向代理示例见 `docs/deploy.md`；不做内置账号。
- [ ] 办公网经 VPN 连续观察 30 min 无断连。
- [ ] 窄屏（≤900 px）：地图 55vh，侧栏可滚动，时间曲线 2 列，关键指标不遮挡。

## 5. 现场演练（T14）

| 演练 | 预期 |
|---|---|
| 拔差分网线 | 链路卡片 3 s 内“差分中断”，恢复后自动清除 |
| 拔 610 网线 | 融合徽标灰化“无数据”，APP 路正常 |
| `systemctl restart gnss-hmi` | 5 s 内页面恢复，轨迹重连 |
| 系统时钟回跳 | 页面时间跟随 node clock；不产生重复事件 |
| 底图版本回滚 | 删除/破坏 `current.mbtiles` 后重启，自动退上一版 |
| 断网重启 | 有缓存显示底图与版本；无缓存显示网格，数据面不阻塞 |

完成后在本文件勾选，并补 `journalctl`/截图路径。
