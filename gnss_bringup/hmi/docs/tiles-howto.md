# MBTiles 卫星影像集成指南

## 概述

rtk-monitor 支持离线卫星影像作为地图底图，格式为 MBTiles（标准瓦片架构）。矿区和野外作业场景下，离线影像避免了网络依赖和数据泄露风险。本指南介绍三条路线，将矿区影像转换为 MBTiles 格式并集成到应用中。

---

## 1. 文件格式与坐标系要求

### 1.1 MBTiles 格式说明

MBTiles 是一种基于 SQLite 的瓦片容器格式，标准结构如下：

- 单个 `.mbtiles` 文件
- 包含金字塔层级瓦片（zoom level 0~28）
- 支持栅格（PNG/JPG）和矢量（PBF）两种瓦片
- 适配 Leaflet 等主流 Web GIS 库
- **TMS 行号翻转**：TileStore 处理 XYZ 请求时内部进行 TMS 行号翻转（row = 2^z-1-y），与标准 MBTiles 输出一致；仅手工编写转换器时需注意此细节

### 1.2 必须的坐标系：EPSG:3857

**重要**：所有导入的影像必须投影到 **Web Mercator** 坐标系（EPSG:3857）。这是 Leaflet 与大多数 Web 地图库的标准假设。

- 若源影像为地理坐标系（EPSG:4326 WGS84），需先重投影
- 若源影像为其他投影系，需先转换到 EPSG:4326，再到 EPSG:3857

---

## 2. 转换路线 A：基于已有 GeoTIFF 正射影像

**适用场景**：已有经过正射影像处理（正确配准与投影）的 GeoTIFF 文件（如航测成果、无人机成图）

### 2.1 前置环境

安装 GDAL 工具套件（包含 `gdal_translate` 和 `gdaladdo`）：

**Ubuntu/Debian**：
```bash
sudo apt-get install gdal-bin
```

**macOS**（Homebrew）：
```bash
brew install gdal
```

**验证安装**：
```bash
gdal_translate --version
gdalinfo --version
```

### 2.2 转换步骤

假设源文件为 `ortho.tif`（已在 EPSG:3857 或可通过下文 warp 转换）。

#### 步骤 1：检查源影像坐标系

```bash
gdalinfo ortho.tif | grep -A 2 "Coordinate System"
```

**预期输出示例**（EPSG:3857）：
```
Coordinate System is:
PROJCS["WGS 84 / Pseudo-Mercator",
    GEOGCS["WGS 84",
```

如输出显示不是 EPSG:3857，先进行重投影：

```bash
gdalwarp -t_srs EPSG:3857 ortho.tif ortho_3857.tif
```

#### 步骤 2：转换为 MBTiles

```bash
gdal_translate -of MBTILES ortho.tif mine.mbtiles
```

**参数说明**：
- `-of MBTILES` — 输出格式为 MBTiles
- `ortho.tif` — 源 GeoTIFF 文件
- `mine.mbtiles` — 输出文件名

**输出示例**：
```
Input file size is 4096, 4096
Output file size is 4096, 4096
```

#### 步骤 3：生成瓦片金字塔（可选但推荐）

```bash
gdaladdo mine.mbtiles 2 4 8 16
```

**说明**：
- `gdaladdo` 生成多层级缩小版本（下采样）
- 参数 `2 4 8 16` 表示创建 1/2、1/4、1/8、1/16 分辨率的缩小版
- 大幅加快 Leaflet 加载速度，尤其在高缩放级别

**耗时参考**：
- 大小 500 MB 的 GeoTIFF → 约 2-5 分钟

### 2.3 验证生成的 MBTiles

```bash
sqlite3 mine.mbtiles ".tables"
```

**预期输出**（至少包含）：
```
metadata  tiles
```

查看元数据：
```bash
sqlite3 mine.mbtiles "SELECT * FROM metadata;"
```

**预期包含字段**（示例）：
| 字段 | 示例值 |
|---|---|
| `name` | `mine` |
| `format` | `png` |
| `bounds` | `-180,-85.05112878,180,85.05112878` |
| `minzoom` | `0` |
| `maxzoom` | `28` |

---

## 3. 转换路线 B：基于 QGIS 导出的 XYZ 瓦片

**适用场景**：已通过 QGIS 将栅格图层导出为 XYZ 瓦片目录结构

### 3.1 前置环境

- QGIS（任何版本，建议 3.20+）
- `mb-util` 工具（Python 工具）

**安装 mb-util**：
```bash
pip install mb-util
```

**验证**：
```bash
mb-util --version
```

### 3.2 QGIS 导出瓦片（如未完成）

1. 在 QGIS 中加载栅格图层（GeoTIFF、JPG+VRT 等）
2. 右键图层 → **Export → Export as Image...**（或通过插件 `MapTiler` 导出）
3. 指定输出目录（如 `~/tiles/`）
4. 格式选择 **XYZ Tiles** 或 **TMS**
5. 设置最小/最大缩放级别（推荐 0-18）
6. 导出

**输出结构示例**（XYZ）：
```
tiles/
├── 0/
│   └── 0/
│       └── 0.png
├── 1/
│   ├── 0/
│   │   ├── 0.png
│   │   └── 1.png
│   └── 1/
│       ├── 0.png
│       └── 1.png
└── ...
```

### 3.3 转换为 MBTiles

```bash
mb-util --image_format=png tiles/ mine.mbtiles
```

**参数说明**：
- `--image_format=png` — 瓦片图片格式（也可用 `jpg`）
- `tiles/` — 输入目录（XYZ 结构）
- `mine.mbtiles` — 输出文件

**进度示例**：
```
Reading tiles from directory...
Saved 2048 tiles
```

### 3.4 验证（同 2.3）

```bash
sqlite3 mine.mbtiles "SELECT COUNT(*) FROM tiles;"
```

应显示正整数，表示瓦片数量。

---

## 4. 转换路线 C：SAS.Planet 卫星影像下载与导出

**适用场景**：需要下载在线卫星影像（如 Google Earth、Bing Maps）作为离线备份，后进行本地投影转换

### 4.1 前置工具

- **SAS.Planet**（开源 GIS 工具，支持多源卫星影像下载）— 下载地址：https://sasgis.org/
- **GDAL**（同路线 A 环境）

### 4.2 SAS.Planet 下载流程

1. 启动 SAS.Planet
2. 在地图上圈定矿区范围
3. 选择影像源（Google Satellite、Bing Maps 等）
4. 菜单 → **Map → Download Map** → 选择缩放级别范围（如 14-17）
5. 本地缓存保存目录（如 `~/.sasgis/cache/`）

### 4.3 将下载的瓦片导出为 GeoTIFF

SAS.Planet 缓存目录结构为 XYZ，可通过 GDAL 虚拟文件系统转换：

```bash
# 构建虚拟 VRT（GDAL 虚拟数据源）
gdalbuildvrt -nocdata 255 \
  -tileindex none \
  mine_vrt.vrt \
  /path/to/sasgis/cache/<source>/<zoom>/tiles/*.png

# 转换为 GeoTIFF（投影自动处理）
gdalwarp -t_srs EPSG:3857 mine_vrt.vrt mine_sasgis.tif

# 转换为 MBTiles（同路线 A）
gdal_translate -of MBTILES mine_sasgis.tif mine.mbtiles
gdaladdo mine.mbtiles 2 4 8 16
```

**说明**：
- 缓存目录结构因源而异（Google、Bing、OSM 等），可在 SAS.Planet 设置中查阅
- 虚拟文件系统会自动处理瓦片的重投影与镶嵌

### 4.4 批量下载脚本示例

如需编程批量下载，可使用 `requests` + 瓦片学 URL 构造：

```python
import os
import requests
from urllib.parse import quote

def download_tiles(minx, miny, maxx, maxy, min_zoom, max_zoom, source='google'):
    """
    source: 'google', 'bing', 'osm' 等
    """
    base_urls = {
        'google': 'https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
        'osm': 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
    }
    url_template = base_urls.get(source)
    
    for z in range(min_zoom, max_zoom + 1):
        # 经纬度 → 瓦片坐标转换（简化示例）
        for x in range(minx, maxx + 1):
            for y in range(miny, maxy + 1):
                url = url_template.format(z=z, x=x, y=y)
                filepath = f'tiles/{z}/{x}/{y}.png'
                os.makedirs(os.path.dirname(filepath), exist_ok=True)
                
                resp = requests.get(url, timeout=10)
                if resp.status_code == 200:
                    with open(filepath, 'wb') as f:
                        f.write(resp.content)
                    print(f'Downloaded {filepath}')
```

**注意**：下载前请检查源数据的使用协议（如 Google Maps 的 ToS）。

---

## 5. 部署与集成

### 5.1 文件放置

将生成的 `.mbtiles` 文件放置到指定目录，例如：

```bash
mkdir -p /data/tiles/
cp mine.mbtiles /data/tiles/

# 验证
ls -lh /data/tiles/mine.mbtiles
```

### 5.2 配置文件

在 `config.yaml` 中指定 MBTiles 路径：

```yaml
web:
  port: 8080
  host: 0.0.0.0
  static_dir: ""
  tiles_path: "/data/tiles/mine.mbtiles"  # 关键配置
```

**参数说明**：
- `tiles_path` — MBTiles 文件的绝对路径
- 空字符串（`""`）— 禁用离线瓦片，界面自动降级为网格背景（见 5.4）

### 5.3 容器部署（Docker）

如果使用 Docker 部署，挂载瓦片文件：

```yaml
volumes:
  - /data/tiles:/data/tiles
  - ./config.yaml:/data/config.yaml
```

### 5.4 无瓦片时的降级行为

当 `web.tiles_path` 为空或文件不存在时：

- **Web 界面自动切换为网格背景**（正方形坐标网格，浅灰色）
- 轨迹、事件标记仍然正常显示
- 基本定位、诊断功能不受影响
- 性能开销最小（无瓦片渲染）

**这是预期的降级行为**，不表示应用故障。可在任何时候通过更新 `tiles_path` 配置并重启应用来启用卫星影像。

---

## 6. 性能优化建议

### 6.1 瓦片大小与层级

| 参数 | 建议 | 说明 |
|---|---|---|
| 最大缩放级别 | 18-20 | 过高（>20）会导致文件巨大（GB+），加载变慢 |
| 最小缩放级别 | 0-2 | 通常保留，用于全球概览 |
| 目标区域覆盖缩放 | 14-18 | 在目标矿区的缩放范围内提供高清 |
| 文件大小 | <2 GB | 适合车载存储和快速加载 |

### 6.2 增量更新

如需更新部分区域的影像而不重新生成整个 MBTiles：

1. 生成增量 MBTiles（仅覆盖变化区域）
2. 使用 `mb-util` 的 `--merge` 选项合并（需特定版本支持）
3. 或删除旧文件，重新部署新的完整 MBTiles

### 6.3 首次加载优化

- 首次打开 Web 界面时，Leaflet 会加载当前视口的瓦片（通常 4-16 个）
- 后续平移/缩放时，仅加载可见范围的新瓦片（从 MBTiles 数据库按需读取）
- 建议在生成金字塔时多创建几个缩小层级（见 2.2 步骤 3）

---

## 7. 故障排查

### 7.1 Web 界面显示网格而非卫星影像

**检查项**：

1. 验证配置文件：
   ```bash
   grep tiles_path config.yaml
   ```
   应输出非空路径，如 `/data/tiles/mine.mbtiles`

2. 验证文件存在且可读：
   ```bash
   file /data/tiles/mine.mbtiles
   ls -lh /data/tiles/mine.mbtiles
   ```
   应显示 SQLite 3.x 格式，大小 > 1 MB

3. 查看应用日志：
   ```bash
   grep -i "tiles\|mbtiles" logs/rtk_monitor.log
   ```
   若有错误信息，如 `File not found`，检查路径与权限

4. 验证 MBTiles 完整性：
   ```bash
   sqlite3 /data/tiles/mine.mbtiles "SELECT COUNT(*) FROM tiles;"
   ```
   应显示正整数，如 `2048`

### 7.2 瓦片加载缓慢

**可能原因**：
- 未生成金字塔缩小层级（见 2.2 步骤 3 的 `gdaladdo`）
- MBTiles 文件存储在慢速磁盘（如 USB 3.0）

**优化方案**：
1. 重新生成金字塔：
   ```bash
   gdaladdo /data/tiles/mine.mbtiles 2 4 8 16
   ```

2. 移动到高速存储（如 SSD 或 /data 分区）

3. 确认 `max_zoom` 在合理范围（< 20）

### 7.3 某些区域出现灰色或无瓦片

**原因**：
- 转换时 zoom 范围不足（min/max_zoom 设置过小）
- GeoTIFF 源只覆盖了部分区域

**解决**：
1. 检查元数据：
   ```bash
   sqlite3 /data/tiles/mine.mbtiles "SELECT name, value FROM metadata WHERE name IN ('minzoom', 'maxzoom', 'bounds');"
   ```

2. 根据需要重新转换（扩大 zoom 范围）

3. 若源数据确实不覆盖某区域，可在 QGIS 中合并多个源影像后再转换

---

## 附录：一键转换脚本示例

将以下脚本保存为 `convert_to_mbtiles.sh`，简化转换流程：

```bash
#!/bin/bash
# 用法：bash convert_to_mbtiles.sh input.tif output.mbtiles

set -e

INPUT="${1:?Usage: $0 <input.tif> <output.mbtiles>}"
OUTPUT="${2:?Usage: $0 <input.tif> <output.mbtiles>}"

echo "1. 检查坐标系..."
gdalinfo "$INPUT" | grep "Coordinate System"

echo "2. 重投影到 EPSG:3857..."
TEMP_3857="$(mktemp --suffix=_3857.tif)"
trap "rm -f $TEMP_3857" EXIT

gdalwarp -t_srs EPSG:3857 "$INPUT" "$TEMP_3857"

echo "3. 转换为 MBTiles..."
gdal_translate -of MBTILES "$TEMP_3857" "$OUTPUT"

echo "4. 生成金字塔..."
gdaladdo "$OUTPUT" 2 4 8 16

echo "5. 验证..."
TILE_COUNT=$(sqlite3 "$OUTPUT" "SELECT COUNT(*) FROM tiles;")
echo "✓ 完成！瓦片数：$TILE_COUNT"
echo "✓ 输出文件：$OUTPUT"
```

运行：
```bash
chmod +x convert_to_mbtiles.sh
bash convert_to_mbtiles.sh mine_ortho.tif mine.mbtiles
```

---

## 参考资源

- [GDAL 官方文档](https://gdal.org/) — `gdal_translate`、`gdalwarp`、`gdaladdo`
- [mb-util GitHub](https://github.com/mapbox/mbutil) — XYZ ↔ MBTiles 互转
- [MBTiles 规范](https://github.com/mapbox/mbtiles-spec/blob/master/1.3/spec.md) — 格式细节
- [SAS.Planet 文档](https://sasgis.org/) — 卫星影像下载工具
- [EPSG:3857 Web Mercator](https://epsg.io/3857) — 坐标系参考
- [Leaflet 文档](https://leafletjs.com/) — 瓦片加载与 TileLayer 配置
