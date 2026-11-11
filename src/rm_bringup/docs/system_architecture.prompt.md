# 系统框架图生成提示词

通过内置 Image 2 图像工具重新生成黑白学术风格框架图，并定向校正静态地图输入和避障扫描连线。最终图片：`system_architecture.png`。

## 初始生成

```text
Use case: infographic-diagram.
Asset type: final academic system architecture figure for a Chinese robotics README and research paper.
Primary request: Regenerate a precise robotics architecture diagram from scratch in STRICT BLACK AND WHITE ONLY. Use white #FFFFFF background and black #000000 text, borders, and arrows. No colored pixels, no colored accents, no gray fills, no gradients, no shadows, no decorative illustrations. Grayscale antialiasing along black text and line edges is acceptable. Academic publication style: restrained, logical, well-spaced, consistent thin straight rectangular borders, clear small filled arrowheads, orthogonal routing, black dashed group boundaries. All rectangles have white interiors. Use elegant Chinese Songti or similar academic typeface for Chinese labels, Times New Roman for English identifiers; keep type large and sharp. No rounded colored cards, no stage background panels. Aim for a landscape 4:3 high-resolution figure with generous margins. Readability at 900px displayed width is essential.
Title, modestly sized: "哨兵自主导航系统框架". Do not add an oversized title, decorative subtitle or explanatory paragraphs.

SYSTEM TOPOLOGY (all nodes and edges below are required; do not silently omit them):
A. Top input node "Mid-360 点云 + IMU" with a small second line "实机 / Gazebo". Its output goes to node "FAST-LIO" with second line "里程计与车体点云".
B. FAST-LIO has three outputs feeding the following independent paths:
  - left PRIOR path inside a thin black dashed boundary headed "先验导航（prior）": node "PCD 地图" → node "GICP 重定位"; FAST-LIO also → "GICP 重定位"; "GICP 重定位" → Nav2 with edge label "定位 TF". A second small input "二维地图" inside this prior boundary connects DIRECTLY to Nav2 as "先验地图".
  - middle EXPLORE path inside a thin black dashed boundary headed "在线探索（explore）": node "三维回环 + PGO" with second line "在线地图重建". FAST-LIO → this node. It has TWO DISTINCT OUTPUTS: (1) → Nav2, edge label "地图 / 定位 TF"; (2) → node "前沿探索器" → node "导航客户端", edge label "探索目标".
  - right perception path: FAST-LIO → node "障碍感知" with second line "地面过滤与高度切片"; this node → Nav2, edge label "/scan".
C. External input node "界面 / 巡检 / 上层决策" → "导航客户端" → "Nav2"; the last edge label is "导航目标". The "导航客户端" receives both this external goal source AND the "前沿探索器" source. These are separate from map delivery.
D. Central main node "Nav2" with second line "全局规划 + MPPI 全向控制".
E. Nav2 → node "自旋参考系适配", edge label "/nav_cmd_vel". Then this adapter → node "CAN 仲裁", edge label "/cmd_vel". CAN node second line "受击自旋 / 急停 / 超时保护".
F. The CAN arbiter is the LAST SHARED NODE, splitting into exactly two output nodes at bottom: "实机：USB-CAN → MCU" and "仿真：载荷解码 → Gazebo".
Small footer text, exactly: "prior / explore 二选一    TF：map → odom → base_link".
Avoid line crossings; if unavoidable, use a wire bridge, never an unintended junction. Connect arrows to box boundaries, not through any text. Ensure GICP → Nav2, PGO → frontier → client → Nav2, and PGO → Nav2 are all visibly present. All labels must be exact Simplified Chinese and the English code identifiers above. No tests, no bugs, no benchmarks, no unsupported features. Keep the entire image purely black and white and suitable for academic print.
```

## 连线校正

```text
Edit the supplied black-and-white academic architecture diagram. Preserve the entire diagram content, thin black rectangular box style, serif academic typography, dashed prior/explore groups, and bottom CAN execution split. Make ONLY the following targeted corrections:
1. PCD 地图 is an offline STATIC MAP INPUT, not an output of FAST-LIO. REMOVE the incoming arrow from FAST-LIO to PCD 地图, including its leftward horizontal segment and downward arrow. Keep the arrow PCD 地图 → GICP 重定位. Keep the separate FAST-LIO → GICP 重定位 arrow. FAST-LIO must have exactly three outgoing connections: GICP 重定位, 三维回环 + PGO, 障碍感知.
2. The /scan connection from 障碍感知 to Nav2 currently runs through the box 界面 / 巡检 / 上层决策. Reroute this /scan wire AROUND that box, using an orthogonal route with generous clearance. It must go straight from 障碍感知 to Nav2 and must not enter, cross or touch the target input box. Preserve the separate arrow 界面 / 巡检 / 上层决策 → 导航客户端. You can slightly reposition the target input box and the scan wire to achieve clean separation.
3. Make the background perfectly solid neutral white and all visible markings neutral black. Exactly black ink on white paper, no textures, no speckle, no colored tints, no off-white cast, no shadows or gray fills. Any antialiasing must be achromatic, with R=G=B. Everything must look like a clean print-ready academic engineering diagram.
Preserve all other arrows, all Chinese labels, and the technical identifiers, including GICP → Nav2, PGO → Nav2, PGO → 前沿探索器 → 导航客户端 → Nav2, Nav2 → 自旋参考系适配 → CAN 仲裁, and CAN 仲裁 → both execution outputs. Do not add a title or any new content.
```
