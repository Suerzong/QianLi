# 低成本 RGB 单目 + 位置控制舵机机械臂的先进抓取策略工程对比报告

> 面向具体硬件:6 自由度串联机械臂(肩/肘/腕 + 双指夹爪,一指固定 + 一指移动),Feetech STS 系列位置控制总线舵机(可读负载百分比/电流),固定俯视 RGB 相机 + 棋盘/网格 homography(像素↔板面 mm),已知黄色立方体(33–55mm)在网格位置,MuJoCo 数字孪生(FK/IK)可用于仿真验证。

---

## 1. 执行摘要(工程结论,先看这里)

- **感知层:不需要任何学习式 6-DoF 抓取网络。** 本任务本质是"已知尺寸立方体 + 俯视平面 + 单目 RGB",退化成一个 **2.5D 平面抓取(x, y, 偏航角)**,而 homography 已经给出毫米级像素映射。经典 CV(颜色阈值 → 轮廓 → 中心/朝向)+ homography 是最稳、最可校准、零训练成本的方案。GraspNet/Contact-GraspNet/AnyGrasp/FoundationGrasp/GG-CNN/Dex-Net 全部依赖深度或点云,在此系统上**不可行**;纯 RGB 的 6-DoF 方法在文献中本质上都需要单目深度估计,毫米级精度不可靠。
- **闭合控制层:这是本系统唯一真正"先进"的发力点。** Feetech STS 总线舵机内置 **Torque Limit(扭矩限制,等效电流限制)**、**Present Load(负载读取)**、**Present Current(电流读取)** 与 **过载保护寄存器**,可以用"位置爬行 + 电流/负载闭环钳位"模拟柔顺夹持,实现教科书式的**负载闭环抓取(contact detection → clamp to target force → hold/verify)**。这替代的是"夹爪闭合控制"环节。
- **运动层:用数字孪生(MuJoCo)做蒙特卡洛参数整定**(接近高度、闭合速度、宽度余量、摩擦敏感性),而不是在线推理。
- **明确不可行:真 6-DoF 深度类抓取网络、真力/力矩传感器级阻抗/导纳、触觉/滑移传感器级 slip 检测、FoundationGrasp 类语言-点云任务导向抓取。**

---

## 2. 系统约束与问题退化分析

| 约束 | 含义 |
|---|---|
| 固定俯视 RGB + homography | 可获得板面毫米坐标,但**无高度信息**;立方体高度只能靠"已知尺寸"推断 |
| 无深度、无点云 | 所有基于 point cloud / depth 的 6-DoF 抓取方法直接出局 |
| 位置控制舵机(读负载%≈ 力代理) | 不能做关节级阻抗/导纳,但**可以**在夹爪层面做电流/负载闭环 |
| 单移动爪 + 固定爪(钳式) | 夹爪闭合是"移动爪推向固定爪"的单侧接触过程;接触检测是"移动爪先碰物体再把它压向固定爪" |
| 已知立方体(33–55mm)+ 网格位置 | 目标宽度已知 → **预期接触位置可预计算**,接近可分段(快速 → 慢速爬行) |
| MuJoCo 数字孪生(FK/IK) | 可用于 IK 路径验证、闭合/提升时序的蒙特卡洛整定、摩擦敏感性分析 |

**问题退化:** 俯视平面 + 顶抓 + 夹爪跨立 → 抓取位姿只需要 `(x, y, yaw)`,z 由已知立方体高度确定,下压量由夹爪开度/立方体尺寸确定。这不在"未知物体杂乱场景 6-DoF 抓取"范畴,而在**经典平面抓取(Pick-and-Place)**范畴。因此学习式方法的收益上限极低,而代价(深度传感器、GPU、数据、标定)极高。

---

## 3. 先进方法"现状对比"表

| 方法 | (a) 硬件/传感器需求 | (b) 改装到本系统的难度 | (c) 对本任务的预期收益 | (d) 引用 |
|---|---|---|---|---|
| **GraspNet-1Billion** | 深度/点云(RGB-D),GPU;输出 6-DoF 抓取位姿+质量分 | 高:需加深度相机+GPU+点云管线;还需抓取质量标注 | 极低(单已知物体,顶抓即可;网络是为未知物体杂乱场景设计) | [CVPR 2020](https://www.openaccess.thecvf.com/content_CVPR_2020/html/Fang_GraspNet-1Billion_A_Large-Scale_Benchmark_for_General_Object_Grasping_CVPR_2020_paper.html) |
| **Contact-GraspNet** | 原始点云(由深度获得);推理需 GPU≥8GB | 高:需深度相机 + TensorFlow 环境 + 点云预处理/分割 | 极低;即使有深度,对单个规则立方体也是大炮打蚊子 | [GitHub](https://github.com/codepk37/contact_graspnet_public) · [arXiv:2103.14127](https://arxiv.org/abs/2103.14127) |
| **AnyGrasp** | RGB-D 深度相机(RealSense 级);输出密集 7-DoF(位姿+宽度)实时抓取 | 高:需深度相机 + GPU 推理 + 标定 | 低;其卖点(杂乱装箱、动态抓取、深度噪声鲁棒)本任务全用不上 | [arXiv:2212.08333](https://arxiv.org/abs/2212.08333) |
| **FoundationGrasp** | 部分点云 + RGB + 语言指令;需 LLM/VLM + 采样器(它直接用 Contact-GraspNet 采样) | 极高:点云、大模型、TaskGrasp 数据;且是"任务导向抓取"(倒水、扫地等),本任务无语义需求 | 无 | [arXiv:2404.10399](https://ar5iv.labs.arxiv.org/html/2404.10399) |
| **GG-CNN** | **深度图**输入 → 每像素 quality/angle/width;50Hz 闭环 | 中:需深度相机;模型很小、算力要求低 | 低-中:如果将来加深度相机,它是平面抓取里最轻量的选择;但本任务经典 CV 已覆盖 | [arXiv:1804.05172](https://arxiv.org/abs/1804.05172) |
| **Dex-Net 4.0 / ambidextrous policies** | RGB-D 点云;并行爪;需要其配套抓取策略+硬件 | 高:深度相机 + 自定义双爪 + 大规模训练 | 无 | [Science Robotics 2019](https://pubmed.ncbi.nlm.nih.gov/33137754/) |
| **纯 RGB 抓取检测(MAD-Net 等)** | 单目 RGB;输出平面抓取框/位姿(需分割融合) | 中:只需现有 RGB 相机 + 训练/部署一个小 CNN | 低:对已知黄色立方体,HSV 阈值 + 轮廓的鲁棒性与可校准性更好;只有在光照/纹理剧烈变化时才值得考虑 | [arXiv:2310.19223](https://arxiv.org/abs/2310.19223) |
| **homography 平面抓取/自动工作区标定(CASE 2023)** | 单目 RGB + 标定板/网格 | **低:与本系统完全同构** | **高(感知层正解)**:像素↔板面 mm 映射 + 抓取点直接解算 | [Robot-Vision-PickPlace (CASE 2023)](https://github.com/mfkiwl/Robot-Vision-PickPlace) · [IROS 2019 平面抓取视觉伺服](https://www.ri.cmu.edu/app/uploads/2019/11/Efort_Grasping__IROS_2019.pdf) |
| **负载/电流闭环抓取(推荐)** | 无需新传感器:用舵机 Present Load(60)/Torque Limit(48)/Present Current(69) | **低:零硬件改动,纯固件/代码** | **高(闭合控制层正解)**:接触检测、钳位到目标力、保持校验 | [STS3215 寄存器表](https://github.com/commanderfun/STS3215/blob/main/REGISTER_REFERENCE.md) · [飞特电流反馈力控夹取 Demo](http://m.qwbw.cn/news/264477) |
| **阻抗/导纳(关节级)** | 需要关节力矩传感器/力控关节 | 高:硬件不具备;只能在夹爪层面"仿柔顺"(见 §6) | 低(关节级) | [Position-based impedance, sensorless force est. (2019)](https://www.ingentaconnect.com/content/mcb/033/2019/00000039/00000003/art00011) |
| **触觉/滑移传感器 slip 检测** | 触觉阵列/滑移传感器 + 快速低层力控 | 高:需更换夹爪/传感器 | 低-中:本任务只需"夹紧后不掉",负载保持监测即可覆盖 | [Slip-aware parallel grippers (arXiv:2410.19660)](https://arxiv.org/abs/2410.19660) · [Current as Touch](https://www.semanticscholar.org/paper/Current-as-Touch%3A-Proprioceptive-Contact-Feedback-Ma-Yao/cd1ebd42ff7b29abee6e2d52dfe6f7faf68f787f) |
| **MuJoCo 仿真抓取验证(推荐,离线)** | 现有数字孪生即可;需建夹爪/立方体模型 | 低:一次建模;蒙特卡洛离线跑 | 中:整定接近高度/闭合速度/宽度余量/摩擦敏感性;不做在线决策 | [MuJoCo grasp 社区讨论 #2309](https://github.com/google-deepmind/mujoco/discussions/2309) · [MuJoCo 摩擦蠕变问题 #3328](https://github.com/google-deepmind/mujoco/issues/3328) |
| **解析抓取质量度量(ε/力闭合)** | 只需物体+夹爪几何与摩擦系数 | 低:手算/库函数即可 | 中:顶抓立方体,双接触点力闭合可闭式求解,给"余量"数字 | [Grasp quality measures review (Roa & Suárez 2015)](https://dlnext.acm.org/doi/10.1007/s10514-014-9402-3) · [GraspIt!](http://www.cs.columbia.edu/~allen/PAPERS/Haptics00.pdf) |
| **2023–2025 前沿(基础模型/数据/语言驱动/摩擦感知)** | Grasp-Anything 数据、语言驱动抓取需 VLM、FirmGrasp 需摩擦模型、DextrAH-RGB 需灵巧手+GPU 策略 | 高 | 极低:面向开放词汇/未知物体/灵巧手,与本"单一已知立方体"任务不匹配 | [Grasp-Anything (ICRA 2024)](https://github.com/Fsoft-AIC/Grasp-Anything) · [Language-driven Grasp Detection (CVPR 2024)](https://openaccess.thecvf.com/content/CVPR2024/html/Vuong_Language-driven_Grasp_Detection_CVPR_2024_paper.html) · [FirmGrasp](https://arxiv.org/pdf/2607.25049) · [DextrAH-RGB](https://browse-export.arxiv.org/pdf/2412.01791) |

---

## 4. 分主题研究结论

### 4.1 学习式 6-DoF 抓取位姿检测:输入/输出/是否适配本系统

- **GraspNet-1Billion**:输入 RGB-D(深度点云),输出候选 6-DoF 抓取位姿及其质量分(基于解析标签),面向平行爪,评估标准就是仿真+真实拾取成功率。**必须有点云**。[CVPR 2020](https://www.openaccess.thecvf.com/content_CVPR_2020/html/Fang_GraspNet-1Billion_A_Large-Scale_Benchmark_for_General_Object_Grasping_CVPR_2020_paper.html)
- **Contact-GraspNet**:输入**原始场景点云**(README 明确:`--np_path` 给 `.npy/.npz` 深度+K,或 Nx3 点云),输出接触点上的 6-DoF 抓取分布;训练/推理均需 GPU(推理 ≥8GB)。**无点云不可用**。[GitHub](https://github.com/codepk37/contact_graspnet_public) · [arXiv:2103.14127](https://arxiv.org/abs/2103.14127)
- **AnyGrasp**:输入 RGB-D(深度是关键,论文强调"对大深度噪声鲁棒"),输出**密集、时序平滑的 7-DoF 抓取(6-DoF 位姿 + 抓取宽度)**,93.3% 装箱成功率、>900 picks/h。深度图缺失则无法工作。[arXiv:2212.08333](https://arxiv.org/abs/2212.08333)
- **FoundationGrasp**:输入部分点云 + RGB + 语言指令,采样-评估两阶段(采样阶段直接调用 Contact-GraspNet),面向"任务导向抓取"。**三重依赖:点云 + LLM/VLM + 大训练数据**,且本任务没有"任务"语义。[arXiv:2404.10399](https://ar5iv.labs.arxiv.org/html/2404.10399)
- **GG-CNN**:输入**深度图**(论文:"one-to-one mapping from a depth image"),输出每像素 quality/angle/width,模型小、单遍、50Hz 闭环、真实抓取 83–88%。是平面抓取里最轻量的深度类方法,但**仍需深度相机**。[arXiv:1804.05172](https://arxiv.org/abs/1804.05172)
- **Dex-Net 4.0**:RGB-D 点云 + 自适应双爪,学习"ambidextrous"抓取策略,深度依赖明确。[Science Robotics 2019](https://pubmed.ncbi.nlm.nih.gov/33137754/)
- **纯 RGB 支线**:存在 RGB-only 平面抓取检测(如 [MAD-Net,arXiv:2310.19223](https://arxiv.org/abs/2310.19223),语义分割融合、抗噪声),输出平面抓取位姿;也有"单目深度估计 + 抓取检测"路线([IEEE Access 2024](https://ieeexplore.ieee.org/ielx7/6287639/6514899/10521649.pdf)),但单目深度在平坦板面上毫米级不可靠。**结论:没有也不需要纯 RGB 的 6-DoF 方法——顶抓立方体本身只有 4-DoF 自由度。**

### 4.2 力/负载闭环抓取(接触检测、钳位、滑移监测)

标准方法(文献与工业实践):
1. **接触检测**:夹爪以低速度爬行,当 Present Load(或电流)越过阈值 → 接触;对单移动爪结构,检测的是"移动爪压到物体并把物体压向固定爪"的负载跳变。
2. **钳位到目标力**:位置控制舵机没有力环,但 **Torque Limit(寄存器 48,0–1000=0–100%)** 是硬件电流限制 → 设置目标 Torque Limit 后,舵机会在该扭矩下"软停滞",等价于**电流受限的力保持**。
3. **保持校验**:钳位后等待位置蠕变停止(爪座实),再以"轻推扰动(±1–2mm)"验证负载保持。
4. **滑移检测(无触觉)**:提升/搬运期间周期性读负载;若负载骤降且位置漂移(未堵转) → 疑似滑脱,立即重钳位或中止。这是"基于负载的启发式 slip",而非真滑移传感。

Feetech STS 提供的具体寄存器(见 [STS3215 寄存器表](https://github.com/commanderfun/STS3215/blob/main/REGISTER_REFERENCE.md)):
- `48 Torque Limit`(RAM,0–1000=0–100%)→ 电流限制,软钳位
- `60 Present Load`(bits 0-9 幅值, bit 10 方向)→ 力代理
- `69 Present Current`(×6.5mA)→ 更直接的电流
- `34/35/36 Protection Torque/Time/Overload Torque` → 过载保护(堵转判据)
- `42 Goal Position / 46 Goal Speed / 44 Goal Time` → 位置+速度+时间控制(爬行用)
- 社区已有"电流反馈力控夹取"实现参考([飞特舵机+夹爪 基于电流反馈的力控夹取 Demo](http://m.qwbw.cn/news/264477))。

参考研究:[Slip-aware parallel grippers](https://arxiv.org/abs/2410.19660)(需传感器化夹爪+快速力控,展示"完整版"做法)、[Current as Touch](https://www.semanticscholar.org/paper/Current-as-Touch%3A-Proprioceptive-Contact-Feedback-Ma-Yao/cd1ebd42ff7b29abee6e2d52dfe6f7faf68f787f)(电机电流当"触觉"的灵巧手柔顺操控——理念与本系统一致,但面向灵巧手)。

### 4.3 用位置控制舵机仿真柔顺/阻抗/导纳

关节级阻抗/导纳需要力矩环,本硬件不具备;但在**夹爪层面**可用"位置爬行 + 电流限制"模拟柔顺:
- **动态钳位(step-and-hold)**:分段逼近,每段小步 + 保持,读负载趋势;避免高速撞物(顶抓立方体时"预期接触位置已知",可先快后慢)。
- **软接触**:把 Goal Speed 调低 + Torque Limit 设为目标夹持力,使"接触→停滞"过程近似为硬弹簧 + 电流限幅的软特性。
- **固件级"力保持"**:接触后不追求到位,而以"停在一个会堵转的位置 + 扭矩限制"实现恒定夹持力——即位置式阻抗的工程近似。
- 外环位置式阻抗的文献支撑:位置控制机器人的无传感力估计阻抗控制器([Position-based impedance, sensorless force estimation](https://www.ingentaconnect.com/content/mcb/033/2019/00000039/00000003/art00011))。

### 4.4 视觉 + 仿真抓取验证(MuJoCo)

- **解析侧**:顶抓立方体 = 两平行接触面上的力闭合。Ferrari–Canny ε 度量/力闭合可在摩擦锥 + 重力负载下闭式求解;给"余量"数字用于选接近高度与宽度余量。见 [Grasp quality measures review (Roa & Suárez)](https://dlnext.acm.org/doi/10.1007/s10514-014-9402-3) 与 [GraspIt!](http://www.cs.columbia.edu/~allen/PAPERS/Haptics00.pdf)。
- **仿真侧**:MuJoCo 数字孪生已有 FK/IK → 补夹爪+立方体模型,做**离线蒙特卡洛**:对抓取位姿施加 ±mm/±deg 抖动、扫描摩擦系数 μ∈[0.2,0.8]、扫闭合速度/下压量,统计"闭合→提升→搬运→放置"成功概率。注意 MuJoCo 接触模型的实际坑:regularized friction 会产生蠕变("pinched capsule 以 μm/s 滑移"),[issue #3328](https://github.com/google-deepmind/mujoco/issues/3328);抓取场景整定经验见 [discussion #2309](https://github.com/google-deepmind/mujoco/discussions/2309)。
- **是否值得**:对单一已知立方体——**值得但只做离线整定,不做在线推理**。价值在于:(1) IK 可达/路径验证;(2) 接近高度、闭合速度、宽度余量的安全区间;(3) 摩擦敏感性。成本:一次建模 + 一夜跑批。不要用它做实时"验证再执行"(本任务几何简单,解析余量已够)。
- 大规模 sim-to-real 属更高投入路线(如 [MuJoCo Playground 像素策略 sim-to-real](https://ar5iv.labs.arxiv.org/html/2502.08844)),与低成本顶抓不匹配。

### 4.5 2023–2025 值得知道的新方向(及为何不适配)

- **基础模型抓取数据**:Grasp-Anything([ICRA 2024](https://github.com/Fsoft-AIC/Grasp-Anything))用 SAM+基础模型大规模生成抓取标注;语言驱动抓取检测([CVPR 2024](https://openaccess.thecvf.com/content/CVPR2024/html/Vuong_Language-driven_Grasp_Detection_CVPR_2024_paper.html))用 VLM 输出抓取框。面向开放词汇/未知物体——本任务无此需求。
- **摩擦感知鲁棒度量**:FirmGrasp([2025](https://arxiv.org/pdf/2607.25049))把摩擦不确定性纳入风险余量——概念上可用于本系统的"余量"分析,但实现重。
- **纯 RGB 灵巧手策略**:DextrAH-RGB([arXiv:2412.01791](https://browse-export.arxiv.org/pdf/2412.01791))——需要灵巧手 + 大规模 RL + GPU 实时策略,明显不适配。
- **无触觉 slip 监测(与本系统最相关)**:以电机电流/负载做接触与滑移感知([Current as Touch](https://www.semanticscholar.org/paper/Current-as-Touch%3A-Proprioceptive-Contact-Feedback-Ma-Yao/cd1ebd42ff7b29abee6e2d52dfe6f7faf68f787f);飞特社区 Demo [电流反馈力控夹取](http://m.qwbw.cn/news/264477))——这正是推荐方案的技术渊源。
- **反应式抓取(reactive grasping)**:把感知-闭合-提升做成连续闭环(如 AnyGrasp 的时序跟踪),工业趋势是"感知 + 本体感受反馈"的闭环;对本系统,闭环落在**夹爪负载环**,而不是视觉环。

---

## 5. 明确"不可行/不划算"清单

| 方案 | 为何不可行 |
|---|---|
| GraspNet-1Billion / Contact-GraspNet / AnyGrasp / Dex-Net 4.0 | **都需要深度/点云**,本系统无深度传感器;且为未知杂乱物体设计 |
| FoundationGrasp / 语言驱动抓取 | 需点云 + LLM/VLM + 大数据 + 任务语义;完全不对口 |
| GG-CNN(深度版) | 需深度相机(若将来加深度相机,它是平面抓取最优轻量选择) |
| 关节级阻抗/导纳/力控 | 需力矩传感或力控关节;本系统只有位置环+负载读数,只能在夹爪层面近似 |
| 触觉阵列/滑移传感器 slip 检测 | 需换夹爪硬件;本任务用"负载保持监测"即可达到足够可靠性 |
| MuJoCo 在线"验证后执行" | 几何简单,解析余量即可;仿真留作离线整定 |
| 2023–2025 基础模型/灵巧手路线 | 面向开放世界/灵巧手/大规模数据,对本单一已知立方体收益≈0 |

---

## 6. 推荐方案(工程实践,按优先级)

### 方案 1(首选,零硬件改动):homography 视觉 + 负载闭环"爬行钳位"夹持

**替代环节:感知 + 闭合控制。** 不引入任何学习网络。

感知(经典 CV + homography):
1. 固定相机拍板面;HSV 颜色阈值提取黄色区域 → 形态学去噪 → 最大轮廓 → 轮廓中心 (u,v) 与(可选)朝向 θ;
2. homography 将 (u,v)→(X,Y) 板面 mm(网格标定已给);
3. 目标抓取点 = 立方体中心,偏航角 θ(立方体对称,θ 可任意;若矩形则取轮廓主轴);
4. 顶抓高度 z = 立方体高度(已知 33–55mm)+ 安全间隙。

闭合控制(STS 寄存器序列,替代"盲目闭合"):
```
1) GoalSpeed=慢(如 60)                # 46
2) 接近至 (立方体宽 - 余量 2~4mm) 位置   # 42 快速段,预期接触位置已知
3) 进入爬行段:每次 GoalPosition +Δ,间隔 20ms 读 PresentLoad(60)
4) Load 跳变 > 阈值 → 判定接触;记录当前宽度
5) 设置 TorqueLimit = 目标夹持力(如 40%) # 48 → 软钳位,不硬堵转
6) 等待 200~500ms 位置蠕变停止(爪座实),读 Load 校验稳定
7) 提升:慢速垂直升;搬运中周期读 Load,若 Load 骤降+位置漂移 → 重钳位或中止
8) 放置:慢速下降,Load 再跳变 → 已触板面,停止下压,开爪(扭矩关) → 回退
```
优点:零新增硬件;完全可校准;失败可诊断(记录每次的负载曲线)。引用:[CASE 2023 homography 标定](https://github.com/mfkiwl/Robot-Vision-PickPlace)、[IROS 2019 平面抓取视觉伺服](https://www.ri.cmu.edu/app/uploads/2019/11/Efort_Grasping__IROS_2019.pdf)、[STS3215 寄存器表](https://github.com/commanderfun/STS3215/blob/main/REGISTER_REFERENCE.md)、[电流反馈力控 Demo](http://m.qwbw.cn/news/264477)。

### 方案 2(离线增强):MuJoCo 数字孪生蒙特卡洛整定 + 解析余量

**替代环节:参数整定(不替换感知/控制,而是给方案 1 提供安全区间)。**
- 建模:夹爪(固定爪+移动爪)、立方体(33/45/55mm 三档)、板面、摩擦参数;
- 扫描:接近高度误差 ±3mm、宽度余量 0~6mm、闭合速度、夹持力(扭矩限制等价摩擦)、提升加速度;
- 输出:成功概率等高线 → 定"目标夹持力 / 余量 / 下压量"默认值;
- 同时用解析力闭合/ε 度量给出余量数字([Roa & Suárez 综述](https://dlnext.acm.org/doi/10.1007/s10514-014-9402-3))。
- 注意 MuJoCo regularized friction 蠕变坑([#3328](https://github.com/google-deepmind/mujoco/issues/3328))与社区整定经验([#2309](https://github.com/google-deepmind/mujoco/discussions/2309))。

### 方案 3(可选升级,低成本):加一块深度相机的"未来路线图"

仅当未来目标扩展到未知物体/杂乱场景时:
- 加低成本深度相机(如 RealSense D435 级);
- 感知换 GG-CNN(平面,轻量,50Hz,[arXiv:1804.05172](https://arxiv.org/abs/1804.05172))或 AnyGrasp(6-DoF,密集,[arXiv:2212.08333](https://arxiv.org/abs/2212.08333));
- 闭合控制保持方案 1 的负载闭环不变(两种深度方法输出都是抓取位姿,闭合逻辑复用);
- 不要直接上 FoundationGrasp/语言驱动(收益与成本比极差)。

---

## 7. 引用链接汇总

- [GraspNet-1Billion (CVPR 2020)](https://www.openaccess.thecvf.com/content_CVPR_2020/html/Fang_GraspNet-1Billion_A_Large-Scale_Benchmark_for_General_Object_Grasping_CVPR_2020_paper.html)
- [Contact-GraspNet GitHub](https://github.com/codepk37/contact_graspnet_public) / [arXiv:2103.14127](https://arxiv.org/abs/2103.14127)
- [AnyGrasp (arXiv:2212.08333)](https://arxiv.org/abs/2212.08333)
- [FoundationGrasp (arXiv:2404.10399)](https://ar5iv.labs.arxiv.org/html/2404.10399)
- [GG-CNN (arXiv:1804.05172)](https://arxiv.org/abs/1804.05172)
- [Dex-Net 4.0 / ambidextrous (Science Robotics 2019)](https://pubmed.ncbi.nlm.nih.gov/33137754/)
- [MAD-Net 纯 RGB 抓取检测 (arXiv:2310.19223)](https://arxiv.org/abs/2310.19223)
- [Attention-based Grasp Detection with Monocular Depth (IEEE Access 2024)](https://ieeexplore.ieee.org/ielx7/6287639/6514899/10521649.pdf)
- [Robot-Vision-PickPlace: homography 自动标定 (IEEE CASE 2023)](https://github.com/mfkiwl/Robot-Vision-PickPlace)
- [Homography-Based Deep Visual Servoing for Planar Grasps (IROS 2019, CMU)](https://www.ri.cmu.edu/app/uploads/2019/11/Efort_Grasping__IROS_2019.pdf)
- [STS3215 寄存器参考(Torque Limit/Present Load/Present Current/过载保护)](https://github.com/commanderfun/STS3215/blob/main/REGISTER_REFERENCE.md)
- [Feetech STS3215 产品页](https://www.feetechrc.com/525603)
- [飞特舵机+夹爪:基于电流反馈的力控夹取 Demo](http://m.qwbw.cn/news/264477)
- [Slip-aware parallel grippers (arXiv:2410.19660)](https://arxiv.org/abs/2410.19660)
- [Current as Touch: 电流作为触觉(Semantic Scholar)](https://www.semanticscholar.org/paper/Current-as-Touch%3A-Proprioceptive-Contact-Feedback-Ma-Yao/cd1ebd42ff7b29abee6e2d52dfe6f7faf68f787f)
- [Grasp quality measures: review and performance (Roa & Suárez 2015)](https://dlnext.acm.org/doi/10.1007/s10514-014-9402-3)
- [GraspIt! 仿真器](http://www.cs.columbia.edu/~allen/PAPERS/Haptics00.pdf)
- [MuJoCo grasp 讨论 #2309](https://github.com/google-deepmind/mujoco/discussions/2309) / [MuJoCo 摩擦蠕变 issue #3328](https://github.com/google-deepmind/mujoco/issues/3328)
- [MuJoCo Playground (arXiv:2502.08844)](https://ar5iv.labs.arxiv.org/html/2502.08844)
- [Grasp-Anything (ICRA 2024)](https://github.com/Fsoft-AIC/Grasp-Anything)
- [Language-driven Grasp Detection (CVPR 2024)](https://openaccess.thecvf.com/content/CVPR2024/html/Vuong_Language-driven_Grasp_Detection_CVPR_2024_paper.html)
- [FirmGrasp (2025)](https://arxiv.org/pdf/2607.25049)
- [DextrAH-RGB (arXiv:2412.01791)](https://browse-export.arxiv.org/pdf/2412.01791)
- [Position-based impedance force controller with sensorless force estimation (2019)](https://www.ingentaconnect.com/content/mcb/033/2019/00000039/00000003/art00011)
