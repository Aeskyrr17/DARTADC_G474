# DARTADC_G474

RM26飞镖系统G474力传感器ADC代码

## 组成

### 1. G4_sensor_linearfitting
此部分包含了用于传感器标定的 Python 脚本，主要用于读取采集的 ADC 测试记录数据，并使用最小二乘法对传感器的 ADC 值与实际物理受力（kg）进行线性拟合计算，最终提供在 C/C++ 中使用的校准系数（`k` 和 `b`）。

#### 1.1 环境配置（创建 .venv 与安装依赖）
该部分的脚本主要是基础数据处理，目前仅使用了 Python 标准库，可以直接运行。但为了后续可能的开发需求，推荐通过以下流程使用虚拟环境进行配置：

1. **打开终端并进入脚本目录**：
   ```bash
   cd G4_sensor_linearfitting
   ```
2. **创建虚拟环境（.venv）**：
   ```bash
   python -m venv .venv
   ```
3. **激活虚拟环境**：
   - **Windows (PowerShell)**:
     ```powershell
     .\.venv\Scripts\Activate.ps1
     ```
   - **Windows (CMD)**:
     ```cmd
     .venv\Scripts\activate.bat
     ```
   - **Linux / macOS (Bash)**:
     ```bash
     source .venv/bin/activate
     ```
4. **依赖**：
    - 目前只有标准库依赖
   ```

#### 1.2 如何使用

该工具包主要由两个脚本组成。典型的标定**工作流**为：先通过 `avg.py` 得到某一特定重量下的 ADC 平均值，再将一系列（重量，ADC）组合送入 `linear_fitting.py` 得到相应的线性公式。

##### 步骤一：使用 `avg.py` 计算 ADC 平均值
`avg.py` 的作用是解析特定格式存放的 txt 文件（从**FreeMaster**导出至 `DataRecord` 文件夹），求出一组静态记录里的 ADC 平均值。

1. 记录特定重量（例如 20kg、50kg）下的传感器 ADC 数值，并将文件存至 [DataRecord/](DataRecord/) 目录下（如 `osc00013.txt`）。
2. 用编辑器打开 [G4_sensor_linearfitting/avg.py](G4_sensor_linearfitting/avg.py)。
3. 在文件最下方找到 `filename` 变量，将其修改为您要读取的 txt 文件绝对或相对路径，例如：
   ```python
   filename = "../DataRecord/osc00013.txt"
   ```
4. 运行脚本：
   ```bash
   python avg.py
   ```
   **输出结果示例**：记录下终端打印的 `Average value`，这个数值就是对应重力下的精准 ADC 值。

##### 步骤二：使用 `linear_fitting.py` 完成标定
`linear_fitting.py` 会读取一组标定点参数，根据 $y = kx + b$ （$x$ 为 ADC，$y$ 为 force_kg）拟合出传感器所需的乘系数与补偿常量，并生成相关的宏定义。

1. 编辑 [G4_sensor_linearfitting/linear_fitting.py](G4_sensor_linearfitting/linear_fitting.py)。
2. **选择数据录入方式**：
   - **静态配置（推荐）**：找到代码上方的 `CALIB_POINTS` 列表，按照 `(force_kg, adc)` 格式填入若干标定组（即步骤一得到的组合），解除注释即可。如果同一重力测了多次，保持 `AVERAGE_SAME_FORCE = True` 即可自动计算出平均值进行拟合。

3. 运行拟合脚本：
   ```bash
   python linear_fitting.py
   ```
4. **获取结果**：终端会打印出均方根误差 (RMSE)、决定系数 (R^2) 等质量报告以验证拟合效果。
   - 然后将对应的k与b填入
 ```
   ../Core/Src/main.cpp
   ```
   开头的宏定义，eg.
   ```cpp
    #define FORCE_SENSOR_L_LINEAR_K 0.0037902456f
    #define FORCE_SENSOR_L_LINEAR_B -124.4756404968f
    #define FORCE_SENSOR_R_LINEAR_K 0.0043918528f
    #define FORCE_SENSOR_R_LINEAR_B -144.3149817305f
```

### 2. DataRecord
用于存放FreeMaster导出的数据

### 3. 其余部分
- 烧录进G474开发板的代码，主要用于ADC数据的读取、滤波与处理。
- 采用硬件一阶低通滤波加上基于参数配置的双模式标定策略（支持公式法或三点线性插值法）。
  - 目前的一阶滤波参数可以将传感器跳变控制在+-0.02kg之间
  - 
- 最终的力学数据利用串口(USART2) DMA方式发送。

**帧通讯协议：**

数据按固定长度包进行发送，每一帧共有 8 个字节，具体格式如下：

| 字节偏移 | 内存(含义) | 数据格式和说明 |
|:---:|:---|:---|
| `[0]` | 帧头标志 'L' | 字符 `'L'`（即 `0x4C`），代表左传感器(Left) |
| `[1~3]` | 左传感器数据 | 无符号 24 位整数（小端模式），单位为 `0.1克(0.1g)`<br>相当于将 `force_kg * 10000` |
| `[4]` | 帧头标志 'R' | 字符 `'R'`（即 `0x52`），代表右传感器(Right) |
| `[5~7]` | 右传感器数据 | 无符号 24 位整数（小端模式），单位为 `0.1克(0.1g)`<br>相当于将 `force_kg * 10000` |

> **提示**：发送数据内部已被自动限幅在 `0.0kg` 到 `100.0kg` 之间。**接收端读取对应的3字节小端后数据，再除以 10000.0 即可还原出对应的 kg 单位浮点数。**

