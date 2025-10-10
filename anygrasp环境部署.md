#  AnyGrasp SDK 部署完整笔记

> **目标环境**：Ubuntu + CUDA 11.7 + Python 3.9
> **用途**：安装并运行 AnyGrasp SDK（含 MinkowskiEngine 与 PointNet2 支持）
> **建议 GPU**：NVIDIA RTX / Orin / A100 等 CUDA 设备

---

##  0️⃣ 基础检查 — GPU 驱动与编译器

```bash
nvidia-smi                # 查看 GPU 驱动及 CUDA 版本
nvcc --version || true    # 查看 CUDA 编译器版本
gcc --version             # 查看系统编译器版本
```

💡 **确保 CUDA 版本与 PyTorch 匹配**（例如 `nvcc 11.7` 对应 `pytorch-cuda=11.7`）

---

##  1️⃣ 创建 Conda 环境（Python 3.9）

```bash
conda create -n anygrasp python=3.9 -y
conda activate anygrasp
```

✅ 建议 Python 版本：`3.8 ~ 3.9`

---

##  2️⃣ 安装 PyTorch（CUDA 11.7）

> 选择 CUDA 11.7 是因为兼容性和稳定性最好。

```bash
conda install -y pytorch torchvision torchaudio pytorch-cuda=11.7 -c pytorch -c nvidia
```

---

##  3️⃣ 校验 PyTorch GPU 可用性

```bash
conda install pytorch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2 pytorch-cuda=11.7 -c pytorch -c nvidia

python -c "import torch; print('CUDA 可用:', torch.cuda.is_available())"
```

✅ 若输出 `CUDA 可用: True` 表示 GPU 可用。

---

##  4️⃣ 安装 MinkowskiEngine

```bash
git clone https://github.com/NVIDIA/MinkowskiEngine.git
cd MinkowskiEngine

# 清理 conda 缓存，避免冲突
conda clean -all

# 安装依赖
pip install ninja
conda install openblas-devel -c anaconda

# 设置 CUDA 环境变量（⚠️ 修改为你的 CUDA 路径）
export CUDA_HOME=/usr/local/cuda-11.7
export MAX_JOBS=2

# 编译安装
python setup.py install --blas_include_dirs=${CONDA_PREFIX}/include --blas=openblas
```

✅ 验证安装成功：

```bash
python -c "import MinkowskiEngine as ME; print('MinkowskiEngine 版本:', ME.__version__)"
```

---

##  5️⃣ 安装 Python 依赖

```bash
cd anygrasp_sdk
export SKLEARN_ALLOW_DEPRECATED_SKLEARN_PACKAGE_INSTALL=True
pip install -r requirements.txt
```

### 🧠 编译 PointNet2

```bash
cd pointnet2
python setup.py install
cd ..
```

---

##  6️⃣ 部署 License 与 Checkpoint

> 提交官方表格后会收到以下三份文件：
>
> 1. **License 文件**（压缩包，需解压）
> 2. **检测模型 checkpoint_detection.tar**
> 3. **跟踪模型 checkpoint_tracking.tar**

### 📁 操作步骤

```bash
# 解压 license 文件
# 复制两份并重命名为 license/
# 分别放入 grasp_detection/ 和 grasp_tracking/ 目录
```

复制 `.so` 动态链接库：

```bash
cd grasp_detection
cp gsnet_versions/gsnet.cpython-39-x86_64-linux-gnu.so gsnet.so
cp ../license_registration/lib_cxx_versions/lib_cxx.cpython-39-x86_64-linux-gnu.so lib_cxx.so
```

放置模型权重：

```bash
mkdir -p log
mv checkpoint_detection.tar log/
```

---

##  7️⃣ 再次编译 PointNet2（确保可用）

```bash
cd pointnet2
python setup.py install
cd ..
```

---

##  8️⃣ 最终验证

```bash
# 检查 PyTorch 与 CUDA 是否正常
python -c "import torch; print('torch', torch.__version__); print('cuda', torch.version.cuda); print('GPU 可用:', torch.cuda.is_available())"

# 检查 MinkowskiEngine 是否安装成功
python -c "import MinkowskiEngine as ME; print('MinkowskiEngine', ME.__version__)"
```

若输出版本号与 `GPU 可用: True`，则环境部署成功 🎉

---

## 📂 目录结构参考

```plaintext
anygrasp_sdk/
├── grasp_detection/
│   ├── gsnet.so
│   ├── lib_cxx.so
│   ├── license/
│   └── log/
│       └── checkpoint_detection.tar
├── grasp_tracking/
│   └── license/
├── pointnet2/
│   ├── setup.py
│   └── ...
└── requirements.txt
```

---

## 💡 常见问题提示

| 问题                                      | 原因                        | 解决方案                                  |
| --------------------------------------- | ------------------------- | ------------------------------------- |
| `torch.cuda.is_available() = False`     | 驱动或 CUDA 版本不匹配            | 重新安装 CUDA Toolkit 或更新驱动               |
| `undefined symbol` 编译错误                 | `gcc` 或 `CUDA_HOME` 配置不一致 | 检查 `nvcc --version` 与 `gcc --version` |
| `ImportError: cannot import name gsnet` | .so 文件未复制或命名错误            | 确认 gsnet.so 与 lib_cxx.so 文件路径正确       |

