#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'int'):
    np.int = int

from gsnet import AnyGrasp

def main():
    # 用 checkpoint 或默认配置初始化
    cfg = {
        "model_path": "./model/checkpoint_detection.tar",
        "device": "cuda"
    }
    anygrasp = AnyGrasp(config=cfg)

    print("AnyGrasp 对象的方法列表：")
    for name in dir(anygrasp):
        print(name)

if __name__ == "__main__":
    main()
