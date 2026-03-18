       # polaris_ultrasound 使用说明（更新版）

    ## 1. 环境配置

    ```bash
    cd /ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash
    ```

    ---

    ## 2. 包含功能列表

    - Polaris 光学跟踪读取节点：`polaris_reader`
    - 超声实时采集：`ultrasound_reader`
    - Stylus 与 Phantom 点采集：`phantom_experiment````
    - 离线超声数据集播放：`us_dataset_player_node`
    - **超声稳定度 Sk 计算（含 ROI mask）：`us_stability_node`**
    - 超声目标点检测：`us_target_estimator_node`
    - ROI mask 绘图工具：`draw_us_mask.py`

    ---

    ## 3. 常用节点

    ### 3.1 Polaris 读取节点

    ```bash
    ros2 run polaris_ultrasound polaris_reader --ros-args \
      -p rom_path:="/opt/ndi/rom_files/8700449_phantom.rom,/opt/ndi/rom_files/8700340_stylus.rom" \
      -p tool_names:="phantom,stylus"
    ```

    发布话题：
    - `/polaris/phantom`
    - `/polaris/stylus`
    - `/polaris/raw`

    ---

    ### 3.2 超声采集节点

    ```bash
    ros2 run polaris_ultrasound ultrasound_reader --ros-args \
      -p device_id:=6
    ```

    发布：`/us_img`

    ---

    ## 4. 离线超声数据播放：us_dataset_player_node

    ```bash
    ros2 run polaris_ultrasound us_dataset_player_node --ros-args \
      -p data_dir:=/ros2_ws/src/silicon_exp1/20250810_072526 \
      -p fps:=10.0 \
      -p json_mode:=fixed_dir \
      -p json_dir:=/ros2_ws/src/silicon_exp1 \
      -p loop:=false
    ```

    输出话题：
    - `/us_img`
    - `/probe_pose`

    ---

    ## 5. 超声稳定度计算：us_stability_node（Sk）

    Sk 结合了：
    - ROI 灰度标准差 Csp  
    - ROI Sobel 梯度均值 Cgrad  
    - 归一化  
    - 指数平滑

    示例：

    ```bash
    ros2 run polaris_ultrasound us_stability_node --ros-args \
      -p image_topic:=/us_img \
      -p stability_topic:=/us_stability \
      -p mask_path:=/ros2_ws/src/silicon_exp1/us_mask.png \
      -p alpha:=0.3 \
      -p c_sp_max:=17.6 \
      -p c_grad_max:=7.1
    ```

    输出话题：`/us_stability`

    ---

    ## 6. Stylus / Phantom 点采集

    ```bash
    ros2 run polaris_ultrasound phantom_experiment
    ```

    保存路径：

    ```
    /ros2_ws/calibration_data/polaris/stylus_phantom/
    ```

    ---

    ## 7. ROI Mask 绘制工具

    ```bash
    python3 draw_us_mask.py \
      --image /ros2_ws/src/silicon_exp1/20250810_072526/us_000001.png \
      --output /ros2_ws/src/silicon_exp1/us_mask.png
    ```

    操作方法：
    - 左键：添加顶点  
    - 右键：闭合区域  
    - r：重置  
    - s：保存  
    - ESC：退出  

    ---

    ## 8. 常用参数

    ```
    image_topic       输入图像话题
    stability_topic   输出 Sk
    mask_path         ROI mask
    json_mode         paired / fixed_dir / fixed_single
    json_dir          JSON 存放目录
    fps               数据播放帧率
    alpha             Sk 平滑系数
    c_sp_max          Csp 归一化上限
    c_grad_max        Cgrad 归一化上限
    ```

    ---

    ## 9. 推荐完整流程

    ### Step 1：播放数据集

    ```bash
    ros2 run polaris_ultrasound us_dataset_player_node --ros-args \
      -p data_dir:=/ros2_ws/src/silicon_exp1/20250810_072526 \
      -p json_mode:=fixed_dir \
      -p json_dir:=/ros2_ws/src/silicon_exp1 \
      -p fps:=10.0 \
      -p loop:=false
    ```

    ### Step 2：运行 Sk 模块

    ```bash
    ros2 run polaris_ultrasound us_stability_node --ros-args \
      -p image_topic:=/us_img \
      -p stability_topic:=/us_stability \
      -p mask_path:=/ros2_ws/src/silicon_exp1/us_mask.png \
      -p alpha:=0.3 \
      -p c_sp_max:=17.6 \
      -p c_grad_max:=7.1
    ```

    ### Step 3：目标点检测

    ```bash
    ros2 run polaris_ultrasound us_target_estimator_node
    ```

    ### Step 4：查看输出

    ```bash
    ros2 topic echo /us_stability
    ros2 topic echo /us_target
    ros2 topic echo /probe_pose
    ```

   
   
    # polaris_ultrasound 使用说明（含 Sk 公式）

    ## 1. 超声稳定度 Sk 定义（数学公式）

    超声图像质量稳定度（Stability Index, Sk）由以下三个成分构成：

    1. ROI 灰度标准差：
       C_sp = std( I(u,v) ),   (u,v ∈ ROI)

    2. ROI Sobel 梯度均值：
       C_grad = mean( sqrt( (Gx)^2 + (Gy)^2 ) )

    3. 各成分归一化到 [0,1]：
       C_sp_norm   = clip( C_sp   / c_sp_max,   0, 1 )
       C_grad_norm = clip( C_grad / c_grad_max, 0, 1 )

    然后将两个分量按权重组合（目前为等权）：

       Sk_raw = 0.5 * C_sp_norm + 0.5 * C_grad_norm

    最后对 Sk_raw 进行指数平滑得到最终输出：

       S_bar(k) = α * Sk_raw(k) + (1 - α) * S_bar(k-1)

    其中：
    - C_sp_max   = 17.6（由统计 998 帧数据得出）
    - C_grad_max = 7.1 （同上）
    - α（alpha）是平滑系数，通常 0.2～0.3

    Sk 和 S_bar 都属于 [0,1]。

    ---

    ## 2. 运行 Sk 计算节点

    ```bash
    ros2 run polaris_ultrasound us_stability_node --ros-args \
      -p image_topic:=/us_img \
      -p stability_topic:=/us_stability \
      -p mask_path:=/ros2_ws/src/silicon_exp1/us_mask.png \
      -p alpha:=0.3 \
      -p c_sp_max:=17.6 \
      -p c_grad_max:=7.1
    ```

    输出话题：`/us_stability`

    ---

    ## 3. 完整 Sk 处理流程

    1. 用 mask 选定 ROI 区域  
    2. 计算 ROI 的灰度标准差 C_sp  
    3. 计算 ROI 的 Sobel 梯度均值 C_grad  
    4. 分别做归一化  
    5. 合成为 Sk_raw  
    6. 用指数平滑得到 S_bar（发布值）  
    7. S_bar 可用作：  
       - 图像质量判断  
       - 动态调节目标检测权重  
       - 控制系统 gating signal

    ---

    ## 4. 查看输出

    ```bash
    ros2 topic echo /us_stability
    ```