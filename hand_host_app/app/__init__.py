"""通用灵巧手上位机桌面应用包。

`app` 包的子模块分工如下：
    config            — 用户偏好的轻量持久化（只记 active_hand）
    platform_compat   — Qt / matplotlib 跨平台兼容补丁
    main_window       — 主窗口与导航、手型切换
    hands.*           — 手型包加载、注册表、运行时上下文
    models.*          — 全局共享数据模型（HandState、关节配置、触觉布局）
    services.*        — 通用后台服务（适配器包装、触觉采集、MuJoCo、视频、力控等）
    widgets.*         — 功能页面（连接/关节/标定/触觉/遥操作/记录/手势/手型管理）
    resources.hands.* — 已安装手型包（manifest.json + 适配器 + XML 等）
"""
