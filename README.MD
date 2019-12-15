不同电脑会连接在不同的SRB设备上，这些配置需要 ~/srb_config.launch给出
安装这个包以后，需要将srb_config.launch复制到用户根目录并且根据SRB名称进行配置。

对于有雷达的SRB，运行
`roslaunch srb_demo le_demo_lidar_edel.launch `
显示电脑要和AGV电脑并网，并且配置ROS主机在AGV上，
然后需要运行：
`roslaunch srb_demo lkc_demo_lidar_key_control.launch `
