<!--
 * @Author: doumeng 1159898567@qq.com
 * @Date: 2026-01-27 09:00:23
 * @LastEditors: doumeng 1159898567@qq.com
 * @LastEditTime: 2026-01-30 15:51:12
 * @FilePath: /GMAPD_RK3588/Readme.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
此版本为RK3588控制APD成像，并实时调整成像参数的版本，未添加通信功能

# UPDATE
- 上位机通过弹目距离计算开门延迟时间
- 通过距离计算滑动窗宽及阈值
- 通过距离计算DBSCAN和填充空洞的大小
- UDP和PCIE分线程单独发送
- 修改补全方式

# FIX
+ 必须要点参数配置才会推流

# TODO
+ 自动计算目标距离及延迟
+ 根据自动计算的距离更新参数
+ 集成通信线程
