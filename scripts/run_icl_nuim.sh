###
 # @Author: GetOverMassif 164567487@qq.com
 # @Date: 2022-10-11 13:50:58
 # @LastEditors: GetOverMassif 164567487@qq.com
 # @LastEditTime: 2022-10-13 19:14:25
 # @FilePath: /DSP-SLAM/scripts/run_redwood.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 



# ICL-NUIM 数据集中的 associations.txt 中顺序为 FrameID, depth_img, FrameID, rgb_img
# 需要修改为： time, rgb_img, time, depth_img

# PROGRAME_TYPE=run
PROGRAME_TYPE=evo

# PROGRAME_TYPE=$1

if [ "$PROGRAME_TYPE" = "run" ]; then
    CMD=./qsp_slam_rgbd
fi
if [ "$PROGRAME_TYPE" = "evo" ]; then
    CMD=./evo_tool_qsp
fi


# $CMD configs/ICL_NUIM_home_2.yaml
$CMD configs/ICL_NUIM_office_2.yaml

# if [ "$SCENE_TYPE" = "home" ]; then
#     $CMD \
#     Vocabulary/ORBvoc.bin \
#     configs/ICL_NUIM_home.yaml \
#     /media/lj/TOSHIBA/dataset/ICL-NUIM/home/living_room_traj$DATA_NUM\_frei_png \
#     map/icl_nuim/home/$DATA_NUM
# fi

# if [ "$SCENE_TYPE" = "office" ]; then
#     $CMD \
#     Vocabulary/ORBvoc.bin \
#     configs/ICL_NUIM_home.yaml \
#     /media/lj/TOSHIBA/dataset/ICL-NUIM/office/traj$DATA_NUM\_frei_png \
#     map/icl_nuim/office/$DATA_NUM
# fi