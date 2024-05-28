
object_class_table = {
"person":[0],"bicycle":[1],"car":[2],"motorcycle":[3],"airplane":[4],"bus":[5],
"train":[6],"truck":[7],"boat":[8],"traffic light":[9],"fire hydrant":[10],
"stop sign":[11],"parking meter":[12],"bench":[13],"bird":[14],"cat":[15],
"dog":[16],"horse":[17],"sheep":[18],"cow":[19],"elephant":[20],
"bear":[21],"zebra":[22],"giraffe":[23],"backpack":[24],"umbrella":[25],
"handbag":[26],"tie":[27],"suitcase":[28],"frisbee":[29],"skis":[30],
"snowboard":[31],"sports ball":[32],"kite":[33],"baseball bat":[34],"baseball glove":[35],
"skateboard":[36],"surfboard":[37],"tennis racket":[38],"bottle":[39],"wine glass":[40],
"cup":[41],"fork":[42],"knife":[43],"spoon":[44],"bowl":[45],
"banana":[46],"apple":[47],"sandwich":[48],"orange":[49],"broccoli":[50],
"carrot":[51],"hot dog":[52],"pizza":[53],"donut":[54],"cake":[55],
"chair":[56, 57],"couch":[13,56,57],"potted plant":[58],"bed":[59],"dining table":[60],
"toilet":[61],"monitor":[62],"laptop":[63],"mouse":[64],"remote":[65],
"keyboard":[66],"cell phone":[67],"microwave":[68],"oven":[69],"toaster":[70],
"sink":[71],"refrigerator":[72],"book":[73],"clock":[74],"vase":[75],
"scissors":[76],"teddy bear":[77],"hair drier":[78],"toothbrush":[79]
}

# object_class_table = {
#     "car": [2],
#     "bench": [13], # 板凳
#     "backpack": [24], # 背包
#     "chair": [56, 57], # 椅子，沙发
#     "bottle": [39], # 瓶子
#     "wine glass": [40], # 酒杯
#     "cup": [41], # 杯子
#     "bowl": [45], # 碗
#     "banana": [46], "apple": [47], "orange": [49],
#     "potted plant": [58], # 盆栽植物
#     "bed": [59],
#     "dining table": [60],
#     "monitor": [62],
#     "laptop": [63],
#     "mouse": [64],
#     "keyboard": [66],
#     "microwave": [68], "oven":[69], "toaster": [70], "refrigerator": [72],
#     "book": [73], "clock": [74], "vase": [75], "teddy bear": [77]
# }


object_classes = list(object_class_table.keys())

object_classes_redwood = ["bench", "chair"]

object_classes_freiburg = ["car"]

object_classes_icl_nuim = ["bench", "chair", "bed", "dining table", "refrigerator", \
                           "vase", "cup", "bowl", "monitor"]

object_classes_on_ground = ["bench", "chair", "potted plant", "bed", "dining table", "refrigerator"]
object_classes_on_table = [c for c in object_classes if c not in object_classes_on_ground ]