import os
import json
import subprocess
import concurrent.futures
import time
from tqdm import tqdm

# 获取当前脚本所在目录的上一层目录
HOME_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SDK_PATH = '/home/leesum/Documents/huawei_soft2024/LinuxReleasev1.2.part1/LinuxReleasev1.2/LinuxReleasev1.2'
SDK_PATH2 = '/home/leesum/Documents/huawei_soft2024/LinuxRelease.part01/LinuxRelease'
SDK_PATH3 = '/home/leesum/Documents/huawei_soft2024/LinuxReleasev2.0'


JUGDE_PROG = os.path.join(SDK_PATH3, 'PreliminaryJudge')

preliminary_maps_list = [
    'maps/map1.txt',
    'maps/map2.txt',
    'maps/map37.txt',
    'maps/map38.txt',
    'maps/map39.txt',
    'maps/map310.txt',
    'maps/map311.txt',
    'maps/map312.txt',
    'maps/map313.txt',
    'maps323/map1.txt',
    'maps323/map2.txt',
]


def rm_replay_dir():
    # 删除 replay 文件夹
    replay_dir = os.path.join(HOME_PATH, 'script/replay')
    if os.path.exists(replay_dir):
        for file in os.listdir(replay_dir):
            os.remove(os.path.join(replay_dir, file))
        os.rmdir(replay_dir)    
    


def get_preliminary_command(map_path,rand_seed,target_program):
    # $(JUGDE_PROG) -s $(RAND_SEED) -l NONE  -m $(MAP_PATH)/$(MAP_SEL)  $(TARGET)
    preliminary_command = f"{JUGDE_PROG} -s {rand_seed} -f 0 -l NONE -m {map_path} {target_program}"
    return preliminary_command
    


def parse_score_from_stderr(stderr):
    lines = stderr.split('\n')
    for line in lines:
        try:
            # 尝试将每一行解析为JSON
            data = json.loads(line)

            # 检查"status"字段是否为"Successful"
            if data.get("status") == "Successful":
                # 如果是，返回"score"字段
                return data.get("score")
        except json.JSONDecodeError:
            # 如果当前行不能被解析为JSON，继续下一行
            continue

    return 0


# 定义一个函数来运行命令并返回输出
# 定义一个函数来运行命令并返回输出
def run_command(cmd:str):
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    try:
        stdout, stderr = process.communicate(timeout = 60)  # 设置超时时间为10秒
    except subprocess.TimeoutExpired:
        process.kill()  # 如果命令超时，杀死进程
        stdout, stderr = process.communicate()
    return stdout, stderr




if __name__ == '__main__':
    rm_replay_dir()
    target_program = os.path.join(HOME_PATH,'build','main')
    test_cmds = [get_preliminary_command(os.path.join(HOME_PATH,map_path),1421323,target_program) for map_path in preliminary_maps_list]

    
    repeat_size = 5
    # 创建一个新的命令列表，其中每个命令都重复 5 次
    test_cmds_repeated = [cmd for cmd in test_cmds for _ in range(repeat_size)]
    preliminary_maps_list_repeated = [map_path for map_path in preliminary_maps_list for _ in range(repeat_size)]
    

    start_time = time.time()
    # 使用ThreadPoolExecutor并行运行所有命令
    with concurrent.futures.ThreadPoolExecutor(max_workers=12) as executor:
        results = list(tqdm(executor.map(run_command, test_cmds_repeated)))
    end_time = time.time()
    print(f"Total time: {end_time - start_time} seconds")

    
    scores_max_dict = {}
    scores_min_dict = {}
    scores_sum_dict = {}
    counts_dict = {}
    
    # 打印所有命令的结果
    for map, result in zip(preliminary_maps_list_repeated, results):
        stdout, stderr = result
        score = parse_score_from_stderr(stdout.decode('utf-8'))
        
        # 取最高分,最低分，平均分
        if scores_max_dict.get(map) == None and score != 0:
            scores_max_dict[map] = score
            scores_min_dict[map] = score
            scores_sum_dict[map] = score
            counts_dict[map] = 1
        elif score != 0:
            scores_max_dict[map] = max(scores_max_dict[map],score)
            scores_min_dict[map] = min(scores_min_dict[map],score)
            scores_sum_dict[map] += score
            counts_dict[map] += 1
            
    
    print("Max score")
    for map,score in scores_max_dict.items():
        print(f"{map} : {score}")
    print(f"Total maps: {len(scores_max_dict)} Total score: {sum(scores_max_dict.values())},average score: {sum(scores_max_dict.values())/len(scores_max_dict)}")
    print("Min score")
    for map,score in scores_min_dict.items():
        print(f"{map} : {score}")
    print(f"Total maps: {len(scores_min_dict)} Total score: {sum(scores_min_dict.values())},average score: {sum(scores_min_dict.values())/len(scores_min_dict)}")
    
    # 计算平均分
    scores_avg_dict = {map: scores_sum_dict[map] / counts_dict[map] for map in scores_sum_dict}
    print("Average score")
    for map,score in scores_avg_dict.items():
        print(f"{map} : {score}")
    print(f"Total maps: {len(scores_avg_dict)} Total score: {sum(scores_avg_dict.values())},average score: {sum(scores_avg_dict.values())/len(scores_avg_dict)}")
        

