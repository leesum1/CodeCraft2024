import concurrent.futures
import json
import os
import subprocess
import time
from typing import NamedTuple
from tqdm import tqdm

# 获取当前脚本所在目录的上一层目录
HOME_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDK_PATH = '/home/leesum/Documents/huawei_soft2024/linux'

JUGDE_PROG = os.path.join(SDK_PATH, 'SemiFinalJudge')

semi_maps_list = [
    'maps41/map1.txt',
    'maps41/map2.txt',
    'maps41/map3.txt',
]


def rm_replay_dir():
    # 删除 replay 文件夹
    replay_dir = os.path.join(HOME_PATH, 'script/replay')
    if os.path.exists(replay_dir):
        for file in os.listdir(replay_dir):
            os.remove(os.path.join(replay_dir, file))
        os.rmdir(replay_dir)


def get_preliminary_command(map_path, rand_seed, target_program, f_time=0):
    # $(JUGDE_PROG) -s $(RAND_SEED) -l NONE  -m $(MAP_PATH)/$(MAP_SEL)  $(TARGET)
    preliminary_command = f"{JUGDE_PROG} -s {rand_seed} -f {f_time} -l NONE -m {map_path} {target_program}"
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


# {"robot_num":14,"ship_num":1,"ship_capacity":60,"total_goods_money":176257,"goted_goods_money":149426,"selled_goods_money":141505,"total_goods_num":2561,"goted_goods_num":1707,"selled_goods_num":1619}
class MapInfo(NamedTuple):
    robot_num: int
    ship_num: int
    ship_capacity: int
    total_goods_money: int
    goted_goods_money: int
    selled_goods_money: int
    total_goods_num: int
    goted_goods_num: int
    selled_goods_num: int


def parse_map_info_from_stderr(stderr) -> MapInfo:
    lines = stderr.split('\n')
    map_info = MapInfo(0, 0, 0, 0, 0, 0, 0, 0, 0)
    for line in lines:
        try:
            # 尝试将每一行解析为JSON
            data = json.loads(line)
            map_info = MapInfo(data.get("robot_num", 0), data.get("ship_num", 0), data.get("ship_capacity", 0),
                               data.get("total_goods_money", 0), data.get("goted_goods_money", 0),
                               data.get("selled_goods_money", 0), data.get("total_goods_num", 0),
                               data.get("goted_goods_num", 0), data.get("selled_goods_num", 0))
        except json.JSONDecodeError:
            # 如果当前行不能被解析为JSON，继续下一行
            continue
    return map_info


def build_program():
    # 将 config.h 中的 #define LOG_ENABLE 注释掉，使用 Linux 工具
    run_command(f"sed -i 's/^#define LOG_ENABLE/\/\/#define LOG_ENABLE/' {HOME_PATH}/config.h")
    run_command(f'make -C {HOME_PATH}')
    # 恢复 config.h 中的 #define LOG_ENABLE 注释，使用 Linux 工具
    run_command(f"sed -i 's/^\/\/#define LOG_ENABLE/#define LOG_ENABLE/' {HOME_PATH}/config.h")


# 定义一个函数来运行命令并返回输出
def run_command(cmd: str):
    process = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    try:
        stdout, stderr = process.communicate(timeout=150)  # 设置超时时间为10秒
    except subprocess.TimeoutExpired:
        process.kill()  # 如果命令超时，杀死进程
        stdout, stderr = process.communicate()
    return stdout, stderr


def statistic_ship_capcity():
    import random
    build_program()
    statistic_cmd_list = list()
    target_program = os.path.join(HOME_PATH, 'build', 'main')
    count = 0
    results = list()
    map_info_list = list()
    rand_seed_list = list()
    while count < 10:
        random_uint32 = random.randint(0, 0xfffffffe)
        map_path = random.choice(semi_maps_list)
        cmd = get_preliminary_command(os.path.join(HOME_PATH, map_path), random_uint32, target_program, f_time=30)
        statistic_cmd_list.append(cmd)
        rand_seed_list.append(random_uint32)
        count += 1
    print(f"Total count: {count}")
    with concurrent.futures.ThreadPoolExecutor(max_workers=32) as executor:
        results = list(tqdm(executor.map(run_command, statistic_cmd_list)))

    print(f"Total results: {len(results)}")
    for result in results:
        stdout, stderr = result
        map_info = parse_map_info_from_stderr(stderr.decode('utf-8'))
        map_info_list.append(map_info)

    # 写入文件中
    with open(os.path.join(HOME_PATH, 'mapinfo.txt'), 'w') as f:
        f.write(
            "seed,ship_capacity,robot_num,ship_num,total_goods_money,goted_goods_money,selled_goods_money,total_goods_num,goted_goods_num,selled_goods_num\n")
        for seed, map_info_item in zip(rand_seed_list, map_info_list):
            f.write(
                f"{seed},{map_info_item.ship_capacity},{map_info_item.robot_num},{map_info_item.ship_num},{map_info_item.total_goods_money},{map_info_item.goted_goods_money},{map_info_item.selled_goods_money},{map_info_item.total_goods_num},{map_info_item.goted_goods_num},{map_info_item.selled_goods_num}\n")


# statistic_ship_capcity()

if __name__ == '__main__':
    rm_replay_dir()
    target_program = os.path.join(HOME_PATH, 'build', 'main')
    # 721423:82
    # 1421323:56
    # 3291948642: 50
    # 1234415677: 100
    # 1298200600: 60
    test_cmds = [get_preliminary_command(os.path.join(HOME_PATH, map_path), 1298200600, target_program) for map_path in
                 semi_maps_list]

    build_program()

    repeat_size = 5
    # 创建一个新的命令列表，其中每个命令都重复 5 次
    test_cmds_repeated = [cmd for cmd in test_cmds for _ in range(repeat_size)]
    semi_maps_list_repeated = [
        map_path for map_path in semi_maps_list for _ in range(repeat_size)]
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
    map_info_dict = dict()
    # 打印所有命令的结果
    for map, result in zip(semi_maps_list_repeated, results):
        stdout, stderr = result
        score = parse_score_from_stderr(stdout.decode('utf-8'))
        map_info = parse_map_info_from_stderr(stderr.decode('utf-8'))

        # 取最高分,最低分，平均分
        if scores_max_dict.get(map) == None and score != 0:
            scores_max_dict[map] = score
            scores_min_dict[map] = score
            scores_sum_dict[map] = score
            map_info_dict[map] = map_info
            counts_dict[map] = 1
        elif score != 0:
            scores_max_dict[map] = max(scores_max_dict[map], score)
            scores_min_dict[map] = min(scores_min_dict[map], score)
            scores_sum_dict[map] += score
            counts_dict[map] += 1

    for map, map_info in map_info_dict.items():
        print(
            f"{map} : robot_num:{map_info.robot_num},ship_num:{map_info.ship_num},ship_capacity:{map_info.ship_capacity},total_goods_money:{map_info.total_goods_money},goted_goods_money:{map_info.goted_goods_money},selled_goods_money:{map_info.selled_goods_money},total_goods_num:{map_info.total_goods_num},goted_goods_num:{map_info.goted_goods_num},selled_goods_num:{map_info.selled_goods_num}")
    print("Max score")
    for map, score in scores_max_dict.items():
        print(f"{map} : {score}")
    print(
        f"Total maps: {len(scores_max_dict)} Total score: {sum(scores_max_dict.values())},average score: {sum(scores_max_dict.values()) / len(scores_max_dict)}")
    print("Min score")
    for map, score in scores_min_dict.items():
        print(f"{map} : {score}")
    print(
        f"Total maps: {len(scores_min_dict)} Total score: {sum(scores_min_dict.values())},average score: {sum(scores_min_dict.values()) / len(scores_min_dict)}")

    # 计算平均分
    scores_avg_dict = {
        map: scores_sum_dict[map] / counts_dict[map] for map in scores_sum_dict}
    print("Average score")
    for map, score in scores_avg_dict.items():
        print(f"{map} : {score}")
    print(
        f"Total maps: {len(scores_avg_dict)} Total score: {sum(scores_avg_dict.values())},average score: {sum(scores_avg_dict.values()) / len(scores_avg_dict)}")
