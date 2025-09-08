import sys
sys.path.append("./")
import time
import select

from multiprocessing import Array, Process, Lock, Event, Semaphore

from my_robot.test_robot import TestRobot, condition

from utils.time_scheduler import TimeScheduler
from utils.robot_worker import RobotWorker
from utils.data_handler import is_enter_pressed

from data.collect_any import CollectAny


if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR

    num_episode = 10
    avg_collect_time = 0

    for i in range(num_episode):
        is_start = False
        
        # 重置进程
        time_lock = Event()
        start_event = Event()
        finish_event = Event()
        start_episode = i
        robot_process = Process(target=RobotWorker, args=(TestRobot, start_episode, time_lock, start_event, finish_event, "robot_worker"))
        time_scheduler = TimeScheduler([time_lock], time_freq=10) # 可以给多个进程同时上锁
        
        robot_process.start()
        while not is_start:
            time.sleep(0.01)
            if is_enter_pressed():
                is_start = True
                start_event.set()
            else:
                time.sleep(1)
    
        time_scheduler.start()
        while is_start:
            time.sleep(0.01)
            if is_enter_pressed():
                finish_event.set()  
                time_scheduler.stop()  
                is_start = False
        
        # 销毁多进程
        if robot_process.is_alive():
            robot_process.join()
            robot_process.close()
        
        avg_collect_time += time_scheduler.real_time_average_time_interval
    
    collection = CollectAny(condition=condition,start_episode=0)
    avg_collect_time /= num_episode
    extra_info = {}
    extra_info["avg_time_interval"] = avg_collect_time
    collection.add_extra_condition_info(extra_info)