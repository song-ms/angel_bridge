from time import sleep
from api_demo.wasp import *


def main():
    wasp = WASP("test_app")

    while(1):
        # 1. 관절 제어 입력하기
        # 여기에 코드를 입력하세요!
        wasp.setEffort([-1000, 500, 1000, -500])

        # 2. 각도 정보 읽기
        # 여기에 코드를 입력하세요!
        cur_pos = wasp.getPosition()
        
        PrintPosition(cur_pos)
        sleep(1) # 1초 대기
        wasp.Release() # 제어 입력 해제

        # 3. 데이터 업로드
        # 여기에 코드를 입력하세요!


    wasp.Terminate()



