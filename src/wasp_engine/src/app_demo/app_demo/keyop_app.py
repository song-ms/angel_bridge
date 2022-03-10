from api_demo.wasp import *


def main():
    wasp = WASP("test_app")

    with KeyOps() as k:
        while(k.esc != True):            
            act = k.getKeyInput()

            # 1. 관절 제어 입력하기
            # 여기에 코드를 입력하세요!
            wasp.setEffort(act)

            # 2. 데이터 업로드
            # 여기에 코드를 입력하세요!
            wasp.uploadData(JOINT_STATE)
            
            for i in range(4):
                k.screen_.addstr(i+2, 0, f'{str(abs(act[i])):4}')
                
            k.screen_.refresh()

    wasp.Terminate()



if __name__ == "__main__":
    main()