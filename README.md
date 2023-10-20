# mk_0
KHU robotics club Hyper's summer project.

## 개발 환경

**OS** : Ubuntu 20.04

**ROS2-distro** : foxy

**Detection Model** : 

## 사용 방법

1. `<워크스페이스>/src/` 아래에 리포지토리를 clone.
    
    ```bash
    $ cd <workspace>/src
    $ git clone https://github.com/kodogyu/mk_0.git
    ```
    
2. `<워크스페이스>`에서 빌드.
    
    ```bash
    $ cd ../
    $ colcon build
    ```
    
3. 워크스페이스의 `setup.bash` 파일 실행.
    
    ```bash
    $ source <workspace>/install/setup.bash
    ```
    
4. 이제 `mk_0` 패키지를 사용할 수 있습니다.
    
    ```bash
    $ ros2 run mk_0 image_processor
    $ ros2 run mk_0 teleop_mk_0
    ```

### 시연 영상
[mk_0_gazebo](https://github.com/kodogyu/mk_0_gazebo) 패키지에서 수정된 turtlebot3 waffle 모델을 사용하여 시뮬레이션하였습니다.

![mk_0 demo](images/mk_0%20demo.gif)
