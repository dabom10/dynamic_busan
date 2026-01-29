#!/usr/bin/env python3
"""
Bartender Complete Launch File
- MariaDB Node
- Query Node
- Shake Node (optional)
"""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Bartender 패키지의 모든 노드를 실행하는 launch 파일"""

    # .env 파일 로드 (python-dotenv 사용)
    try:
        from dotenv import load_dotenv
        # 절대 경로 사용 (ROS2 launch에서 __file__ 경로가 달라질 수 있음)
        env_path = Path('/home/chans/dynamic_busan/.env')
        print(f"[DEBUG] Loading .env from: {env_path}")
        print(f"[DEBUG] .env file exists: {env_path.exists()}")
        result = load_dotenv(dotenv_path=env_path, override=True)
        print(f"[DEBUG] .env file loaded: {result}")
    except ImportError:
        print("Warning: python-dotenv not installed. Using system environment variables only.")
    except Exception as e:
        print(f"Warning: Could not load .env file: {e}")

    # 환경 변수에서 DB 설정 읽기
    db_host = os.environ.get('DB_HOST', 'localhost')
    db_port = int(os.environ.get('DB_PORT', '3306'))
    db_user = os.environ.get('DB_USER', 'root')
    db_password = os.environ.get('DB_PASSWORD', '')
    db_name = os.environ.get('DB_NAME', 'test')

    # 디버깅: 로드된 값 출력
    print("="*50)
    print("[DEBUG] DB Configuration Loaded:")
    print(f"  DB_HOST: {db_host}")
    print(f"  DB_PORT: {db_port}")
    print(f"  DB_USER: {db_user}")
    print(f"  DB_NAME: {db_name}")
    print("="*50)

    # Launch 인자 선언 (환경 변수 값을 기본값으로 사용)
    db_host_arg = DeclareLaunchArgument(
        'db_host',
        default_value=db_host,
        description='MariaDB host address'
    )

    db_port_arg = DeclareLaunchArgument(
        'db_port',
        default_value=str(db_port),
        description='MariaDB port'
    )

    db_user_arg = DeclareLaunchArgument(
        'db_user',
        default_value=db_user,
        description='MariaDB username'
    )

    db_password_arg = DeclareLaunchArgument(
        'db_password',
        default_value=db_password,
        description='MariaDB password'
    )

    db_name_arg = DeclareLaunchArgument(
        'db_name',
        default_value=db_name,
        description='Database name'
    )

    # MariaDB 노드
    mariadb_node = Node(
        package='bartender',
        executable='db',
        name='mariadb_node',
        output='screen',
        parameters=[{
            'db_host': db_host,
            'db_port': db_port,
            'db_user': db_user,
            'db_password': db_password,
            'db_name': db_name,
        }],
        emulate_tty=True,
    )

    # Query 노드 (대화형 모드)
    query_node = Node(
        package='bartender',
        executable='query',
        name='query_node',
        output='screen',
        emulate_tty=True,
    )

    # Shake 노드
    shake_node = Node(
        package='bartender',
        executable='shake',
        name='shake_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        # Launch 인자들
        db_host_arg,
        db_port_arg,
        db_user_arg,
        db_password_arg,
        db_name_arg,

        # 노드들
        mariadb_node,
        query_node,
        shake_node,
    ])
