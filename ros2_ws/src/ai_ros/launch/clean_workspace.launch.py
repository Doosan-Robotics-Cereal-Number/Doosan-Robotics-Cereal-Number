#!/usr/bin/env python3
"""
clean_workspace.launch.py

RealSense 워크스페이스 추적 노드와 GPT 오케스트레이터를 동시에 실행.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_topic = LaunchConfiguration("depth_topic")
    info_topic = LaunchConfiguration("info_topic")
    color_topic = LaunchConfiguration("color_topic")
    workspace_roi = LaunchConfiguration("workspace_roi_px")
    visualize = LaunchConfiguration("visualize")
    stable_event_topic = LaunchConfiguration("stable_event_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    enable_requests = LaunchConfiguration("enable_requests")
    gpt_model = LaunchConfiguration("gpt_model")
    tts_model = LaunchConfiguration("tts_model")
    audio_voice = LaunchConfiguration("audio_voice")
    audio_speed = LaunchConfiguration("audio_speed")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
                description="Depth 이미지 토픽",
            ),
            DeclareLaunchArgument(
                "info_topic",
                default_value="/camera/camera/aligned_depth_to_color/camera_info",
                description="Depth CameraInfo 토픽",
            ),
            DeclareLaunchArgument(
                "color_topic",
                default_value="/camera/camera/color/image_raw",
                description="컬러 이미지 토픽 (디버그 패널용)",
            ),
            DeclareLaunchArgument(
                "workspace_roi_px",
                default_value="258,18,432,305|438,110,536,251",
                description="워크스페이스 ROI 정의",
            ),
            DeclareLaunchArgument(
                "visualize",
                default_value="true",
                description="OpenCV 시각화 여부",
            ),
            DeclareLaunchArgument(
                "stable_event_topic",
                default_value="/workspace_stable_object",
                description="안정 물체 이벤트 토픽",
            ),
            DeclareLaunchArgument(
                "debug_image_topic",
                default_value="/workspace_debug",
                description="디버그 이미지 퍼블리시 토픽",
            ),
            DeclareLaunchArgument(
                "enable_requests",
                default_value="true",
                description="GPT 요청 실행 여부 (false면 로깅만 수행)",
            ),
            DeclareLaunchArgument(
                "gpt_model",
                default_value="gpt-5",
                description="GPT Realtime 모델 ID (예: gpt-realtime-mini, gpt-4o-realtime-preview, gpt-5 등)",
            ),
            DeclareLaunchArgument(
                "tts_model",
                default_value="gpt-realtime-mini",
                description="TTS용 Realtime 모델 ID (오디오 지원 모델 사용)",
            ),
            DeclareLaunchArgument(
                "audio_voice",
                default_value="echo",
                description="Realtime 음성 ID (예: alloy, charlie, verse 등)",
            ),
            DeclareLaunchArgument(
                "audio_speed",
                default_value="1.2",
                description="Realtime 음성 재생 속도 (0.25~4.0, 1.0=기본)",
            ),
            Node(
                package="ai_ros",
                executable="realsense_node",
                name="workspace_tracker",
                output="screen",
                parameters=[
                    {
                        "depth_topic": depth_topic,
                        "info_topic": info_topic,
                        "color_topic": color_topic,
                        "workspace_roi_px": workspace_roi,
                        "visualize": visualize,
                        "publish_debug_image": True,
                        "debug_image_topic": debug_image_topic,
                        "stable_event_topic": stable_event_topic,
                    }
                ],
            ),
            Node(
                package="ai_ros",
                executable="cleanup_orchestrator_node",
                name="cleanup_orchestrator",
                output="screen",
                parameters=[
                    {
                        "stable_event_topic": stable_event_topic,
                        "debug_image_topic": debug_image_topic,
                        "enable_requests": enable_requests,
                        "gpt_model": gpt_model,
                        "tts_model": tts_model,
                        "audio_voice": audio_voice,
                        "audio_speed": audio_speed,

                    }
                ],
                arguments=["--ros-args", "--log-level", "cleanup_orchestrator:=debug"],

            ),
        ]
    )
