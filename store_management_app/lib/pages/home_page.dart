import 'package:flutter/material.dart';
import 'package:lottie/lottie.dart';
import '../services/ros2_service.dart';
import '../widgets/primary_action_button.dart';
import 'cleaning_detail_page.dart';

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  final ROS2Service _ros2Service = ROS2Service();

  @override
  void initState() {
    super.initState();
    // 앱 시작 시 ROS2 연결
    _ros2Service.connect();
  }

  @override
  void dispose() {
    _ros2Service.disconnect();
    super.dispose();
  }

  void _handleSubscribeCleaning() {
    _ros2Service.subscribeToCleaningTopic();

    // Cleaning 상세 페이지로 이동
    Navigator.push(
      context,
      MaterialPageRoute(
        builder: (context) => CleaningDetailPage(
          ros2Service: _ros2Service,
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF121212),
      body: SafeArea(
        child: Container(
          width: double.infinity,
          height: double.infinity,
          clipBehavior: Clip.antiAlias,
          decoration: const BoxDecoration(color: Color(0xFF121212)),
          child: Stack(
            children: [
              // 중앙 텍스트
              const Positioned(
                left: 0,
                right: 0,
                top: 100,
                child: Text(
                  '두산이가 1개의 알림을 보냈어요.',
                  textAlign: TextAlign.center,
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 26,
                    fontFamily: 'Pretendard',
                    fontWeight: FontWeight.w700,
                    height: 1.33,
                  ),
                ),
              ),

              // 확인하기 버튼
              Positioned(
                left: 115,
                top: 160,
                child: PrimaryActionButton(
                  label: '확인하기',
                  onPressed: _handleSubscribeCleaning,
                  width: 160,
                ),
              ),

              // 중앙 원형 애니메이션/이미지
              Positioned(
                left: -61,
                top: 480,
                child: SizedBox(
                  width: 512,
                  height: 512,
                  child: Lottie.asset(
                    'assets/animations/voice-ai-animation-listen.json',
                    fit: BoxFit.contain,
                    repeat: true,
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

