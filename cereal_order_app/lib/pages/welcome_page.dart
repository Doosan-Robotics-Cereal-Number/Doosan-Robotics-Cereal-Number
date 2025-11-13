import 'package:flutter/material.dart';
import 'dart:async';
import '../services/status_service.dart';
import '../services/status_service_factory.dart';
import '../config/app_config.dart';

class WelcomePage extends StatefulWidget {
  const WelcomePage({super.key});

  @override
  State<WelcomePage> createState() => _WelcomePageState();
}

class _WelcomePageState extends State<WelcomePage> {
  // 상태 서비스
  late StatusService _statusService;
  StreamSubscription<String>? _voiceOrderSubscription;

  @override
  void initState() {
    super.initState();
    _initializeService();
  }

  /// 서비스 초기화
  void _initializeService() {
    // 설정에 따라 서비스 생성
    _statusService = StatusServiceFactory.create(
      AppConfig.currentServiceType,
      ros2ServerUrl: AppConfig.ros2ServerUrl,
      ros2TopicName: AppConfig.ros2TopicName,
      ros2TopicType: AppConfig.ros2TopicType,
    );

    // 서비스 시작
    _statusService.start();

    // 음성 주문 시작 토픽 구독
    _voiceOrderSubscription = _statusService.voiceOrderStartStream.listen((message) {
      print('[WelcomePage] 음성 주문 시작: $message');
      if (mounted) {
        // 음성 주문 페이지로 이동
        Navigator.pushNamed(context, '/voice-order');
      }
    });
  }

  @override
  void dispose() {
    _voiceOrderSubscription?.cancel();
    _statusService.stop();
    _statusService.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      body: Row(
          children: [
            // 왼쪽 컨텐츠
            Expanded(
              child: Padding(
                padding: const EdgeInsets.only(left: 120.0),
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    // 메인 제목
                    const Text(
                      '반가워요\n좋은 오후에요!',
                      style: TextStyle(
                        fontSize: 100,
                        fontWeight: FontWeight.bold,
                        color: Color(0xFF121212),
                        height: 1.333, // 160px line height (160 / 120 = 1.333)
                      ),
                    ),
                    const SizedBox(height: 64),
                    // 버튼들 (Row로 나란히 배치)
                    Row(
                      children: [
                        // 시리얼 주문하기 버튼
                        SizedBox(
                          width: 600,
                          height: 100,
                          child: ElevatedButton(
                            onPressed: () {
                              print('[WelcomePage] 메뉴에서 선택하기 버튼 클릭됨');
                              Navigator.pushNamed(context, '/cereal-selection');
                            },
                            style: ElevatedButton.styleFrom(
                              backgroundColor: const Color(0xFF0064FF),
                              foregroundColor: Colors.white,
                              padding: const EdgeInsets.symmetric(
                                horizontal: 40,
                                vertical: 16,
                              ),
                              shape: RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(25), // 더 둥근 모서리
                              ),
                              elevation: 2,
                            ),
                            child: const Text(
                              '메뉴에서 선택하기',
                              style: TextStyle(
                                fontSize: 32,
                                fontWeight: FontWeight.w500,
                              ),
                            ),
                          ),
                        ),
                        const SizedBox(width: 24), // 버튼 간격
                        // 음성으로 주문하기 버튼
                        SizedBox(
                          width: 600,
                          height: 100,
                          child: ElevatedButton(
                            onPressed: () {
                              print('[WelcomePage] 음성 주문 시작 버튼 클릭됨');
                              Navigator.pushNamed(context, '/voice-order');
                            },
                            style: ElevatedButton.styleFrom(
                              backgroundColor: const Color(0xFF0064FF),
                              foregroundColor: Colors.white,
                              padding: const EdgeInsets.symmetric(
                                horizontal: 40,
                                vertical: 16,
                              ),
                              shape: RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(25),
                              ),
                              elevation: 2,
                            ),
                            child: const Text(
                              '대화로 주문하기',
                              style: TextStyle(
                                fontSize: 32,
                                fontWeight: FontWeight.w500,
                              ),
                            ),
                          ),
                        ),
                      ],
                    ),
                  ],
                ),
              ),
            ),
            // 우측 이미지
            // Padding(
            //   padding: const EdgeInsets.only(right: 120.0),
            //   child: Image.asset(
            //     'assets/images/welcome-3d.png',
            //     width: 600,
            //     height: 600,
            //     fit: BoxFit.contain,
            //     errorBuilder: (context, error, stackTrace) {
            //       return const SizedBox(width: 600, height: 600);
            //     },
            //   ),
            // ),
          ],
        ),
    );
  }
}


