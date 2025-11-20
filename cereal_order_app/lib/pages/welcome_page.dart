import 'package:flutter/material.dart';
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

  @override
  void initState() {
    super.initState();
    print('[WelcomePage] ========== initState 호출 ==========');
    _initializeService();
    print('[WelcomePage] ✅ initState 완료');
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
  }

  @override
  void dispose() {
    print('[WelcomePage] ========== dispose() 호출 ==========');
    try {
      _statusService.stop();
      print('[WelcomePage] ✅ statusService.stop() 완료');
    } catch (e, stackTrace) {
      print('[WelcomePage] ❌ statusService.stop() 중 에러: $e');
      print('[WelcomePage] 스택 트레이스: $stackTrace');
    }
    
    try {
      _statusService.dispose();
      print('[WelcomePage] ✅ statusService.dispose() 완료');
    } catch (e, stackTrace) {
      print('[WelcomePage] ❌ statusService.dispose() 중 에러: $e');
      print('[WelcomePage] 스택 트레이스: $stackTrace');
    }
    
    super.dispose();
    print('[WelcomePage] ✅ dispose() 완료');
  }

  @override
  Widget build(BuildContext context) {
    print('[WelcomePage] ========== build() 호출 ==========');
    print('[WelcomePage] context: $context');
    return Scaffold(
      body: Stack(
        children: [
          // 베이스 배경 그라디언트
          Container(
            width: double.infinity,
            height: double.infinity,
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.bottomLeft,
                end: Alignment.topRight,
                colors: [
                  const Color(0xFFDFE9FF), // 좌측 하단
                  const Color(0xFFFFFFFF), // 우측 상단
                ],
              ),
            ),
          ),
          // 첫 번째 그라디언트 레이어
          Container(
            width: double.infinity,
            height: double.infinity,
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
                colors: [
                  const Color(0xFFFFFFFF).withOpacity(0.0),
                  const Color(0xFFE8E4FF).withOpacity(0.3),
                ],
              ),
            ),
          ),
          // 두 번째 그라디언트 레이어
          Container(
            width: double.infinity,
            height: double.infinity,
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
                colors: [
                  const Color(0xFFFFFFFF),
                  const Color(0xFFCEC9FF).withOpacity(0.4),
                ],
              ),
            ),
          ),
          // 세 번째 그라디언트 레이어
          Container(
            width: double.infinity,
            height: double.infinity,
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
                colors: [
                  const Color(0xFFFFFFFF),
                  const Color(0xFFC7D8FC).withOpacity(0.5),
                ],
              ),
            ),
          ),
          // 컨텐츠
          Padding(
            padding: const EdgeInsets.only(left: 300.0),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                // 메인 제목
                const Text(
                  '반가워요\n좋은 오후에요!',
                  textAlign: TextAlign.left,
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
                    // 메뉴에서 선택하기 버튼 (왼쪽)
                    GestureDetector(
                      onTap: () {
                        print('[WelcomePage] 메뉴에서 선택하기 버튼 클릭됨');
                        Navigator.pushNamed(context, '/cereal-selection');
                      },
                      child: Container(
                        width: 520,
                        height: 240,
                        clipBehavior: Clip.antiAlias,
                        decoration: ShapeDecoration(
                          color: Colors.white,
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(30),
                          ),
                          shadows: const [
                            BoxShadow(
                              color: Color(0x14CCCCCC),
                              blurRadius: 180,
                              offset: Offset(0, 3),
                              spreadRadius: 40,
                            )
                          ],
                        ),
                        child: Column(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            Image.asset(
                              'assets/images/option-icon-menu-selection.png',
                              width: 100,
                              height: 100,
                            ),
                            const SizedBox(height: 16),
                            Text(
                              '메뉴에서 선택하기',
                              textAlign: TextAlign.center,
                              style: TextStyle(
                                color: const Color(0xFF121212),
                                fontSize: 34,
                                fontFamily: 'Pretendard',
                                fontWeight: FontWeight.w700,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                    const SizedBox(width: 100), // 버튼 간격
                    // 대화로 주문하기 버튼 (오른쪽)
                    GestureDetector(
                      onTap: () {
                        print('[WelcomePage] 음성 주문 시작 버튼 클릭됨');
                        Navigator.pushNamed(context, '/voice-order');
                      },
                      child: Container(
                        width: 520,
                        height: 240,
                        clipBehavior: Clip.antiAlias,
                        decoration: ShapeDecoration(
                          color: Colors.white,
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(30),
                          ),
                          shadows: const [
                            BoxShadow(
                              color: Color(0x14CCCCCC),
                              blurRadius: 180,
                              offset: Offset(0, 3),
                              spreadRadius: 40,
                            )
                          ],
                        ),
                        child: Column(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            Image.asset(
                              'assets/images/option-icon-voice-selection.png',
                              width: 100,
                              height: 100,
                            ),
                            const SizedBox(height: 16),
                            Text(
                              '대화로 주문하기',
                              textAlign: TextAlign.center,
                              style: TextStyle(
                                color: const Color(0xFF121212),
                                fontSize: 34,
                                fontFamily: 'Pretendard',
                                fontWeight: FontWeight.w700,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                  ],
                ),
              ],
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
