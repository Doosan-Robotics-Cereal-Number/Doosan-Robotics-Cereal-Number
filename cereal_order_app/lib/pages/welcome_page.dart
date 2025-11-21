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
  StreamSubscription<String>? _voiceOrderReadySubscription;  // 음성 주문 준비 구독
  bool _hasClickedVoiceOrderButton = false;  // 버튼 클릭 여부
  bool _hasNavigatedToVoiceOrder = false;  // 중복 이동 방지

  @override
  void initState() {
    super.initState();
    print('[WelcomePage] ========== initState 호출 ==========');
    _initializeService();
    print('[WelcomePage] ✅ initState 완료');
  }

  /// 서비스 초기화
  void _initializeService() async {
    // 설정에 따라 서비스 생성
    _statusService = StatusServiceFactory.create(
      AppConfig.currentServiceType,
      ros2ServerUrl: AppConfig.ros2ServerUrl,
      ros2TopicName: AppConfig.ros2TopicName,
      ros2TopicType: AppConfig.ros2TopicType,
    );

    // 서비스 시작
    await _statusService.start();
    
    // 음성 주문 준비 토픽 구독 시작
    _subscribeToVoiceOrderReady();
  }

  /// 음성 주문 준비 토픽 구독
  /// "ready" 메시지를 받으면 자동으로 voice_order_page로 이동
  void _subscribeToVoiceOrderReady() {
    print('[WelcomePage] ========== 음성 주문 준비 토픽 구독 시작 ==========');
    
    // 이미 구독 중이면 취소
    _voiceOrderReadySubscription?.cancel();
    
    // 음성 주문 준비 스트림 구독
    _voiceOrderReadySubscription = (_statusService as dynamic).voiceOrderReadyStream.listen((readyMessage) {
      print('[WelcomePage] 음성 주문 준비 메시지 수신: "$readyMessage"');
      
      // "ready" 메시지를 받으면 자동으로 voice_order_page로 이동
      if (mounted && !_hasNavigatedToVoiceOrder && readyMessage.toLowerCase().trim() == 'ready') {
        print('[WelcomePage] ✅ "ready" 메시지 확인! voice_order_page로 자동 이동');
        _hasNavigatedToVoiceOrder = true;
        
        // voice_order_page로 자동 이동
        Navigator.pushNamed(context, '/voice-order');
      } else if (readyMessage.toLowerCase().trim() != 'ready') {
        print('[WelcomePage] ⚠️ "ready"가 아닌 메시지 수신: "$readyMessage" (무시)');
      }
    }, onError: (error) {
      print('[WelcomePage] ❌ 음성 주문 준비 스트림 에러: $error');
    });
    
    print('[WelcomePage] 음성 주문 준비 토픽 구독 완료 (대기 중...)');
    print('[WelcomePage] ========== 음성 주문 준비 토픽 구독 종료 ==========');
  }

  /// 대화로 주문하기 버튼 클릭 처리
  /// 버튼은 start_voice_order 토픽을 발행하는 용도 (화면 전환 없음)
  void _onVoiceOrderButtonClick() async {
    if (_hasClickedVoiceOrderButton) {
      print('[WelcomePage] ⚠️ 이미 버튼을 클릭했음');
      return;
    }

    print('[WelcomePage] ========== 대화로 주문하기 버튼 클릭 ==========');
    
    setState(() {
      _hasClickedVoiceOrderButton = true;
    });
    
    // start_voice_order 토픽 발행 (화면 전환 없음)
    await _statusService.publishVoiceOrderStart();
    
    print('[WelcomePage] ✅ start_voice_order 토픽 발행 완료');
    print('[WelcomePage] voice_order_ready 토픽에서 "ready" 메시지 대기 중...');
  }

  @override
  void dispose() {
    print('[WelcomePage] ========== dispose() 호출 ==========');
    _voiceOrderReadySubscription?.cancel();  // 음성 주문 준비 구독 취소
    print('[WelcomePage] 음성 주문 준비 구독 취소 완료');
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
                      onTap: _onVoiceOrderButtonClick,
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
