import 'package:flutter/material.dart';
import '../models/order_data.dart';
import '../services/ros2_status_service.dart';
import '../config/app_config.dart';

class WelcomePage extends StatefulWidget {
  const WelcomePage({super.key});

  @override
  State<WelcomePage> createState() => _WelcomePageState();
}

class _WelcomePageState extends State<WelcomePage> {
  ROS2StatusService? _ros2Service;
  bool _isSending = false;
  bool _isConnected = false;

  @override
  void initState() {
    super.initState();
    print('🔵 [WelcomePage] ROS2 서비스 초기화 시작...');
    
    // ROS2 서비스 초기화
    _ros2Service = ROS2StatusService(
      serverUrl: AppConfig.ros2ServerUrl,
      topicName: AppConfig.ros2TopicName,
      topicType: AppConfig.ros2TopicType,
    );
    
    // 연결 상태 모니터링
    _ros2Service!.connectionStream.listen((connected) {
      if (mounted) {
        setState(() {
          _isConnected = connected;
        });
        if (connected) {
          print('✅ [WelcomePage] ROS2 연결 성공!');
        } else {
          print('❌ [WelcomePage] ROS2 연결 끊김');
        }
      }
    });
    
    _ros2Service!.start();
    print('🔵 [WelcomePage] ROS2 서비스 start() 호출 완료');
  }

  @override
  void dispose() {
    _ros2Service?.dispose();
    super.dispose();
  }

  /// order_data 토픽 전송 테스트
  Future<void> _sendOrderDataTest() async {
    if (_isSending) return;

    print('');
    print('═══════════════════════════════════════════════════');
    print('📤 [WelcomePage 테스트] order_data 토픽 전송 시작');
    print('═══════════════════════════════════════════════════');
    print('🔗 연결 상태: ${_isConnected ? "연결됨 ✅" : "연결 안됨 ❌"}');
    print('🔗 서비스: ${_ros2Service != null ? "생성됨" : "null"}');
    
    if (!_isConnected) {
      print('❌ ROS2가 연결되지 않았습니다!');
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(
            content: Text('❌ ROS2 연결이 필요합니다. 잠시 후 다시 시도하세요.'),
            backgroundColor: Colors.red,
            duration: Duration(seconds: 3),
          ),
        );
      }
      return;
    }

    setState(() {
      _isSending = true;
    });

    try {
      // 테스트 주문 데이터
      final testOrderData = 'start_sequence_a,많이,매장컵';
      
      print('📋 전송 데이터: "$testOrderData"');
      print('🚀 publishOrderInfo() 호출 중...');
      
      await _ros2Service?.publishOrderInfo(orderData: testOrderData);
      
      print('✅ publishOrderInfo() 완료');
      print('═══════════════════════════════════════════════════');
      print('');
      
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(
            content: Text('✅ order_data 토픽 전송 완료!'),
            backgroundColor: Colors.green,
            duration: Duration(seconds: 2),
          ),
        );
      }
    } catch (e) {
      print('❌ [WelcomePage 테스트] order_data 전송 실패: $e');
      print('═══════════════════════════════════════════════════');
      print('');
      
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('❌ 전송 실패: $e'),
            backgroundColor: Colors.red,
            duration: const Duration(seconds: 2),
          ),
        );
      }
    } finally {
      if (mounted) {
        setState(() {
          _isSending = false;
        });
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      body: Stack(
        children: [
          Row(
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
                    // 시리얼 주문하기 버튼
                    SizedBox(
                      width: 600,
                      height: 100,
                      child: ElevatedButton(
                        onPressed: () {
                          // 원래 플로우: 시리얼 선택 → 양 선택 → 컵 선택 → 로딩
                          Navigator.pushNamed(
                            context, 
                            '/cereal-selection',
                            arguments: OrderData(), // 빈 OrderData 객체로 시작
                          );
                          
                          // [주석처리] 테스트용: 기본 주문 데이터로 바로 로딩 페이지로 이동
                          // final testOrderData = OrderData(
                          //   selectedCereal: 'start_sequence_a',  // A석
                          //   selectedQuantity: '많이',
                          //   selectedCup: '매장컵',
                          // );
                          // 
                          // Navigator.pushNamed(
                          //   context,
                          //   '/loading',
                          //   arguments: testOrderData,
                          // );
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
                          '시리얼 주문하기',
                          style: TextStyle(
                            fontSize: 32,
                            fontWeight: FontWeight.w500,
                          ),
                        ),
                      ),
                    ),
                    const SizedBox(height: 20),
                    // order_data 토픽 전송 테스트 버튼
                    SizedBox(
                      width: 600,
                      height: 80,
                      child: ElevatedButton(
                        onPressed: _isSending ? null : _sendOrderDataTest,
                        style: ElevatedButton.styleFrom(
                          backgroundColor: const Color(0xFF10B981),
                          foregroundColor: Colors.white,
                          disabledBackgroundColor: Colors.grey,
                          padding: const EdgeInsets.symmetric(
                            horizontal: 40,
                            vertical: 16,
                          ),
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(25),
                          ),
                          elevation: 2,
                        ),
                        child: _isSending
                            ? const Row(
                                mainAxisAlignment: MainAxisAlignment.center,
                                children: [
                                  SizedBox(
                                    width: 20,
                                    height: 20,
                                    child: CircularProgressIndicator(
                                      color: Colors.white,
                                      strokeWidth: 2,
                                    ),
                                  ),
                                  SizedBox(width: 12),
                                  Text(
                                    '전송 중...',
                                    style: TextStyle(
                                      fontSize: 24,
                                      fontWeight: FontWeight.w500,
                                    ),
                                  ),
                                ],
                              )
                            : const Text(
                                '📤 order_data 토픽 전송 테스트',
                                style: TextStyle(
                                  fontSize: 24,
                                  fontWeight: FontWeight.w500,
                                ),
                              ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
            // 우측 이미지
            Padding(
              padding: const EdgeInsets.only(right: 120.0),
              child: Image.asset(
                'assets/images/welcome-3d.png',
                width: 600,
                height: 600,
                fit: BoxFit.contain,
                errorBuilder: (context, error, stackTrace) {
                  return const SizedBox(width: 600, height: 600);
                },
              ),
            ),
          ],
        ),
        // 연결 상태 표시
        Positioned(
          top: 20,
          right: 20,
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 12),
            decoration: BoxDecoration(
              color: _isConnected 
                  ? Colors.green.withOpacity(0.9)
                  : Colors.red.withOpacity(0.9),
              borderRadius: BorderRadius.circular(25),
              boxShadow: [
                BoxShadow(
                  color: Colors.black.withOpacity(0.2),
                  blurRadius: 6,
                  offset: const Offset(0, 3),
                ),
              ],
            ),
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                Icon(
                  _isConnected ? Icons.check_circle : Icons.error,
                  color: Colors.white,
                  size: 24,
                ),
                const SizedBox(width: 10),
                Text(
                  _isConnected ? 'ROS2 연결됨' : 'ROS2 연결 안됨',
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
      ),
    );
  }
}


