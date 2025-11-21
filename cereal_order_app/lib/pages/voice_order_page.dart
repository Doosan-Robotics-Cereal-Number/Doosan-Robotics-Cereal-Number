import 'package:flutter/material.dart';
import 'dart:async';
import 'package:pulsator/pulsator.dart';
import 'package:lottie/lottie.dart';
import '../services/status_service.dart';
import '../services/status_service_factory.dart';
import '../config/app_config.dart';
import '../models/order_data.dart';

class VoiceOrderPage extends StatefulWidget {
  const VoiceOrderPage({super.key});

  @override
  State<VoiceOrderPage> createState() => _VoiceOrderPageState();
}

class _VoiceOrderPageState extends State<VoiceOrderPage> {
  late StatusService _statusService;
  StreamSubscription<Map<String, String>>? _orderInfoSubscription;  // 주문 정보 구독
  StreamSubscription<String>? _orderCancelSubscription;  // 주문 취소 구독
  OrderData? orderData;

  // 주문 정보 표시용 상태 변수
  String? _receivedMenu;
  String? _receivedSize;
  String? _receivedCup;
  bool _hasNavigatedToLoading = false; // 중복 이동 방지

  @override
  void initState() {
    super.initState();
    _initializeService();
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

    // 음성 주문 시작 신호 발행
    await _statusService.publishVoiceOrderStart();

    // 주문 완료 스트림 구독은 voice_order_loading_page에서 처리하므로 여기서는 제거

    // 주문 정보 스트림 구독
    _orderInfoSubscription = (_statusService as dynamic).orderInfoStream.listen((orderInfo) {
      if (mounted && !_hasNavigatedToLoading) {
        _receivedMenu = orderInfo['menu'];
        _receivedSize = orderInfo['size'];
        _receivedCup = orderInfo['cup'];
        
        print('[VoiceOrderPage] 주문 정보 수신: 메뉴=$_receivedMenu, 양=$_receivedSize, 컵=$_receivedCup');
        
        // OrderData로 변환
        OrderData voiceOrderData = _convertToOrderData(_receivedMenu!, _receivedSize!, _receivedCup!);
        
        // voice_order_loading_page로 이동
        _hasNavigatedToLoading = true;
        Navigator.pushNamed(
          context,
          '/voice-order-loading',
          arguments: {
            'orderData': voiceOrderData,
            'statusService': _statusService,
          },
        );
      }
    });

    // 주문 취소 스트림 구독
    _orderCancelSubscription = (_statusService as dynamic).orderCancelStream.listen((cancelReason) {
      print('[VoiceOrderPage] ========== 음성 취소 신호 수신 ==========');
      print('[VoiceOrderPage] 취소 이유: $cancelReason');
      print('[VoiceOrderPage] mounted 상태: $mounted');
      
      if (mounted) {
        try {
          print('[VoiceOrderPage] Navigator.pop() 실행 전...');
          print('[VoiceOrderPage] context: $context');
          Navigator.pop(context);
          print('[VoiceOrderPage] ✅ Navigator.pop() 실행 완료');
        } catch (e, stackTrace) {
          print('[VoiceOrderPage] ❌ 음성 취소 처리 중 에러 발생: $e');
          print('[VoiceOrderPage] 스택 트레이스: $stackTrace');
        }
      } else {
        print('[VoiceOrderPage] ⚠️ mounted가 false입니다. Navigator.pop() 실행하지 않음');
      }
      
      print('[VoiceOrderPage] ========== 음성 취소 신호 처리 완료 ==========');
    });
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    // 이전 화면에서 전달받은 주문 데이터 가져오기 (있는 경우)
    orderData = ModalRoute.of(context)?.settings.arguments as OrderData?;
  }

  /// 주문 정보를 OrderData 형식으로 변환
  OrderData _convertToOrderData(String menu, String size, String cup) {
    // 메뉴 변환: 코코볼/그래놀라 → start_sequence_a/start_sequence_b
    String? cerealType;
    if (menu == '코코볼') {
      cerealType = 'start_sequence_a';
    } else if (menu == '그래놀라') {
      cerealType = 'start_sequence_b';
    }

    // 양 변환: 많이/보통/적게 (그대로 사용)
    String? quantity = size;

    // 컵 변환: 개인컵/매장컵 (그대로 사용)
    String? cupType = cup;

    return OrderData(
      selectedCereal: cerealType,
      selectedQuantity: quantity,
      selectedCup: cupType,
    );
  }

  @override
  void dispose() {
    print('[VoiceOrderPage] ========== dispose() 호출 ==========');
    _orderInfoSubscription?.cancel();  // 주문 정보 구독 취소
    print('[VoiceOrderPage] 주문 정보 구독 취소 완료');
    _orderCancelSubscription?.cancel();  // 주문 취소 구독 취소
    print('[VoiceOrderPage] 주문 취소 구독 취소 완료');
    // statusService는 voice_order_loading_page에서 관리하므로 여기서는 정리하지 않음
    // _statusService.stop();
    // _statusService.dispose();
    super.dispose();
    print('[VoiceOrderPage] ✅ dispose() 완료');
  }

  /// 뒤로가기 버튼 클릭
  void _onBackPressed() async {
    print('[VoiceOrderPage] ========== 뒤로가기 버튼 클릭 시작 ==========');
    print('[VoiceOrderPage] mounted 상태: $mounted');
    print('[VoiceOrderPage] 현재 Navigator 스택 확인...');
    
    // Navigator 스택 정보 로깅
    try {
      Navigator.of(context, rootNavigator: false);
      print('[VoiceOrderPage] Navigator 상태: 존재함');
    } catch (e) {
      print('[VoiceOrderPage] Navigator 상태: 없음 ($e)');
    }
    
    try {
      // 음성 주문 취소 신호 발행
      print('[VoiceOrderPage] 음성 주문 취소 신호 발행 시작...');
      await _statusService.publishVoiceOrderCancel();
      print('[VoiceOrderPage] ✅ 음성 주문 취소 신호 발행 완료');
      
      // 음성 취소와 동일한 로직: 이전 화면으로 복귀
      if (mounted) {
        print('[VoiceOrderPage] Navigator.pop() 실행 전...');
        print('[VoiceOrderPage] context: $context');
        Navigator.pop(context);
        print('[VoiceOrderPage] ✅ Navigator.pop() 실행 완료');
      } else {
        print('[VoiceOrderPage] ⚠️ mounted가 false입니다. Navigator.pop() 실행하지 않음');
      }
    } catch (e, stackTrace) {
      print('[VoiceOrderPage] ❌ 뒤로가기 처리 중 에러 발생: $e');
      print('[VoiceOrderPage] 스택 트레이스: $stackTrace');
    }
    
    print('[VoiceOrderPage] ========== 뒤로가기 버튼 클릭 종료 ==========');
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 40.0, vertical: 30.0),
          child: Column(
            children: [
              // 뒤로가기 버튼
              Align(
                alignment: Alignment.topLeft,
                child: GestureDetector(
                  onTap: _onBackPressed,
                  child: Container(
                    padding: const EdgeInsets.symmetric(
                      horizontal: 24,
                      vertical: 12,
                    ),
                    decoration: BoxDecoration(
                      color: Colors.white,
                      border: Border.all(
                        color: const Color(0xFFDCDCDC),
                        width: 1,
                      ),
                      borderRadius: BorderRadius.circular(8),
                    ),
                    child: const Text(
                      '뒤로가기',
                      style: TextStyle(
                        fontSize: 16,
                        fontWeight: FontWeight.w400,
                        color: Color(0xFF666666),
                      ),
                    ),
                  ),
                ),
              ),
              const SizedBox(height: 60),
              // 메인 컨텐츠 영역
              const Spacer(),

              // Pulsator 애니메이션 (주문 정보가 없을 때만 표시)
              if (_receivedMenu == null || _receivedSize == null || _receivedCup == null)
                Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    // "듣고있어요" 텍스트
                    const Text(
                      '듣고있어요',
                      style: TextStyle(
                        fontSize: 48,
                        fontWeight: FontWeight.w600,
                        color: Color(0xFF121212),
                      ),
                    ),
                    const SizedBox(height: 60),
                    // 애니메이션 영역
                    Center(
                      child: SizedBox(
                        width: 600,
                        height: 600,
                        child: Stack(
                          alignment: Alignment.center,
                          children: [
                            // Pulsator 애니메이션
                            Pulsator(
                              count: 8,
                              duration: const Duration(seconds: 4),
                              repeat: 0,
                              style: PulseStyle(
                                color: const Color(0xFFE2DDFF),
                                borderColor: Colors.white,
                                borderWidth: 4.0,
                                gradientStyle: PulseGradientStyle(
                                  startColor: Colors.white,
                                  start: 0.5,
                                  reverseColors: true,
                                ),
                                pulseCurve: Curves.easeOutQuint,
                                opacityCurve: Curves.easeOut,
                              ),
                            ),
                            // AI 캐릭터 Lottie 애니메이션
                            Lottie.asset(
                              'assets/animations/voice-ai-animation-listen.json',
                              width: 360,
                              height: 360,
                              fit: BoxFit.contain,
                            ),
                          ],
                        ),
                      ),
                    ),
                  ],
                ),

              // 주문 정보 표시 부분 제거 (voice_order_loading_page로 이동)

              const Spacer(),
              
              const SizedBox(height: 20),
            ],
          ),
        ),
      ),
    );
  }

}

