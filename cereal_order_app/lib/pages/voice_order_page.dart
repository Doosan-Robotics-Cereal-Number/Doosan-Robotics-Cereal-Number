import 'package:flutter/material.dart';
import 'dart:async';
import 'package:pulsator/pulsator.dart';
import 'package:lottie/lottie.dart';
import '../services/status_service.dart';
import '../services/status_service_factory.dart';
import '../services/ros2_status_service.dart';
import '../config/app_config.dart';
import '../models/order_data.dart';
import '../widgets/voice_order_test_button.dart';

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

  /// 테스트용 랜덤 주문 처리
  /// ROS2 토픽에 직접 발행하여 실제 음성 주문과 동일한 흐름으로 처리
  /// 발행 실패 시 /loading으로 직접 이동
  Future<void> _onTestOrderGenerated(String orderCsv) async {
    if (_hasNavigatedToLoading) {
      print('[VoiceOrderPage] 이미 로딩 페이지로 이동했습니다.');
      return;
    }

    print('[VoiceOrderPage] 테스트 주문 CSV: $orderCsv');
    
    // CSV 파싱하여 OrderData 생성
    List<String> parts = orderCsv.split(',');
    if (parts.length != 3) {
      print('[VoiceOrderPage] ❌ 잘못된 CSV 형식: $orderCsv');
      return;
    }
    
    // 한글 변환 (orderInfoStream과 동일한 로직)
    String menu = parts[0].contains('sequence_a') ? '코코볼' : '그래놀라';
    String size = parts[1] == 'large' ? '많이' :
                  parts[1] == 'small' ? '적게' : '보통';
    String cup = parts[2] == 'personal' ? '개인컵' : '매장컵';
    
    // OrderData로 변환
    OrderData voiceOrderData = _convertToOrderData(menu, size, cup);
    
    print('[VoiceOrderPage] 주문 내역: 메뉴=$menu, 양=$size, 컵=$cup');
    
    // ROS2 연결 상태 확인
    if (!_statusService.isConnected) {
      print('[VoiceOrderPage] ⚠️ ROS2 연결되지 않음. /loading으로 직접 이동');
      _hasNavigatedToLoading = true;
      Navigator.pushNamed(
        context,
        '/loading',
        arguments: voiceOrderData,
      );
      return;
    }
    
    print('[VoiceOrderPage] ROS2 토픽에 주문 정보 발행 시도...');
    
    // ROS2 토픽에 주문 정보 발행 시도
    bool publishSuccess = false;
    if (_statusService is ROS2StatusService) {
      // ROS2StatusService인 경우 publishString을 직접 사용하여 성공 여부 확인
      final ros2Service = _statusService as ROS2StatusService;
      const orderTopic = '/dsr01/kiosk/order';
      
      try {
        // 토픽 광고 (advertise) - private 메서드이므로 publishOrderInfo를 먼저 호출
        await _statusService.publishOrderInfo(orderData: orderCsv);
        
        // 짧은 딜레이 후 publishString으로 직접 발행하여 성공 여부 확인
        await Future.delayed(const Duration(milliseconds: 100));
        publishSuccess = await ros2Service.publishString(orderTopic, orderCsv);
      } catch (e) {
        print('[VoiceOrderPage] ❌ 토픽 발행 중 에러: $e');
        publishSuccess = false;
      }
    } else {
      // ROS2StatusService가 아닌 경우 (ManualStatusService 등)
      try {
        await _statusService.publishOrderInfo(orderData: orderCsv);
        // 연결 상태로 성공 여부 판단
        publishSuccess = _statusService.isConnected;
      } catch (e) {
        print('[VoiceOrderPage] ❌ 토픽 발행 중 에러: $e');
        publishSuccess = false;
      }
    }
    
    if (publishSuccess) {
      print('[VoiceOrderPage] ✅ 주문 정보 발행 성공. orderInfoStream 리스너가 처리합니다.');
      // 발행 성공 시 orderInfoStream 리스너가 자동으로 처리하므로 여기서는 아무것도 하지 않음
      // orderInfoStream 리스너가 주문 정보를 받아서 voice_order_loading_page로 이동함
    } else {
      print('[VoiceOrderPage] ❌ 주문 정보 발행 실패. /loading으로 직접 이동');
      // 발행 실패 시 /loading으로 직접 이동
      _hasNavigatedToLoading = true;
      Navigator.pushNamed(
        context,
        '/loading',
        arguments: voiceOrderData,
      );
    }
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

              // 주문 정보 표시 부분 제거 (voice_order_loading_page로 이동)

              const Spacer(),
              
              // 테스트 버튼 (디버그 모드일 때만 표시)
              if (AppConfig.debugMode)
                VoiceOrderTestButton(
                  onTestOrderGenerated: _onTestOrderGenerated,
                ),
              
              const SizedBox(height: 20),
            ],
          ),
        ),
      ),
    );
  }

}

