import 'package:flutter/material.dart';
import '../models/order_data.dart';
import '../services/status_service.dart';
import '../config/app_config.dart';

class LoadingPage extends StatefulWidget {
  final OrderData? orderData;
  final StatusService statusService;
  
  const LoadingPage({
    super.key,
    this.orderData,
    required this.statusService,
  });

  @override
  State<LoadingPage> createState() => _LoadingPageState();
}

class _LoadingPageState extends State<LoadingPage> with TickerProviderStateMixin {
  late AnimationController _progressController;
  late Animation<double> _progressAnimation;
  int remainingSeconds = 3;
  bool _orderPublished = false;

  @override
  void initState() {
    super.initState();
    
    // 프로그레스바 애니메이션 설정
    _progressController = AnimationController(
      duration: const Duration(seconds: 3),
      vsync: this,
    );
    
    _progressAnimation = Tween<double>(
      begin: 0.0,
      end: 1.0,
    ).animate(CurvedAnimation(
      parent: _progressController,
      curve: Curves.easeInOut,
    ));

    // 주문 정보 발행
    _publishOrderToRobot();

    // 3초 타이머 시작
    _startCountdown();
    
    // 프로그레스바 애니메이션 시작
    _progressController.forward();
    
    // 위젯이 완전히 빌드된 후에 주문 정보 발행
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _publishOrderToRobot();
    });
  }

  /// 주문 정보를 로봇에 발행
  Future<void> _publishOrderToRobot() async {
    if (_orderPublished) {
      print('⚠️ [디버그] 이미 주문 정보를 발행했습니다');
      return;
    }
    
    print('🔍 [디버그] _publishOrderToRobot 호출됨');
    print('🔍 [디버그] _orderPublished: $_orderPublished');
    print('🔍 [디버그] ROS2 연결 상태: ${widget.statusService.isConnected}');
    
    // ⭐ 핵심: ROS2 연결이 완료될 때까지 기다리기
    if (!widget.statusService.isConnected) {
      print('⏳ [디버그] ROS2 연결 대기 중... (최대 5초)');
      
      int waitCount = 0;
      while (!widget.statusService.isConnected && waitCount < 50) {
        await Future.delayed(const Duration(milliseconds: 100));
        waitCount++;
        
        if (waitCount % 10 == 0) {
          print('⏳ 대기 중... ${waitCount * 100}ms');
        }
      }
      
      if (!widget.statusService.isConnected) {
        print('❌ [디버그] ROS2 연결 타임아웃 (5초) - 토픽 발행 불가');
        print('💡 [힌트] Network Manager에서 rosbridge_server가 실행 중인지 확인하세요');
        return;
      }
      
      print('✅ [디버그] ROS2 연결 완료! 토픽 발행 시작');
    }
    
    // route arguments에서 직접 가져오기
    final OrderData? orderData = ModalRoute.of(context)?.settings.arguments as OrderData?;
    
    print('🔍 [디버그] orderData == null: ${orderData == null}');
    
    if (orderData == null) {
      print('⚠️ [디버그] orderData가 null입니다. widget.orderData 확인...');
      print('🔍 [디버그] widget.orderData == null: ${widget.orderData == null}');
      
      // widget.orderData도 확인
      if (widget.orderData != null) {
        print('✅ [디버그] widget.orderData를 사용합니다');
        _publishOrderWithData(widget.orderData!);
        return;
      }
      
      print('❌ [디버그] 주문 정보를 찾을 수 없습니다');
      return;
    }
    
    _publishOrderWithData(orderData);
  }

  /// 실제 주문 데이터 발행
  Future<void> _publishOrderWithData(OrderData orderData) async {
    if (_orderPublished) return;
    _orderPublished = true;

    print('✅ [디버그] orderData 확보 완료!');

    // 1. 시리얼 종류 → seat 매핑
    String cerealType = orderData.selectedCereal ?? 'start_sequence_a';
    String seat = cerealType == 'start_sequence_b' ? 'B' : 'A';
    
    // 2. 양 (한글 그대로 사용)
    String portion = orderData.selectedQuantity ?? '보통';
    
    // 3. 컵 타입 (한글 그대로 사용)
    String cupType = orderData.selectedCup ?? '매장컵';

    // 4. JSON 형식으로 생성
    String orderDataStr = '{"seat":"$seat", "portion":"$portion", "cup_type":"$cupType"}';

    // 5. 로봇에 발행
    print('');
    print('═══════════════════════════════════════════════════');
    print('📤 [주문 토픽 전송] LoadingPage에서 발행 시작');
    print('═══════════════════════════════════════════════════');
    print('🎯 토픽명: ${AppConfig.orderTopicName}');
    print('📦 원본 데이터:');
    print('   - 시리얼: $cerealType → seat: $seat');
    print('   - 양: $portion');
    print('   - 컵: $cupType');
    print('📨 JSON 전송 데이터: $orderDataStr');
    print('═══════════════════════════════════════════════════');

    await widget.statusService.publishOrderInfo(orderData: orderDataStr);

    print('');
    print('✅✅✅ [주문 토픽 전송 완료] ✅✅✅');
    print('로봇이 주문을 받았습니다!');
    print('═══════════════════════════════════════════════════');
    print('');
  }

  void _startCountdown() {
    Future.doWhile(() async {
      await Future.delayed(const Duration(seconds: 1));
      if (mounted) {
        setState(() {
          remainingSeconds--;
        });
        return remainingSeconds > 0;
      }
      return false;
    });
    // 타이머는 동작하지만 자동으로 페이지 이동하지 않음
    // ROS2의 order_done 토픽을 받아야 페이지 이동
  }

  @override
  void dispose() {
    _progressController.dispose();
    super.dispose();
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
              const SizedBox(height: 60),
              // 메인 메시지
              const Text(
                '든든한 하루를 챙겨드릴게요.',
                style: TextStyle(
                  fontSize: 32,
                  fontWeight: FontWeight.bold,
                  color: Color(0xFF333333),
                ),
                textAlign: TextAlign.center,
              ),
              const SizedBox(height: 80),
              // 컨텐츠 영역
              Expanded(
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    // 프로그레스바
                    Center(
                      child: AnimatedBuilder(
                        animation: _progressAnimation,
                        builder: (context, child) {
                          return Container(
                            width: 1436,
                            height: 120,
                            decoration: BoxDecoration(
                              color: Colors.grey.shade300,
                              borderRadius: BorderRadius.circular(20),
                            ),
                            child: FractionallySizedBox(
                              alignment: Alignment.centerLeft,
                              widthFactor: _progressAnimation.value,
                              child: Container(
                                decoration: BoxDecoration(
                                  color: const Color(0xFF0064FF),
                                  borderRadius: BorderRadius.circular(20),
                                ),
                              ),
                            ),
                          );
                        },
                      ),
                    ),
                    const SizedBox(height: 20),
                    // 예상 대기 시간
                    const Text(
                      '예상 대기 시간',
                      style: TextStyle(
                        fontSize: 24,
                        color: Colors.black,
                      ),
                      textAlign: TextAlign.center,
                    ),
                    const SizedBox(height: 8),
                    Text(
                      '${remainingSeconds}초',
                      style: const TextStyle(
                        fontSize: 64,
                        fontWeight: FontWeight.bold,
                        color: Colors.grey,
                      ),
                      textAlign: TextAlign.center,
                    ),
                  ],
                ),
              ),
              const SizedBox(height: 30),
            ],
          ),
        ),
      ),
    );
  }
}

