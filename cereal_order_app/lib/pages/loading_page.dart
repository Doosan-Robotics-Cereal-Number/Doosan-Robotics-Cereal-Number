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
  }

  /// 주문 정보를 로봇에 발행
  Future<void> _publishOrderToRobot() async {
    if (_orderPublished || widget.orderData == null) return;
    
    _orderPublished = true;

    // 1. 시리얼 종류 (이미 영어: start_sequence_a 또는 start_sequence_b)
    String cerealType = widget.orderData!.selectedCereal ?? 'start_sequence_a';
    
    // 2. 양 매핑 (한글 → 영어)
    String quantityKr = widget.orderData!.selectedQuantity ?? '보통';
    String quantity = 'medium';  // 기본값
    
    switch (quantityKr) {
      case '많이':
        quantity = 'large';
        break;
      case '보통':
        quantity = 'medium';
        break;
      case '적게':
        quantity = 'small';
        break;
    }
    
    // 3. 컵 타입 매핑 (한글 → 영어)
    String cupTypeKr = widget.orderData!.selectedCup ?? '매장컵';
    String cupType = 'store';  // 기본값
    
    if (cupTypeKr == '개인컵') {
      cupType = 'personal';
    } else if (cupTypeKr == '매장컵') {
      cupType = 'store';
    }

    // 4. CSV 형식으로 결합 (쉼표로 구분)
    String orderDataStr = '$cerealType,$quantity,$cupType';

    // 5. 로봇에 발행
    print('');
    print('═══════════════════════════════════════════════════');
    print('📤 [주문 토픽 전송] LoadingPage에서 발행 시작');
    print('═══════════════════════════════════════════════════');
    print('🎯 토픽명: ${AppConfig.orderTopicName}');
    print('📦 원본 데이터:');
    print('   - 시리얼: $cerealType ($quantityKr)');
    print('   - 양: $quantity ($quantityKr)');
    print('   - 컵: $cupType ($cupTypeKr)');
    print('📨 전송 데이터: "$orderDataStr"');
    print('═══════════════════════════════════════════════════');

    await widget.statusService.publishOrderInfo(orderData: orderDataStr);

    print('✅ [주문 토픽 전송] 발행 완료!');
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

