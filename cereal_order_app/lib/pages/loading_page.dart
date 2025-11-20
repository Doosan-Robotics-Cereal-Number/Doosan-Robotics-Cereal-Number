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

class _LoadingPageState extends State<LoadingPage> {
  bool _orderPublished = false;

  @override
  void initState() {
    super.initState();
    
    // 주문 정보 발행
    _publishOrderToRobot();
    
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
    
    // ⭐ props를 우선 사용하고, 없을 때만 route arguments 확인
    OrderData? orderData = widget.orderData;
    
    // props에 없으면 route arguments에서 가져오기
    if (orderData == null) {
      final routeArgs = ModalRoute.of(context)?.settings.arguments;
      // route arguments가 Map인지 OrderData인지 안전하게 확인
      if (routeArgs is OrderData) {
        orderData = routeArgs;
        print('✅ [디버그] route arguments에서 OrderData 직접 추출');
      } else if (routeArgs is Map<String, dynamic>) {
        // Map인 경우 'orderData' 키에서 추출 (음성 주문 플로우)
        orderData = routeArgs['orderData'] as OrderData?;
        print('✅ [디버그] route arguments에서 Map의 orderData 키 추출');
      } else {
        print('⚠️ [디버그] route arguments 타입을 알 수 없음: ${routeArgs?.runtimeType}');
      }
    } else {
      print('✅ [디버그] widget.orderData 사용');
    }
    
    print('🔍 [디버그] orderData == null: ${orderData == null}');
    
    if (orderData == null) {
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

    // 1. 시리얼 종류 → ROS2 형식으로 변환
    // voice_order_bridge 형식: start_sequence_a (코코볼) 또는 start_sequence_b (조리퐁/그래놀라)
    String cerealType = orderData.selectedCereal ?? 'start_sequence_a';
    String menu = cerealType == 'start_sequence_b' ? 'start_sequence_b' : 'start_sequence_a';
    
    // 2. 양 → ROS2 형식으로 변환
    // voice_order_bridge 형식: small (적게), medium (보통), large (많이)
    String quantity = orderData.selectedQuantity ?? '보통';
    String size;
    if (quantity == '적게') {
      size = 'small';
    } else if (quantity == '많이') {
      size = 'large';
    } else {
      size = 'medium'; // '보통' 또는 기본값
    }
    
    // 3. 컵 타입 → ROS2 형식으로 변환
    // voice_order_bridge 형식: personal (개인컵), store (매장컵)
    String cupType = orderData.selectedCup ?? '매장컵';
    String cup = cupType == '개인컵' ? 'personal' : 'store';

    // 4. CSV 형식으로 생성 (voice_order_bridge와 동일한 형식)
    // 형식: "start_sequence_a,medium,store"
    String orderDataStr = '$menu,$size,$cup';

    // 5. 로봇에 발행
    print('');
    print('═══════════════════════════════════════════════════');
    print('📤 [주문 토픽 전송] LoadingPage에서 발행 시작');
    print('═══════════════════════════════════════════════════');
    print('🎯 토픽명: ${AppConfig.orderTopicName}');
    print('📦 원본 데이터:');
    print('   - 시리얼: $cerealType → ROS2: $menu');
    print('   - 양: $quantity → ROS2: $size');
    print('   - 컵: $cupType → ROS2: $cup');
    print('📨 CSV 전송 데이터: $orderDataStr');
    print('═══════════════════════════════════════════════════');

    await widget.statusService.publishOrderInfo(orderData: orderDataStr);

    print('');
    print('✅ [주문 토픽 전송 완료]');
    print('로봇이 주문을 받았습니다!');
    print('═══════════════════════════════════════════════════');
    print('');
  }

  @override
  void dispose() {
    super.dispose();
  }

  /// 주문 데이터 가져오기
  OrderData? _getOrderData() {
    // 강제로 로그 출력 (디버깅용)
    debugPrint('');
    debugPrint('═══════════════════════════════════════════════════');
    debugPrint('🔍 [LoadingPage] _getOrderData() 함수 호출됨!');
    debugPrint('🔍 [LoadingPage] 주문 데이터 확인 시작');
    debugPrint('═══════════════════════════════════════════════════');
    
    print('');
    print('═══════════════════════════════════════════════════');
    print('🔍 [LoadingPage] 주문 데이터 확인 시작');
    print('═══════════════════════════════════════════════════');
    
    // ⭐ props를 우선 사용하고, 없을 때만 route arguments 확인
    OrderData? widgetData = widget.orderData;
    OrderData? routeData;
    
    // route arguments에서 안전하게 가져오기
    final routeArgs = ModalRoute.of(context)?.settings.arguments;
    if (routeArgs is OrderData) {
      routeData = routeArgs;
    } else if (routeArgs is Map<String, dynamic>) {
      // Map인 경우 'orderData' 키에서 추출 (음성 주문 플로우)
      routeData = routeArgs['orderData'] as OrderData?;
    }
    
    // 최종 데이터: widget.orderData 우선, 없으면 route arguments
    final orderData = widgetData ?? routeData;
    
    print('📦 데이터 소스 확인:');
    print('   - routeData (arguments): ${routeData != null ? "✅ 있음" : "❌ 없음"}');
    if (routeData != null) {
      print('      └─ 시리얼: ${routeData.selectedCereal ?? "null"}');
      print('      └─ 양: ${routeData.selectedQuantity ?? "null"}');
      print('      └─ 컵: ${routeData.selectedCup ?? "null"}');
    }
    print('   - widget.orderData: ${widgetData != null ? "✅ 있음" : "❌ 없음"}');
    if (widgetData != null) {
      print('      └─ 시리얼: ${widgetData.selectedCereal ?? "null"}');
      print('      └─ 양: ${widgetData.selectedQuantity ?? "null"}');
      print('      └─ 컵: ${widgetData.selectedCup ?? "null"}');
    }
    print('   - 최종 사용할 데이터: ${orderData != null ? "✅ 있음" : "❌ 없음"}');
    
    if (orderData != null) {
      print('');
      print('📋 [LoadingPage] 최종 주문 정보:');
      print('   ┌─────────────────────────────────────────┐');
      print('   │ 시리얼 종류: ${orderData.selectedCereal ?? "null"} ${orderData.selectedCereal == null ? "⚠️" : "✅"}');
      if (orderData.selectedCereal != null) {
        final cerealName = orderData.selectedCereal == 'start_sequence_b' ? '조리퐁' : '코코볼';
        print('   │   → 표시명: $cerealName');
      }
      print('   │ 양: ${orderData.selectedQuantity ?? "null"} ${orderData.selectedQuantity == null ? "⚠️" : "✅"}');
      if (orderData.selectedQuantity != null) {
        final quantityDisplay = orderData.selectedQuantity == '보통' ? '적당히' : orderData.selectedQuantity;
        print('   │   → 표시명: $quantityDisplay');
      }
      print('   │ 컵: ${orderData.selectedCup ?? "null"} ${orderData.selectedCup == null ? "⚠️" : "✅"}');
      print('   └─────────────────────────────────────────┘');
      
      // 각 필드별 상세 확인
      print('');
      print('🔎 필드별 상세 확인:');
      print('   1. selectedCereal:');
      print('      - 값: ${orderData.selectedCereal ?? "null"}');
      if (orderData.selectedCereal != null) {
        print('      - 타입: ${orderData.selectedCereal.runtimeType}');
        print('      - null 여부: ✅ 값 있음');
      } else {
        print('      - 타입: null');
        print('      - null 여부: ❌ null');
      }
      
      print('   2. selectedQuantity:');
      print('      - 값: ${orderData.selectedQuantity ?? "null"}');
      if (orderData.selectedQuantity != null) {
        print('      - 타입: ${orderData.selectedQuantity.runtimeType}');
        print('      - null 여부: ✅ 값 있음');
      } else {
        print('      - 타입: null');
        print('      - null 여부: ❌ null');
      }
      
      print('   3. selectedCup:');
      print('      - 값: ${orderData.selectedCup ?? "null"}');
      if (orderData.selectedCup != null) {
        print('      - 타입: ${orderData.selectedCup.runtimeType}');
        print('      - null 여부: ✅ 값 있음');
      } else {
        print('      - 타입: null');
        print('      - null 여부: ❌ null');
      }
      
      // 검증 결과
      print('');
      print('✅ 검증 결과:');
      final allFieldsSet = orderData.selectedCereal != null && 
                          orderData.selectedQuantity != null && 
                          orderData.selectedCup != null;
      if (allFieldsSet) {
        print('   ✅ 모든 필드가 정상적으로 설정되었습니다!');
      } else {
        print('   ⚠️ 일부 필드가 null입니다:');
        if (orderData.selectedCereal == null) print('      - selectedCereal: null');
        if (orderData.selectedQuantity == null) print('      - selectedQuantity: null');
        if (orderData.selectedCup == null) print('      - selectedCup: null');
      }
    } else {
      print('');
      print('❌ [LoadingPage] 주문 정보가 없습니다!');
      print('   - routeData: ${routeData != null ? "있음" : "없음"}');
      print('   - widget.orderData: ${widgetData != null ? "있음" : "없음"}');
    }
    
    print('═══════════════════════════════════════════════════');
    print('');
    
    return orderData;
  }

  /// 시리얼 종류 이미지 경로 가져오기 (코코볼/조리퐁)
  String _getCerealTypeImage(OrderData orderData) {
    final cereal = orderData.selectedCereal ?? 'start_sequence_a';
    print('🔍 [LoadingPage] 시리얼 종류 이미지 선택:');
    print('   - selectedCereal: ${orderData.selectedCereal}');
    print('   - 사용할 값: $cereal');
    
    final imagePath = cereal == 'start_sequence_b' 
        ? 'assets/images/menu-2.png'  // 조리퐁
        : 'assets/images/menu-1.png'; // 코코볼
    
    print('   - 이미지 경로: $imagePath');
    return imagePath;
  }

  /// 시리얼 양 이미지 경로 가져오기 (많이/보통/적게)
  String _getCerealImage(OrderData orderData) {
    final quantity = orderData.selectedQuantity ?? '보통';
    if (quantity == '많이') {
      return 'assets/images/many-cereal.png';
    } else if (quantity == '적게') {
      return 'assets/images/small-cereal.png';
    }
    return 'assets/images/normal-cereal.png';
  }

  /// 시리얼 이름 가져오기
  String _getCerealName(OrderData orderData) {
    final cereal = orderData.selectedCereal ?? 'start_sequence_a';
    return cereal == 'start_sequence_b' ? '조리퐁' : '코코볼';
  }

  /// 양 텍스트 표시 (보통 → 적당히 변환)
  String _getQuantityText(OrderData orderData) {
    final quantity = orderData.selectedQuantity ?? '보통';
    return quantity == '보통' ? '적당히' : quantity;
  }

  /// 컵 이미지 경로 가져오기
  String _getCupImage(OrderData orderData) {
    final cup = orderData.selectedCup ?? '매장컵';
    return cup == '매장컵' ? 'assets/images/cup-1.png' : 'assets/images/cup-2.png';
  }

  @override
  Widget build(BuildContext context) {
    debugPrint('');
    debugPrint('🎨 [LoadingPage] build 메서드 호출됨!');
    print('');
    print('🎨 [LoadingPage] build 메서드 호출됨');
    print('═══════════════════════════════════════════════════');
    
    final orderData = _getOrderData();
    
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
          SafeArea(
            child: Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  // 메인 메시지
                  const Text(
                    '든든한 하루를 챙겨드릴게요.',
                    style: TextStyle(
                      fontSize: 80,
                      fontWeight: FontWeight.bold,
                      color: Color(0xFF121212),
                      height: 1.2,
                    ),
                    textAlign: TextAlign.center,
                  ),
                  const SizedBox(height: 40),
                  const Text(
                    '주문한 시리얼을 만들고 있어요',
                    style: TextStyle(
                      fontSize: 32,
                      color: Color(0xFF666666),
                    ),
                    textAlign: TextAlign.center,
                  ),
                  const SizedBox(height: 80),
                  // 주문 내역 표시
                  if (orderData != null) ...[
                    Row(
                      mainAxisAlignment: MainAxisAlignment.center,
                      crossAxisAlignment: CrossAxisAlignment.center,
                      children: [
                        // 시리얼 종류 이미지 (코코볼/조리퐁)
                        Column(
                          children: [
                            Image.asset(
                              _getCerealTypeImage(orderData),
                              width: 280,
                              height: 280,
                              fit: BoxFit.contain,
                              errorBuilder: (context, error, stackTrace) {
                                print('❌ [LoadingPage] 시리얼 종류 이미지 로드 실패: $error');
                                print('   - 시도한 경로: ${_getCerealTypeImage(orderData)}');
                                return Container(
                                  width: 280,
                                  height: 280,
                                  color: Colors.grey[300],
                                  child: Column(
                                    mainAxisAlignment: MainAxisAlignment.center,
                                    children: [
                                      const Icon(Icons.image_not_supported, size: 50),
                                      const SizedBox(height: 8),
                                      Text(
                                        '이미지 없음\n${_getCerealName(orderData)}',
                                        textAlign: TextAlign.center,
                                        style: const TextStyle(fontSize: 16),
                                      ),
                                    ],
                                  ),
                                );
                              },
                            ),
                            const SizedBox(height: 16),
                            Text(
                              _getCerealName(orderData),
                              style: const TextStyle(
                                fontSize: 34,
                                fontWeight: FontWeight.w700,
                                color: Color(0xFF121212),
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(width: 100),
                        // 양 표시
                        Column(
                          children: [
                            Image.asset(
                              _getCerealImage(orderData),
                              width: 280,
                              height: 280,
                              fit: BoxFit.contain,
                              errorBuilder: (context, error, stackTrace) {
                                return Container(
                                  width: 280,
                                  height: 280,
                                  color: Colors.grey[300],
                                  child: const Icon(Icons.image_not_supported, size: 50),
                                );
                              },
                            ),
                            const SizedBox(height: 16),
                            Text(
                              _getQuantityText(orderData),
                              style: const TextStyle(
                                fontSize: 34,
                                fontWeight: FontWeight.w700,
                                color: Color(0xFF121212),
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(width: 100),
                        // 매장 컵 이미지
                        Column(
                          children: [
                            Image.asset(
                              _getCupImage(orderData),
                              width: 280,
                              height: 280,
                              fit: BoxFit.contain,
                              errorBuilder: (context, error, stackTrace) {
                                return Container(
                                  width: 280,
                                  height: 280,
                                  color: Colors.grey[300],
                                  child: const Icon(Icons.image_not_supported, size: 50),
                                );
                              },
                            ),
                            const SizedBox(height: 16),
                            Text(
                              orderData.selectedCup ?? '매장컵',
                              style: const TextStyle(
                                fontSize: 34,
                                fontWeight: FontWeight.w700,
                                color: Color(0xFF121212),
                              ),
                            ),
                          ],
                        ),
                      ],
                    ),
                  ] else ...[
                    // orderData가 없을 때 표시
                    Container(
                      padding: const EdgeInsets.all(40),
                      decoration: BoxDecoration(
                        color: Colors.white.withOpacity(0.8),
                        borderRadius: BorderRadius.circular(20),
                      ),
                      child: const Text(
                        '주문 정보를 불러오는 중입니다...',
                        style: TextStyle(
                          fontSize: 32,
                          color: Color(0xFF666666),
                        ),
                      ),
                    ),
                  ],
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

