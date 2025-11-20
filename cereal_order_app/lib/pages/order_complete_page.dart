import 'package:flutter/material.dart';
import '../models/order_data.dart';

class OrderCompletePage extends StatefulWidget {
  const OrderCompletePage({super.key});

  @override
  State<OrderCompletePage> createState() => _OrderCompletePageState();
}

class _OrderCompletePageState extends State<OrderCompletePage> {
  OrderData? orderData;
  int remainingSeconds = 5;

  @override
  void initState() {
    super.initState();
    
    // 3초 카운트다운 시작
    _startCountdown();
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
    }).then((_) {
      if (mounted) {
        // 3초 후 자동으로 첫 화면으로 돌아가기
        Navigator.pushNamedAndRemoveUntil(
          context, 
          '/', 
          (route) => false,
        );
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    // 이전 화면에서 전달받은 주문 데이터 가져오기
    orderData = ModalRoute.of(context)?.settings.arguments as OrderData?;
    
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
                  // 메인 완료 메시지
                  const Text(
                    '시리얼이 준비되었어요!',
                    style: TextStyle(
                      fontSize: 80,
                      fontWeight: FontWeight.bold,
                      color: Color(0xFF121212),
                      height: 1.2,
                    ),
                    textAlign: TextAlign.center,
                  ),
                  const SizedBox(height: 40),
                  // 보조 메시지
                  const Text(
                    '픽업대에서 받아주세요.',
                    style: TextStyle(
                      fontSize: 32,
                      color: Color(0xFF666666),
                    ),
                    textAlign: TextAlign.center,
                  ),
                  const SizedBox(height: 80),
                  // 주문 완료 이미지
                  Image.asset(
                    'assets/images/order-complete.png',
                    width: 468,
                    height: 468,
                    fit: BoxFit.contain,
                    errorBuilder: (context, error, stackTrace) {
                      return Container(
                        width: 468,
                        height: 468,
                        color: Colors.grey[300],
                        child: const Icon(Icons.image_not_supported, size: 50),
                      );
                    },
                  ),
                  const SizedBox(height: 80),
                  // 처음으로 버튼
                  SizedBox(
                    width: 800,
                    height: 120,
                    child: ElevatedButton(
                      onPressed: () {
                        print('[OrderCompletePage] 처음으로 버튼 클릭됨');
                        // 첫 화면으로 돌아가기
                        Navigator.pushNamedAndRemoveUntil(
                          context, 
                          '/', 
                          (route) => false,
                        );
                      },
                      style: ElevatedButton.styleFrom(
                        backgroundColor: const Color(0xFF0064FF),
                        foregroundColor: Colors.white,
                        padding: const EdgeInsets.symmetric(vertical: 20),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(12),
                        ),
                        elevation: 0,
                      ),
                      child: Text(
                        '$remainingSeconds초 후에 주문 시작 화면으로 넘어가요.',
                        style: const TextStyle(
                          fontSize: 22,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
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


