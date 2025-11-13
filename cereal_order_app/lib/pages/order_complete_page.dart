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
      backgroundColor: Colors.white,
      body: Container(
        child: SafeArea(
          child: Padding(
            padding: const EdgeInsets.all(20.0),
            child: Column(
              children: [
                const Spacer(),
                // 메인 완료 메시지
                const Text(
                  '시리얼이 준비되었어요!',
                  style: TextStyle(
                    fontSize: 32,
                    fontWeight: FontWeight.bold,
                    color: Color(0xFF333333),
                  ),
                  textAlign: TextAlign.center,
                ),
                const SizedBox(height: 20),
                // 보조 메시지
                const Text(
                  '픽업대에서 받아주세요.',
                  style: TextStyle(
                    fontSize: 24,
                    color: Color(0xFF666666),
                  ),
                  textAlign: TextAlign.center,
                ),
                const Spacer(),
                // 주문 정보 표시 (선택사항)
                // if (orderData != null)
                //   Container(
                //     padding: const EdgeInsets.all(16),
                //     margin: const EdgeInsets.symmetric(horizontal: 20),
                //     decoration: BoxDecoration(
                //       color: Colors.white.withOpacity(0.9),
                //       borderRadius: BorderRadius.circular(12),
                //       boxShadow: [
                //         BoxShadow(
                //           color: Colors.black.withOpacity(0.1),
                //           spreadRadius: 1,
                //           blurRadius: 5,
                //           offset: const Offset(0, 2),
                //         ),
                //       ],
                //     ),
                //     child: Column(
                //       children: [
                //         Text(
                //           '주문 정보',
                //           style: TextStyle(
                //             fontSize: 16,
                //             fontWeight: FontWeight.bold,
                //             color: Colors.grey.shade700,
                //           ),
                //         ),
                //         const SizedBox(height: 8),
                //         Text(
                //           '${orderData.selectedCereal == 'cocoball' ? '코코볼' : '그래놀라'} ${orderData.selectedQuantity} 양',
                //           style: const TextStyle(
                //             fontSize: 14,
                //             color: Colors.black,
                //           ),
                //         ),
                //         Text(
                //           '${orderData.selectedCup == 'store' ? '매장 컵' : '개인 컵'}',
                //           style: const TextStyle(
                //             fontSize: 14,
                //             color: Colors.black,
                //           ),
                //         ),
                //       ],
                //     ),
                //   ),
                // 처음으로 버튼
                Center(
                  child: SizedBox(
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
                ),
                const SizedBox(height: 40),
              ],
            ),
          ),
        ),
      ),
    );
  }
}


