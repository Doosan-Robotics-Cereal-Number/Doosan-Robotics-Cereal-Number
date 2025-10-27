import 'package:flutter/material.dart';
import '../models/order_data.dart';

class OrderCompletePage extends StatefulWidget {
  const OrderCompletePage({super.key});

  @override
  State<OrderCompletePage> createState() => _OrderCompletePageState();
}

class _OrderCompletePageState extends State<OrderCompletePage> {
  OrderData? orderData;

  @override
  void initState() {
    super.initState();
    
    // 5초 후 자동으로 첫 화면으로 돌아가기
    Future.delayed(const Duration(seconds: 5), () {
      if (mounted) {
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
      body: Container(
        decoration: const BoxDecoration(
          color: Color(0xFFF2F4F6),
        ),
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
                    fontSize: 28,
                    fontWeight: FontWeight.bold,
                    color: Color(0xFF4D4D4D),
                  ),
                  textAlign: TextAlign.center,
                ),
                const SizedBox(height: 16),
                // 보조 메시지
                const Text(
                  '픽업대에서 받아주세요.',
                  style: TextStyle(
                    fontSize: 18,
                    color: Colors.black,
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
                  child: GestureDetector(
                    onTap: () {
                      // 첫 화면으로 돌아가기
                      Navigator.pushNamedAndRemoveUntil(
                        context, 
                        '/', 
                        (route) => false,
                      );
                    },
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
                        '처음으로',
                        style: TextStyle(
                          fontSize: 16,
                          fontWeight: FontWeight.w400,
                          color: Color(0xFF666666),
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


