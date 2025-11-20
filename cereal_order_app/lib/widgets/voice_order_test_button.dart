import 'package:flutter/material.dart';
import 'dart:math';

/// 음성 주문 테스트용 버튼 위젯
/// 
/// 랜덤 주문 내역을 ROS2 CSV 형식으로 생성하여 콜백으로 전달합니다.
/// AppConfig.debugMode가 true일 때만 표시됩니다.
class VoiceOrderTestButton extends StatelessWidget {
  final Function(String) onTestOrderGenerated; // CSV 문자열 전달

  const VoiceOrderTestButton({
    super.key,
    required this.onTestOrderGenerated,
  });

  /// 랜덤 주문 데이터를 ROS2 CSV 형식으로 생성
  /// 형식: "start_sequence_a,medium,store"
  static String generateRandomOrderCsv() {
    final random = Random();
    
    // 메뉴: start_sequence_a (코코볼) 또는 start_sequence_b (그래놀라)
    final menus = ['start_sequence_a', 'start_sequence_b'];
    final selectedMenu = menus[random.nextInt(menus.length)];
    
    // 양: small (적게), medium (보통), large (많이)
    final sizes = ['small', 'medium', 'large'];
    final selectedSize = sizes[random.nextInt(sizes.length)];
    
    // 컵: personal (개인컵) 또는 store (매장컵)
    final cups = ['personal', 'store'];
    final selectedCup = cups[random.nextInt(cups.length)];
    
    // CSV 형식으로 반환: "start_sequence_a,medium,store"
    return '$selectedMenu,$selectedSize,$selectedCup';
  }

  void _onTestButtonPressed() {
    final randomOrderCsv = generateRandomOrderCsv();
    print('[VoiceOrderTestButton] 랜덤 주문 CSV 생성: $randomOrderCsv');
    onTestOrderGenerated(randomOrderCsv);
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 16),
      decoration: BoxDecoration(
        color: const Color(0xFFFF6B6B),
        borderRadius: BorderRadius.circular(12),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 8,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: Material(
        color: Colors.transparent,
        child: InkWell(
          onTap: _onTestButtonPressed,
          borderRadius: BorderRadius.circular(12),
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                const Icon(
                  Icons.bug_report,
                  color: Colors.white,
                  size: 24,
                ),
                const SizedBox(width: 12),
                const Text(
                  '테스트: 랜덤 주문 생성',
                  style: TextStyle(
                    fontSize: 18,
                    fontWeight: FontWeight.w600,
                    color: Colors.white,
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

