import 'package:flutter/material.dart';
import '../models/order_data.dart';
import '../widgets/selection_page_layout.dart';

class CerealSelectionPage extends StatefulWidget {
  const CerealSelectionPage({super.key});

  @override
  State<CerealSelectionPage> createState() => _CerealSelectionPageState();
}

class _CerealSelectionPageState extends State<CerealSelectionPage> {
  String? selectedCereal = 'cocoball'; // 기본값으로 코코볼 선택
  final OrderData orderData = OrderData();

  @override
  void initState() {
    super.initState();
    orderData.selectedCereal = 'cocoball';
  }

  @override
  Widget build(BuildContext context) {
    return SelectionPageLayout(
      title: '어떤 시리얼로 준비할까요?',
      backButtonText: '처음으로',
      confirmButtonText: '메뉴 선택하기',
      isConfirmEnabled: selectedCereal != null,
      onConfirmPressed: () {
        Navigator.pushNamed(
          context,
          '/cup-selection',
          arguments: orderData,
        );
      },
      contentArea: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          // 코코볼 옵션
          Flexible(
            child: GestureDetector(
              onTap: () {
                setState(() {
                  selectedCereal = 'cocoball';
                  orderData.selectedCereal = 'cocoball';
                });
              },
              child: Container(
                width: 380,
                height: 500,
                decoration: BoxDecoration(
                  color: Colors.white,
                  borderRadius: BorderRadius.circular(20),
                  border: selectedCereal == 'cocoball'
                      ? Border.all(color: const Color(0xFF0095FF), width: 4)
                      : Border.all(color: Colors.transparent, width: 4),
                ),
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    const SizedBox(height: 40),
                    // 시리얼 이미지 - 코코볼
                    Container(
                      width: 200,
                      height: 200,
                      child: ClipRRect(
                        borderRadius: BorderRadius.circular(16),
                        child: Image.asset(
                          'assets/images/menu-1.png',
                          width: 200,
                          height: 200,
                          fit: BoxFit.cover,
                        ),
                      ),
                    ),
                    const SizedBox(height: 30),
                    // BEST 배지
                    Container(
                      padding: const EdgeInsets.symmetric(
                        horizontal: 20,
                        vertical: 6,
                      ),
                      decoration: BoxDecoration(
                        color: const Color(0xFF0095FF),
                        borderRadius: BorderRadius.circular(20),
                      ),
                      child: const Text(
                        'BEST',
                        style: TextStyle(
                          color: Colors.white,
                          fontSize: 14,
                          fontWeight: FontWeight.bold,
                          letterSpacing: 1,
                        ),
                      ),
                    ),
                    const SizedBox(height: 16),
                    const Text(
                      '코코볼',
                      style: TextStyle(
                        fontSize: 24,
                        fontWeight: FontWeight.bold,
                        color: Colors.black,
                      ),
                    ),
                    const SizedBox(height: 40),
                  ],
                ),
              ),
            ),
          ),
          const SizedBox(width: 40),
          // 그래놀라 옵션
          Flexible(
            child: GestureDetector(
              onTap: () {
                setState(() {
                  selectedCereal = 'granola';
                  orderData.selectedCereal = 'granola';
                });
              },
              child: Container(
                width: 380,
                height: 500,
                decoration: BoxDecoration(
                  color: Colors.white,
                  borderRadius: BorderRadius.circular(20),
                  border: selectedCereal == 'granola'
                      ? Border.all(color: const Color(0xFF0095FF), width: 4)
                      : Border.all(color: Colors.transparent, width: 4),
                ),
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    const SizedBox(height: 40),
                    // 시리얼 이미지 - 그래놀라
                    Container(
                      width: 200,
                      height: 200,
                      child: ClipRRect(
                        borderRadius: BorderRadius.circular(16),
                        child: Image.asset(
                          'assets/images/menu-2.png',
                          width: 200,
                          height: 200,
                          fit: BoxFit.cover,
                        ),
                      ),
                    ),
                    const SizedBox(height: 30),
                    // NEW 배지
                    Container(
                      padding: const EdgeInsets.symmetric(
                        horizontal: 20,
                        vertical: 6,
                      ),
                      decoration: BoxDecoration(
                        color: const Color(0xFFFF6B9D),
                        borderRadius: BorderRadius.circular(20),
                      ),
                      child: const Text(
                        'NEW',
                        style: TextStyle(
                          color: Colors.white,
                          fontSize: 14,
                          fontWeight: FontWeight.bold,
                          letterSpacing: 1,
                        ),
                      ),
                    ),
                    const SizedBox(height: 16),
                    const Text(
                      '그래놀라',
                      style: TextStyle(
                        fontSize: 24,
                        fontWeight: FontWeight.bold,
                        color: Colors.black,
                      ),
                    ),
                    const SizedBox(height: 40),
                  ],
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}


