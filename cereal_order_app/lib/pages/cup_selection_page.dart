import 'package:flutter/material.dart';
import '../models/order_data.dart';
import '../widgets/selection_page_layout.dart';

class CupSelectionPage extends StatefulWidget {
  const CupSelectionPage({super.key});

  @override
  State<CupSelectionPage> createState() => _CupSelectionPageState();
}

class _CupSelectionPageState extends State<CupSelectionPage> {
  String? selectedCup = 'store'; // 기본값으로 매장 컵 선택
  OrderData? orderData;

  @override
  void initState() {
    super.initState();
    // 다음 프레임에서 orderData 설정
    WidgetsBinding.instance.addPostFrameCallback((_) {
      setState(() {
        orderData?.selectedCup = 'store';
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    // 이전 화면에서 전달받은 주문 데이터 가져오기
    orderData = ModalRoute.of(context)?.settings.arguments as OrderData?;
    
    // 첫 빌드에서 orderData가 있으면 기본값 설정
    if (orderData != null && orderData!.selectedCup == null) {
      orderData!.selectedCup = 'store';
    }
    
    return SelectionPageLayout(
      title: '어떤 컵에 담아드릴까요?',
      backButtonText: '뒤로가기',
      confirmButtonText: '컵 선택하기',
      isConfirmEnabled: selectedCup != null,
      showAppBar: false,
      onConfirmPressed: () {
        Navigator.pushNamed(
          context,
          '/loading',
          arguments: orderData,
        );
      },
      contentArea: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          // 매장 컵 옵션
          GestureDetector(
            onTap: () {
              setState(() {
                selectedCup = 'store';
                orderData?.selectedCup = 'store';
              });
            },
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                // 컵 이미지 - 매장 컵
                Container(
                  decoration: BoxDecoration(
                    color: Colors.white,
                    borderRadius: BorderRadius.circular(20),
                    border: Border.all(
                      color: selectedCup == 'store'
                          ? const Color(0xFF0064FF)
                          : const Color(0xFFF1F1F1),
                      width: 3,
                    ),
                  ),
                  child: ClipRRect(
                    borderRadius: BorderRadius.circular(17),
                    child: Image.asset(
                      'assets/images/cup-1.png',
                      width: 246,
                      height: 246,
                      fit: BoxFit.cover,
                    ),
                  ),
                ),
                const SizedBox(height: 30),
                // BASIC 배지
                Container(
                  padding: const EdgeInsets.symmetric(
                    horizontal: 24,
                    vertical: 8,
                  ),
                  decoration: BoxDecoration(
                    color: const Color(0xFF0064FF),
                    borderRadius: BorderRadius.circular(20),
                  ),
                  child: const Text(
                    'BASIC',
                    style: TextStyle(
                      color: Colors.white,
                      fontSize: 14,
                      fontWeight: FontWeight.w600,
                      letterSpacing: 0.5,
                    ),
                  ),
                ),
                const SizedBox(height: 16),
                const Text(
                  '매장 컵',
                  style: TextStyle(
                    fontSize: 22,
                    fontWeight: FontWeight.w600,
                    color: Colors.black,
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(width: 60),
          // 개인 컵 옵션
          GestureDetector(
            onTap: () {
              setState(() {
                selectedCup = 'personal';
                orderData?.selectedCup = 'personal';
              });
            },
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                // 컵 이미지 - 개인 컵
                Container(
                  decoration: BoxDecoration(
                    color: Colors.white,
                    borderRadius: BorderRadius.circular(20),
                    border: Border.all(
                      color: selectedCup == 'personal'
                          ? const Color(0xFF0064FF)
                          : const Color(0xFFF1F1F1),
                      width: 3,
                    ),
                  ),
                  child: ClipRRect(
                    borderRadius: BorderRadius.circular(17),
                    child: Image.asset(
                      'assets/images/cup-2.png',
                      width: 246,
                      height: 246,
                      fit: BoxFit.cover,
                    ),
                  ),
                ),
                const SizedBox(height: 30),
                // ECO 배지
                Container(
                  padding: const EdgeInsets.symmetric(
                    horizontal: 24,
                    vertical: 8,
                  ),
                  decoration: BoxDecoration(
                    color: const Color(0xFF10B981),
                    borderRadius: BorderRadius.circular(20),
                  ),
                  child: const Text(
                    'ECO',
                    style: TextStyle(
                      color: Colors.white,
                      fontSize: 14,
                      fontWeight: FontWeight.w600,
                      letterSpacing: 0.5,
                    ),
                  ),
                ),
                const SizedBox(height: 16),
                const Text(
                  '개인 컵',
                  style: TextStyle(
                    fontSize: 22,
                    fontWeight: FontWeight.w600,
                    color: Colors.black,
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}


