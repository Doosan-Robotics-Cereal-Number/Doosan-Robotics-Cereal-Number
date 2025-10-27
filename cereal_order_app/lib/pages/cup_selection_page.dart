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
          '/quantity-selection',
          arguments: orderData,
        );
      },
      contentArea: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          // 매장 컵 옵션
          Flexible(
            child: GestureDetector(
              onTap: () {
                setState(() {
                  selectedCup = 'store';
                  orderData?.selectedCup = 'store';
                });
              },
              child: Container(
                width: 380,
                height: 500,
                decoration: BoxDecoration(
                  color: Colors.white,
                  borderRadius: BorderRadius.circular(20),
                  border: selectedCup == 'store'
                      ? Border.all(color: const Color(0xFF0095FF), width: 4)
                      : Border.all(color: Colors.transparent, width: 4),
                ),
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    const SizedBox(height: 40),
                    // 컵 이미지 - 매장 컵
                    Container(
                      width: 200,
                      height: 200,
                      child: ClipRRect(
                        borderRadius: BorderRadius.circular(16),
                        child: Image.asset(
                          'assets/images/cup-1.png',
                          width: 200,
                          height: 200,
                          fit: BoxFit.cover,
                        ),
                      ),
                    ),
                    const SizedBox(height: 30),
                    // BASIC 배지
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
                        'BASIC',
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
                      '매장 컵',
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
          // 개인 컵 옵션
          Flexible(
            child: GestureDetector(
              onTap: () {
                setState(() {
                  selectedCup = 'personal';
                  orderData?.selectedCup = 'personal';
                });
              },
              child: Container(
                width: 380,
                height: 500,
                decoration: BoxDecoration(
                  color: Colors.white,
                  borderRadius: BorderRadius.circular(20),
                  border: selectedCup == 'personal'
                      ? Border.all(color: const Color(0xFF0095FF), width: 4)
                      : Border.all(color: Colors.transparent, width: 4),
                ),
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    const SizedBox(height: 40),
                    // 컵 이미지 - 개인 컵
                    Container(
                      width: 200,
                      height: 200,
                      child: ClipRRect(
                        borderRadius: BorderRadius.circular(16),
                        child: Image.asset(
                          'assets/images/cup-2.png',
                          width: 200,
                          height: 200,
                          fit: BoxFit.cover,
                        ),
                      ),
                    ),
                    const SizedBox(height: 30),
                    // ECO 배지
                    Container(
                      padding: const EdgeInsets.symmetric(
                        horizontal: 20,
                        vertical: 6,
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
                          fontWeight: FontWeight.bold,
                          letterSpacing: 1,
                        ),
                      ),
                    ),
                    const SizedBox(height: 16),
                    const Text(
                      '개인 컵',
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


