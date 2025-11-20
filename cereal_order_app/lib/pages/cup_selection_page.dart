import 'package:flutter/material.dart';
import '../models/order_data.dart';
import '../widgets/selection_page_layout.dart';

class CupSelectionPage extends StatefulWidget {
  const CupSelectionPage({super.key});

  @override
  State<CupSelectionPage> createState() => _CupSelectionPageState();
}

class _CupSelectionPageState extends State<CupSelectionPage> {
  String? selectedCup = '매장컵'; // 기본값으로 매장 컵 선택
  OrderData? orderData;

  @override
  void initState() {
    super.initState();
    // 다음 프레임에서 orderData 설정
    WidgetsBinding.instance.addPostFrameCallback((_) {
      setState(() {
        orderData?.selectedCup = '매장컵';
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    // 이전 화면에서 전달받은 주문 데이터 가져오기
    orderData = ModalRoute.of(context)?.settings.arguments as OrderData?;
    
    if (orderData == null) {
      print('⚠️ [CupSelectionPage] orderData가 null입니다!');
    }
    
    // 첫 빌드에서 orderData가 있으면 기본값 설정
    if (orderData != null && orderData!.selectedCup == null) {
      orderData!.selectedCup = '매장컵';
      print('[CupSelectionPage] 기본값 설정: 매장컵');
    }
    
    return SelectionPageLayout(
      title: '어떤 컵에 담아드릴까요?',
      backButtonText: '뒤로가기',
      confirmButtonText: '컵 선택하기',
      isConfirmEnabled: selectedCup != null,
      showAppBar: false,
      onConfirmPressed: () {
        print('[CupSelectionPage] 컵 선택하기 버튼 클릭됨');
        print('📦 [CupSelectionPage] 전달할 주문 정보:');
        print('   - 시리얼: ${orderData?.selectedCereal}');
        print('   - 양: ${orderData?.selectedQuantity}');
        print('   - 컵: ${orderData?.selectedCup}');
        Navigator.pushNamed(
          context,
          '/loading',
          arguments: orderData,
        );
      },
      onBackPressed: () {
        print('[CupSelectionPage] 뒤로가기 버튼 클릭됨');
        Navigator.pop(context);
      },
      contentArea: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          // 매장 컵 옵션
          GestureDetector(
            onTap: () {
              print('[CupSelectionPage] 매장 컵 선택됨');
              setState(() {
                selectedCup = '매장컵';
                orderData!.selectedCup = '매장컵';
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
                      color: selectedCup == '매장컵'
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
              print('[CupSelectionPage] 개인 컵 선택됨');
              setState(() {
                selectedCup = '개인컵';
                orderData!.selectedCup = '개인컵';
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
                      color: selectedCup == '개인컵'
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


