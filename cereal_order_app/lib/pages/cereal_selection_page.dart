import 'package:flutter/material.dart';
import '../models/order_data.dart';
import '../widgets/selection_page_layout.dart';

class CerealSelectionPage extends StatefulWidget {
  const CerealSelectionPage({super.key});

  @override
  State<CerealSelectionPage> createState() => _CerealSelectionPageState();
}

class _CerealSelectionPageState extends State<CerealSelectionPage> {
  String? selectedCereal = 'start_sequence_a'; // 기본값으로 코코볼 선택
  OrderData? orderData;

  @override
  void initState() {
    super.initState();
    // 다음 프레임에서 orderData 설정
    WidgetsBinding.instance.addPostFrameCallback((_) {
      setState(() {
        orderData?.selectedCereal = 'start_sequence_a';
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    // 이전 화면에서 전달받은 주문 데이터 가져오기
    orderData = ModalRoute.of(context)?.settings.arguments as OrderData?;
    
    // 첫 빌드에서 orderData가 있으면 기본값 설정
    if (orderData != null && orderData!.selectedCereal == null) {
      orderData!.selectedCereal = 'start_sequence_a';
    }
    
    return SelectionPageLayout(
      title: '어떤 시리얼로 준비할까요?',
      backButtonText: '뒤로가기',
      confirmButtonText: '메뉴 선택하기',
      isConfirmEnabled: selectedCereal != null,
      showAppBar: false,
      onConfirmPressed: () {
        print('[CerealSelectionPage] 메뉴 선택하기 버튼 클릭됨');
        Navigator.pushNamed(
          context,
          '/quantity-selection',
          arguments: orderData,
        );
      },
      onBackPressed: () {
        print('[CerealSelectionPage] 뒤로가기 버튼 클릭됨');
        Navigator.pop(context);
      },
      contentArea: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          // 코코볼 옵션
          GestureDetector(
            onTap: () {
              print('[CerealSelectionPage] 코코볼 선택됨');
              setState(() {
                selectedCereal = 'start_sequence_a';
                orderData?.selectedCereal = 'start_sequence_a';
              });
            },
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                // 시리얼 이미지 - 코코볼
                Container(
                  decoration: BoxDecoration(
                    color: Colors.white,
                    borderRadius: BorderRadius.circular(20),
                    border: Border.all(
                      color: selectedCereal == 'start_sequence_a'
                          ? const Color(0xFF0064FF)
                          : const Color(0xFFF1F1F1),
                      width: 3,
                    ),
                  ),
                  child: ClipRRect(
                    borderRadius: BorderRadius.circular(17),
                    child: Image.asset(
                      'assets/images/menu-1.png',
                      width: 246,
                      height: 246,
                      fit: BoxFit.cover,
                    ),
                  ),
                ),
                const SizedBox(height: 30),
                // BEST 배지
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
                    'BEST',
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
                  '코코볼',
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
          // 그래놀라 옵션
          GestureDetector(
            onTap: () {
              print('[CerealSelectionPage] 그래놀라 선택됨');
              setState(() {
                selectedCereal = 'start_sequence_b';
                orderData?.selectedCereal = 'start_sequence_b';
              });
            },
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                // 시리얼 이미지 - 그래놀라
                Container(
                  decoration: BoxDecoration(
                    color: Colors.white,
                    borderRadius: BorderRadius.circular(20),
                    border: Border.all(
                      color: selectedCereal == 'start_sequence_b'
                          ? const Color(0xFF0064FF)
                          : const Color(0xFFF1F1F1),
                      width: 3,
                    ),
                  ),
                  child: ClipRRect(
                    borderRadius: BorderRadius.circular(17),
                    child: Image.asset(
                      'assets/images/menu-2.png',
                      width: 246,
                      height: 246,
                      fit: BoxFit.cover,
                    ),
                  ),
                ),
                const SizedBox(height: 30),
                // NEW 배지
                Container(
                  padding: const EdgeInsets.symmetric(
                    horizontal: 24,
                    vertical: 8,
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
                      fontWeight: FontWeight.w600,
                      letterSpacing: 0.5,
                    ),
                  ),
                ),
                const SizedBox(height: 16),
                const Text(
                  '그래놀라',
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


