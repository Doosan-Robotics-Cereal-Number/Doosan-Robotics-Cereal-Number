import 'package:flutter/material.dart';
import '../models/order_data.dart';
import '../widgets/selection_page_layout.dart';

class CerealSelectionPage extends StatefulWidget {
  const CerealSelectionPage({super.key});

  @override
  State<CerealSelectionPage> createState() => _CerealSelectionPageState();
}

class _CerealSelectionPageState extends State<CerealSelectionPage> {
  String? selectedCereal;
  OrderData? orderData;

  @override
  void initState() {
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    // 이전 화면에서 전달받은 주문 데이터 가져오기
    orderData = ModalRoute.of(context)?.settings.arguments as OrderData?;
    
    // ⭐ orderData가 null이면 새로 생성 (주문 흐름의 시작점)
    if (orderData == null) {
      orderData = OrderData();
      print('[CerealSelectionPage] 새로운 OrderData 생성됨');
    }
    
    // orderData의 selectedCereal과 로컬 selectedCereal 동기화
    if (orderData!.selectedCereal != null) {
      // orderData에 값이 있으면 그것을 사용
      if (selectedCereal != orderData!.selectedCereal) {
        selectedCereal = orderData!.selectedCereal;
        print('[CerealSelectionPage] orderData에서 시리얼 값 동기화: $selectedCereal');
      }
    } else {
      // orderData에 값이 없으면 기본값 설정 (코코볼)
      if (selectedCereal == null) {
        selectedCereal = 'start_sequence_a';
        orderData!.selectedCereal = 'start_sequence_a';
        print('[CerealSelectionPage] 기본값 설정: start_sequence_a (코코볼)');
      }
    }
    
    return SelectionPageLayout(
      title: '어떤 시리얼로 준비할까요?',
      backButtonText: '뒤로가기',
      confirmButtonText: '메뉴 선택하기',
      isConfirmEnabled: selectedCereal != null,
      showAppBar: false,
      onConfirmPressed: () {
        print('[CerealSelectionPage] 메뉴 선택하기 버튼 클릭됨');
        print('📦 [CerealSelectionPage] 전달할 주문 정보:');
        print('   - 시리얼: ${orderData?.selectedCereal}');
        print('   - 양: ${orderData?.selectedQuantity}');
        print('   - 컵: ${orderData?.selectedCup}');
        
        // 마지막으로 selectedCereal과 orderData 동기화 확인
        if (selectedCereal != null && orderData != null) {
          orderData!.selectedCereal = selectedCereal;
          print('✅ [CerealSelectionPage] 최종 동기화: orderData.selectedCereal = $selectedCereal');
        }
        
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
              print('[CerealSelectionPage] 코코볼 선택됨 → start_sequence_a');
              setState(() {
                selectedCereal = 'start_sequence_a';
                if (orderData != null) {
                  orderData!.selectedCereal = 'start_sequence_a';
                  print('✅ [CerealSelectionPage] orderData.selectedCereal 업데이트: ${orderData!.selectedCereal}');
                } else {
                  print('⚠️ [CerealSelectionPage] orderData가 null입니다!');
                }
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
          // 조리퐁 옵션
          GestureDetector(
            onTap: () {
              print('[CerealSelectionPage] 조리퐁 선택됨 → start_sequence_b');
              setState(() {
                selectedCereal = 'start_sequence_b';
                if (orderData != null) {
                  orderData!.selectedCereal = 'start_sequence_b';
                  print('✅ [CerealSelectionPage] orderData.selectedCereal 업데이트: ${orderData!.selectedCereal}');
                } else {
                  print('⚠️ [CerealSelectionPage] orderData가 null입니다!');
                }
              });
            },
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                // 시리얼 이미지 - 조리퐁
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
                  '조리퐁',
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


